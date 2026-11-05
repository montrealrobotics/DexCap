import json
import logging
import socket
import threading
import time
from argparse import ArgumentParser, ArgumentTypeError
import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as pb
from importlib.resources import files

import yaml
from dexcap.sensors.rigidbodySento import create_primitive_shape
from dexcap.sensors.rokoko_module import RokokoModule
from dexcap.sensors.cameras.usb_camera_module import MultiCameraWrapper
from dexcap.sensors.quest_teleop_module import QuestTeleopModule
from dexcap.utils.ik_solver import LeapHandIKSolver, ParallelGripperIKSolver
import os
import h5py

# Robot deployment imports
import redis
import pickle
from deoxys.robot_interfaces.xarm_interface import XArmInterface
from deoxys.robot_interfaces.utils.utils import ActionType
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from dexcap.utils.utils import check_connection, get_logger, StatusCode, extract_img_observation, resize_with_pad, \
    convert_to_hardware
from dexcap.utils.status import StatusBoard, BoardLogHandler, RateLimitedWarning, start_status_server

logger = get_logger("teleop_server")
warn_every = RateLimitedWarning(logger, period=5.0)

# left_camera_id: str = "29712701"
# right_camera_id: str = "23697076"
# wrist_camera_id: str = "12715737"

left_camera_id: str = "0"
right_camera_id: str = "2"
wrist_camera_id: str = "4"

def init_robot(robot_interface, arm_start_joints, position_controller_cfg):

    robot_interface._state_buffer = []
    for _ in range(10):
        robot_interface.control(
            controller_type="JOINT_POSITION",
            action=arm_start_joints,
            action_type=ActionType.move_to,
            controller_cfg=position_controller_cfg,
        )
        time.sleep(0.5)
    logger.info("Robot initial position sent")
    return

def init_leaphand(redis_client):
    hand_target = [0.0 for _ in range(16)]
    redis_client.set('right_leap_action', pickle.dumps(convert_to_hardware(hand_target)))


class QuestConnection:
    """Logs the headset connection when it changes (waiting, receiving, robot placed, lost) instead of
    repeating a warning every few seconds. The current state also goes on the status page."""

    def __init__(self, board, quest, laptop_ip, lost_after=3.0, remind_every=30.0):
        self.board = board
        self.quest = quest
        self.lost_after = lost_after
        self.remind_every = remind_every
        self.text = {
            "waiting": f"Start the ARCap app in the headset and enter this laptop's IP ({laptop_ip})",
            # The app only streams poses after the robot is placed, so poses without a WorldFrame mean
            # the app was opened before this server was listening and has to be restarted
            "receiving": "Restart the ARCap app in the headset: it was opened before the teleop server was ready",
            "placed": "Robot placed: press A in the headset to record",
            "lost": "Lost the Quest stream: is the app still open and the headset awake?",
        }
        self.state = None
        self.last_reminder = time.time()
        self.told_restart = False
        self.world_frame_ts = None
        self._set("waiting", logging.INFO, self.text["waiting"])

    def _set(self, state, level, message):
        self.state = state
        self.board.set_info(quest=self.text[state])
        logger.log(level, message)

    def update(self, status):
        age = self.board.streams["quest_in"].age()
        if age is None or age > self.lost_after:
            if self.state in ("receiving", "placed"):
                self._set("lost", logging.WARNING, self.text["lost"])
                self.last_reminder = time.time()
            elif time.time() - self.last_reminder > self.remind_every:
                self.last_reminder = time.time()
                logger.info("Still waiting for the Quest. " + self.text[self.state])
            return
        if self.state in ("waiting", "lost"):
            self._set("receiving", logging.INFO,
                      "Receiving data from the Quest" if self.state == "waiting" else "Quest stream back")
        if self.quest.wf_receive_ts != self.world_frame_ts and self.quest.world_frame is not None:
            self.world_frame_ts = self.quest.wf_receive_ts
            self.told_restart = False
            self._set("placed", logging.INFO, self.text["placed"])
        elif self.quest.world_frame is not None and self.state != "placed":
            self.state = "placed"  # stream came back after a gap; the robot placement is unchanged
            self.board.set_info(quest=self.text["placed"])
        elif status == StatusCode.APP_RESTART and not self.told_restart:
            # Hand poses before any WorldFrame: the app was started before the teleop server
            self.told_restart = True
            logger.warning(self.text["receiving"])


def str2bool(value):
    """Lets boolean flags be given as `--flag`, `--flag True` or `--flag False`."""
    if isinstance(value, bool):
        return value
    if value.lower() in ("true", "1", "yes", "y"):
        return True
    if value.lower() in ("false", "0", "no", "n"):
        return False
    raise ArgumentTypeError(f"expected True or False, got {value!r}")


def watch_hand_tuning(path, rokoko, board, period=0.5):
    """Apply the launcher's finger length (a JSON file it rewrites) to the glove targets while running."""
    last_mtime = None
    while True:
        try:
            mtime = os.path.getmtime(path)
            if mtime != last_mtime:
                last_mtime = mtime
                with open(path) as f:
                    factor = float(json.load(f)["finger_length"])
                if 0.5 <= factor <= 3.0 and factor != rokoko.finger_length:
                    rokoko.set_finger_length(factor)
                    logger.info(f"Finger length set to {factor * 100:.0f}%")
                board.set_info(finger_length=f"{rokoko.finger_length * 100:.0f}%")
        except (OSError, ValueError, KeyError, TypeError):
            pass  # missing or half-written file: keep the current length and try again
        time.sleep(period)


def monitor_robot(board, robot_interface=None, redis_client=None, period=0.2):
    """Background liveness checks for the arm (deoxys state stream) and the LEAP hand (redis)."""
    arm_stats = board.stream("arm_state", "Arm state (deoxys)") if robot_interface is not None else None
    leap_stats = board.stream("leap_redis", "LEAP hand redis", warn_after=1.5, dead_after=5.0) if redis_client is not None else None
    last_arm_ts = None
    last_ping = 0.0
    while True:
        if arm_stats is not None:
            try:
                state = robot_interface.last_state()
            except Exception as e:
                state = None
                arm_stats.set_detail(last_error=repr(e))
            # A changing state means deoxys is alive: the xArm state carries the publisher's timestamp,
            # the Franka state its measured joint angles, which always move slightly
            if isinstance(state, dict):
                stamp, joints = state.get("timestamp"), state.get("joint_positions")
            else:
                joints = getattr(state, "q", None)
                stamp = tuple(joints) if joints is not None else None
            if state is not None and stamp is not None and stamp != last_arm_ts:
                last_arm_ts = stamp
                arm_stats.mark(joints=np.round(np.asarray(joints), 2).tolist())
        if leap_stats is not None and time.time() - last_ping > 1.0:
            last_ping = time.time()
            try:
                redis_client.ping()
                leap_stats.mark()
            except Exception as e:
                leap_stats.set_detail(last_error=repr(e))
        time.sleep(period)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--frequency", type=int, default=30)
    parser.add_argument("--real_robot", type=str2bool, nargs="?", const=True, default=True)
    parser.add_argument("--use_gloves", type=str2bool, nargs="?", const=True, default=False)
    # --use_arm False with --real_robot True drives only the LEAP hand, e.g. to debug the hand on its own
    parser.add_argument("--use_arm", type=str2bool, nargs="?", const=True, default=True)
    # --use_hand False with --real_robot True leaves the LEAP hand alone, e.g. to test the arm on its own
    parser.add_argument("--use_hand", type=str2bool, nargs="?", const=True, default=True)
    parser.add_argument("--use_camera", type=str2bool, nargs="?", const=True, default=False,
                        help="Record the cameras set at the top of this file (or use --wrist_camera etc.)")
    # Camera device numbers (OpenCV index, /dev/videoN); give the ones to record, leave out the others
    parser.add_argument("--wrist_camera", type=str, default=None)
    parser.add_argument("--left_camera", type=str, default=None)
    parser.add_argument("--right_camera", type=str, default=None)
    # Tactile sensors (e.g. GelSight Mini on the gripper fingers), also by device number
    parser.add_argument("--tactile_left", type=str, default=None)
    parser.add_argument("--tactile_right", type=str, default=None)
    parser.add_argument("--robot_arm", type=str, default="xarm")
    parser.add_argument("--gripper_type", type=str, default="xarm_g")
    parser.add_argument("--vr_host", type=str, default=None, help="Quest IP, overrides TELEOP.VR_HOST in the config")
    parser.add_argument("--laptop_ip", type=str, default=None,
                        help="This laptop's IP, shown as the one to enter in the headset app; overrides LOCAL_HOST.IP_WIFI")
    parser.add_argument("--arm_host", type=str, default=None,
                        help="Where the arm controller runs, overrides CTRL_HOST.IP_ETH (host.docker.internal = "
                             "a container on this laptop)")
    parser.add_argument("--status_port", type=int, default=8080, help="Port for the status page (0 to disable)")
    parser.add_argument("--data_dir", type=str, default="data", help="Folder recordings are saved in")
    # Local ports fed by scripts/udp_relay.py, which keeps the Docker-published ports open across restarts
    parser.add_argument("--pose_port", type=int, default=None, help="Overrides TELEOP.POSE_CMD_PORT (Quest poses in)")
    parser.add_argument("--rokoko_port", type=int, default=None, help="Overrides TELEOP.ROKOKO_PORT (Rokoko stream in)")
    parser.add_argument("--hand_tuning", type=str, default=None,
                        help="JSON file with {\"finger_length\": factor}, reread while running (written by the launcher)")
    parser.add_argument("--hand_mapping", choices=["fingertips", "geort"], default="fingertips",
                        help="LEAP hand from the glove: fingertip IK, or a trained GeoRT model (--geort_model)")
    parser.add_argument("--geort_model", type=str, default=None,
                        help="GeoRT model folder, e.g. /workspace/GeoRT/checkpoint/leap_right_<date>_<name>")
    args = parser.parse_args()

    board = StatusBoard()
    logger.addHandler(BoardLogHandler(board))
    if args.status_port:
        start_status_server(board, port=args.status_port)
        logger.info(f"Status page on http://localhost:{args.status_port}")
    c = pb.connect(pb.GUI)
    vis_sp = []
    hand_ctrl = args.use_gloves
    real_robot = args.real_robot
    use_arm = real_robot and args.use_arm
    use_leap = real_robot and args.use_hand and args.gripper_type == "leap"
    robot_arm = args.robot_arm
    gripper = args.gripper_type
    ip_config_file = files("dexcap").joinpath("configs/" + robot_arm + '.yaml')
    ip_config = YamlConfig(str(ip_config_file)).as_easydict()
    arm_config_file = files("dexcap").joinpath("configs/robot_config/" + robot_arm + '_arm.yaml')
    arm_config = YamlConfig(str(arm_config_file)).as_easydict()
    if args.vr_host:
        ip_config.TELEOP.VR_HOST = args.vr_host
    if args.laptop_ip:
        ip_config.LOCAL_HOST.IP_WIFI = args.laptop_ip
    if args.arm_host:
        ip_config.CTRL_HOST.IP_ETH = args.arm_host
    if args.pose_port:
        ip_config.TELEOP.POSE_CMD_PORT = args.pose_port
    if args.rokoko_port:
        ip_config.TELEOP.ROKOKO_PORT = args.rokoko_port
    # The LEAP hand runs on the control computer; without CONTROL_COMPUTER_IP that is the arm control host
    hand_ip = ip_config.CTRL_HOST.get("CONTROL_COMPUTER_IP", ip_config.CTRL_HOST.IP_ETH)
    board.set_info(stage="starting", arm=robot_arm, gripper=gripper,
                   gloves="on" if args.use_gloves else "off", vr_host=ip_config.TELEOP.VR_HOST)

    camera_type = "zed"
    camera_kwargs={}
    camera_reader = None
    tactile_ids = {}  # "tactile_left"/"tactile_right" -> device number, for the sensors that were found
    video_names = {}  # device number -> role, used to name each recording's full-resolution videos
    right_cam = False
    left_cam = False
    wrist_cam = False

    arm_start_joints = arm_config['fixed_joints']
    impedance_controller_cfg = YamlConfig(arm_config['impedance_controller_cfg']).as_easydict()
    position_controller_cfg = YamlConfig(arm_config['position_controller_cfg']).as_easydict()

    c_code = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1]]
    for i in range(4):
        vis_sp.append(create_primitive_shape(pb, 0.1, pb.GEOM_SPHERE, [0.02], color=c_code[i]))

    robot_interface = None
    redis_client = None
    board.set_info(stage="waiting for VR headset")
    check_connection("VR_headset", ip_config.TELEOP.VR_HOST)
    if use_arm:
        board.set_info(stage="waiting for arm control host")
        check_connection("Arm_control", ip_config.CTRL_HOST.IP_ETH)
        if robot_arm == "xarm":
            if gripper == "xarm_g":
                robot_interface = XArmInterface(general_cfg=ip_config, has_gripper=True, control_freq=args.frequency)
                arm_start_joints.append(arm_config['gripper_init'][0])
            else:
                robot_interface = XArmInterface(general_cfg=ip_config, has_gripper=False, control_freq=args.frequency)
        else:
            robot_interface = FrankaInterface(general_cfg=ip_config, use_visualizer=False, has_gripper=False, control_freq=args.frequency)

        board.set_info(stage="moving arm to start pose")
        init_robot(robot_interface, arm_start_joints, position_controller_cfg)
    elif real_robot:
        logger.warning("Arm disabled (--use_arm False): only the LEAP hand is commanded")
    if real_robot and gripper == "leap" and not use_leap:
        logger.warning("LEAP hand disabled (--use_hand False): only the arm is commanded")
    if real_robot:
        if use_leap:
            board.set_info(stage="waiting for LEAP hand host")
            check_connection("LEAP_hand", hand_ip)
            redis_client = redis.Redis(hand_ip, port=ip_config.CTRL_HOST.HAND_PORT, db=0, socket_timeout=1.0)
            init_leaphand(redis_client)

    # Cameras and tactile sensors are recorded in every mode, with or without the arm or hand
    camera_ids = {"wrist": args.wrist_camera, "left": args.left_camera, "right": args.right_camera}
    if args.use_camera and not any(camera_ids.values()):
        camera_ids = {"wrist": wrist_camera_id, "left": left_camera_id, "right": right_camera_id}
    camera_ids.update({"tactile_left": args.tactile_left, "tactile_right": args.tactile_right})
    camera_ids = {name: str(cid) for name, cid in camera_ids.items() if cid not in (None, "")}
    if camera_ids:
        board.set_info(stage="opening cameras")
        # Only open the cameras that are used
        camera_reader = MultiCameraWrapper(camera_kwargs, device_ids=[int(cid) for cid in camera_ids.values()])
        found = {name: cid for name, cid in camera_ids.items() if cid in camera_reader.camera_dict}
        for name, cid in camera_ids.items():
            if name not in found:
                logger.warning(f"{name.replace('_', ' ').capitalize()} camera (device {cid}, /dev/video{cid}) "
                               "not found: its images won't be recorded")
        tactile_ids = {name: cid for name, cid in found.items() if name.startswith("tactile_")}
        video_names = {cid: name for name, cid in found.items()}
        wrist_cam, left_cam, right_cam = "wrist" in found, "left" in found, "right" in found
        wrist_camera_id = camera_ids.get("wrist", "")
        left_camera_id = camera_ids.get("left", "")
        right_camera_id = camera_ids.get("right", "")
        board.set_info(cameras=", ".join(f"{name} {cid}" + ("" if name in found else " (not found)")
                                         for name, cid in camera_ids.items()))
        camera_reader.start_background()  # cameras decode in parallel; the loop takes the latest frames

    if real_robot:
        threading.Thread(target=monitor_robot, args=(board, robot_interface, redis_client), daemon=True).start()

    if hand_ctrl:
        rokoko = RokokoModule(ip_config)
        rokoko.start_background(board.stream("rokoko", "Rokoko gloves"))
        if args.hand_tuning:
            threading.Thread(target=watch_hand_tuning, args=(args.hand_tuning, rokoko, board), daemon=True).start()

    geort = None
    if hand_ctrl and gripper == "leap" and args.hand_mapping == "geort":
        from dexcap.utils.geort_mapping import GeoRTHand
        geort = GeoRTHand(args.geort_model)
        logger.info(f"Hand mapping: GeoRT model {os.path.basename(args.geort_model.rstrip('/'))} ({geort.device})")
    board.set_info(hand_mapping=f"GeoRT {os.path.basename(args.geort_model.rstrip('/'))}" if geort else "fingertip targets")

    if gripper == "leap":
        solver = LeapHandIKSolver(arm_config)
    else:
        solver = ParallelGripperIKSolver(arm_config)
    quest = QuestTeleopModule(ip_config, solver=solver, timeout=1, data_root=args.data_dir)
    quest.stats_in = board.stream("quest_in", "Quest pose in")
    quest.stats_out = board.stream("ik_out", "IK results to Quest")
    robot_cmd_stats = board.stream("robot_cmd", "Robot commands (while recording)")

    start_time = time.time()
    fps_counter = 0
    packet_counter = 0
    logger.info("Initialization completed")
    current_ts = time.time()
    quest_conn = QuestConnection(board, quest, ip_config.LOCAL_HOST.get("IP_WIFI", "this laptop"))
    no_arm_ctrl = False

    recording = False
    traj_count = 0
    board.set_info(stage="running", recording="no", trajectories=0, data_dir=os.path.abspath(args.data_dir))
    while True:
        # TODO: May cause communication issues, need to tune on AR side.
        wait = 1 / args.frequency - (time.time() - current_ts)
        if wait > 0:
            time.sleep(wait)
        current_ts = time.time()
        try:
            status, right_wrist, head_pose= quest.receive()
            quest_conn.update(status)

            gloves_ok = True
            if hand_ctrl:
                glove = rokoko.latest(max_age=0.5)
                if glove is None:
                    gloves_ok = False
                    age = board.streams["rokoko"].age()
                    warn_every("rokoko", "No Rokoko data " + ("received yet" if age is None else f"for {age:.1f}s") +
                               ": the robot won't move until glove data arrives. Check Rokoko Studio "
                               "streaming and the gloves' WiFi")
                elif not rokoko.hands_present[1]:
                    gloves_ok = False
                    warn_every("rokoko_right", "Right glove missing from the Rokoko stream: the robot won't move "
                                               "until it is back")
                else:
                    left_positions, right_positions = glove
                    rokoko.send_joint_data(np.vstack([left_positions, right_positions]))
                    frozen = rokoko.frozen_for(1)
                    if frozen is not None and frozen > 2.0:
                        warn_every("rokoko_frozen", f"Right glove data unchanged for {frozen:.0f}s, is the glove powered on?")

            if status in (StatusCode.SOCKET_TIMEOUT, StatusCode.APP_RESTART):
                pass  # reported by quest_conn
            elif status == StatusCode.SUCCESS and not gloves_ok:
                pass
            elif status == StatusCode.SUCCESS:
                if right_wrist is not None:
                    right_wrist_pos, right_wrist_rot = right_wrist
                    right_wrist_orn = Rotation.from_quat(right_wrist_rot)
                    head_pos = head_pose[0]
                    head_orn = Rotation.from_quat(head_pose[1])
                    if geort is not None:
                        right_arm_q, right_hand_q = solver.solve_system_world(
                            right_wrist_pos, right_wrist_orn, hand_q=geort.forward(rokoko.right_points))
                    elif hand_ctrl:
                        hand_tip_pose = right_wrist_orn.apply(right_positions) + right_wrist_pos
                        hand_tip_pose = hand_tip_pose[[1,2,3,0]]
                        right_arm_q, right_hand_q = solver.solve_system_world(right_wrist_pos, right_wrist_orn, hand_tip_pose)
                    else:
                        right_arm_q, right_hand_q = solver.solve_system_world(right_wrist_pos, right_wrist_orn)
                    quest.send_ik_result(right_arm_q, right_hand_q)
                    obs_dict = {"timestamp": {}}
                    if quest.data_dir is not None:
                        if not recording:
                            recording = True
                            board.set_info(recording="yes")
                            logger.info(f"Recording started -> {quest.data_dir}")
                            current_actions = []
                            current_joint_pos = []
                            current_ee_pos = []
                            current_ee_quat = []
                            current_gripper_qpos = []
                            current_agentview_right_imgs, current_agentview_left_imgs, current_eye_in_hand_imgs = [], [], []
                            current_tactile_imgs = {name: [] for name in tactile_ids}
                            if camera_reader is not None:
                                # Full-resolution video of every camera, next to this recording's traj.hdf5
                                camera_reader.start_recording(quest.data_dir, video_names)

                        if real_robot:
                            if robot_interface is None:
                                pass  # hand only: no arm command
                            elif robot_arm == "xarm":
                                if gripper == 'xarm_g':
                                    if no_arm_ctrl:
                                        fixed_q = np.array(arm_config['fixed_joints'][:-1], dtype=np.float64)
                                        gripper_q = np.array([right_hand_q[0]], dtype=np.float64)
                                        robot_q = np.concatenate([fixed_q, gripper_q])
                                    else:
                                        robot_q = np.concatenate([right_arm_q, right_hand_q])
                                else:
                                    # LEAP hand is driven over redis, not through the arm interface,
                                    # so the arm command is just the 6 arm joints.
                                    robot_q = np.asarray(right_arm_q, dtype=np.float64)
                                robot_interface.control(controller_type="JOINT_POSITION", action=robot_q, action_type=ActionType.delta)

                            else:
                                robot_interface.control(controller_type="JOINT_IMPEDANCE", action=right_arm_q, controller_cfg=impedance_controller_cfg)

                            if redis_client is not None:
                                redis_client.set('right_leap_action', pickle.dumps(convert_to_hardware(right_hand_q)))
                            robot_cmd_stats.mark()

                        action_vec = np.concatenate([right_arm_q, right_hand_q])
                        current_actions.append(action_vec)

                        if robot_interface is not None:
                            robot_last_state = robot_interface.last_state()

                            if robot_arm == "xarm":
                                joint_positions = np.array(robot_last_state["joint_positions"], dtype=np.float32)
                                ee_pose = np.array(robot_last_state["ee_pos"], dtype=np.float32)
                                ee_quat = np.array(robot_last_state["ee_quat"], dtype=np.float32)
                                gripper_qpos = np.array(robot_last_state["gripper_pos"], dtype=np.float32)
                            else:
                                joint_positions = np.array(robot_last_state.q, dtype=np.float32)
                                ee_quat, ee_pose = robot_interface.last_eef_quat_and_pos()
                                ee_pose = np.array(ee_pose, dtype=np.float32)
                                ee_quat = np.array(ee_quat, dtype=np.float32)

                                if gripper == "leap":
                                    gripper_qpos = np.array(right_hand_q, dtype=np.float32)
                                else:
                                    gripper_qpos = np.array(robot_interface.last_gripper_q(), dtype=np.float32)

                            current_joint_pos.append(joint_positions)
                            current_ee_pos.append(ee_pose)
                            current_ee_quat.append(ee_quat)
                            current_gripper_qpos.append(gripper_qpos)

                        else:
                            current_joint_pos.append(np.array(right_arm_q, dtype=np.float32))
                            current_ee_pos.append(np.array(right_wrist_pos, dtype=np.float32))
                            current_ee_quat.append(np.array(right_wrist[1], dtype=np.float32))
                            current_gripper_qpos.append(np.array(right_hand_q, dtype=np.float32))

                        if camera_reader is not None:
                            camera_obs, camera_timestamp = camera_reader.read_cameras()
                            obs_dict.update(camera_obs)
                            obs_dict["timestamp"]["cameras"] = camera_timestamp
                            image_observations = obs_dict.get("image", {})  # empty if every camera missed this frame
                            img_obs = extract_img_observation(
                                image_observations, left_camera_id, right_camera_id, wrist_camera_id
                            )

                            # Process and collect images (a camera can miss a frame now and then)
                            for flag, key, frames in ((wrist_cam, "wrist_image", current_eye_in_hand_imgs),
                                                      (left_cam, "left_image", current_agentview_left_imgs),
                                                      (right_cam, "right_image", current_agentview_right_imgs)):
                                if not flag:
                                    continue
                                if img_obs[key] is None:
                                    warn_every(key, f"No frame from the {key.split('_')[0]} camera, skipping it")
                                    continue
                                frames.append(resize_with_pad(img_obs[key], 224, 224))

                            # Tactile sensors are single-lens cameras: one image per device, BGR like the others
                            for name, cid in tactile_ids.items():
                                frame = image_observations.get(cid)
                                if frame is None:
                                    frame = image_observations.get(f"{cid}_left")
                                if frame is None:
                                    warn_every(name, f"No frame from the {name.replace('_', ' ')} sensor, skipping it")
                                    continue
                                current_tactile_imgs[name].append(resize_with_pad(frame[..., :3][..., ::-1], 224, 224))

                    else:
                        if recording:
                            recording = False
                            board.set_info(recording="no")
                            if camera_reader is not None:
                                camera_reader.stop_recording()  # closes the videos so they are complete
                            if len(current_actions) > 0:
                                traj_count += 1
                                hdf5_path = os.path.join(quest.prev_data_dir, "traj.hdf5")

                                with h5py.File(hdf5_path, "a") as f:
                                    data_grp = f.require_group("data")
                                    ep_grp = data_grp.create_group(f"traj_{traj_count}")
                                    ep_grp.create_dataset("actions", data=np.array(current_actions, dtype=np.float32))

                                    obs_grp = ep_grp.create_group("obs")
                                    obs_grp.create_dataset("robot0_joint_pos", data=np.array(current_joint_pos, dtype=np.float32))
                                    obs_grp.create_dataset("robot0_eef_pos", data=np.array(current_ee_pos, dtype=np.float32))
                                    obs_grp.create_dataset("robot0_eef_quat", data=np.array(current_ee_quat, dtype=np.float32))
                                    obs_grp.create_dataset("robot0_gripper_qpos", data=np.array(current_gripper_qpos, dtype=np.float32))

                                    if camera_reader is not None:
                                        if len(current_eye_in_hand_imgs) > 0:
                                            obs_grp.create_dataset(
                                                "robot0_eye_in_hand",
                                                data=np.array(current_eye_in_hand_imgs, dtype=np.uint8),
                                                compression="gzip",
                                            )

                                        if left_cam and len(current_agentview_left_imgs) > 0:
                                            obs_grp.create_dataset(
                                                "robot0_agentview_left",
                                                data=np.array(current_agentview_left_imgs, dtype=np.uint8),
                                                compression="gzip",
                                            )

                                        if right_cam and len(current_agentview_right_imgs) > 0:
                                            obs_grp.create_dataset(
                                                "robot0_agentview_right",
                                                data=np.array(current_agentview_right_imgs, dtype=np.uint8),
                                                compression="gzip",
                                            )

                                        for name, frames in current_tactile_imgs.items():
                                            if frames:
                                                obs_grp.create_dataset(f"robot0_{name}", data=np.array(frames, dtype=np.uint8),
                                                                       compression="gzip")

                                logger.info(f"Trajectory saved ({len(current_actions)} steps) -> {hdf5_path} [traj_{traj_count}]")
                                board.set_info(trajectories=traj_count, last_saved=hdf5_path)
                                current_eye_in_hand_imgs.clear()
                                current_agentview_left_imgs.clear()
                                current_agentview_right_imgs.clear()
                                for frames in current_tactile_imgs.values():
                                    frames.clear()
                                time.sleep(1.0)
                                if robot_interface is not None:
                                    init_robot(robot_interface, arm_start_joints, position_controller_cfg)


        except socket.error as e:
            logger.error(e)
            pass
        except (ValueError, IndexError) as e:
            # Malformed packet (e.g. headset app version mismatch); skip it instead of crashing
            logger.error(f"Skipped bad packet: {e!r}")
        except KeyboardInterrupt:
            #camera.close()
            if hand_ctrl:
                rokoko.close()
            # quest.close()
            if robot_interface is not None:
                robot_interface.close()
            break
        else:
            packet_time = time.time()
            fps_counter += 1
            packet_counter += 1

            if (packet_time - start_time) > 1.0:
                logger.debug(f"received {fps_counter} packets in a second")
                start_time += 1.0
                fps_counter = 0

