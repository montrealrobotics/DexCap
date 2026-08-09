import socket
import time
from argparse import ArgumentParser
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
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from dexcap.utils.utils import check_connection, get_logger, StatusCode, extract_img_observation, resize_with_pad

logger = get_logger("teleop_server")

# left_camera_id: str = "29712701"
# right_camera_id: str = "23697076"
# wrist_camera_id: str = "12715737"

left_camera_id: str = "0"
right_camera_id: str = "1"
wrist_camera_id: str = "2"

def convert_to_hardware(joint_angles):
    real_right_robot_hand_q = np.zeros(16)
    real_right_robot_hand_q[0:16] = joint_angles[0:16]
    real_right_robot_hand_q[0:2] = real_right_robot_hand_q[0:2][::-1]
    real_right_robot_hand_q[4:6] = real_right_robot_hand_q[4:6][::-1]
    real_right_robot_hand_q[8:10] = real_right_robot_hand_q[8:10][::-1]
    real_right_robot_hand_q[:16] += np.pi
    return real_right_robot_hand_q.tolist()


def init_robot(robot_interface, arm_start_joints, position_controller_cfg):

    robot_interface._state_buffer = []
    for _ in range(10):
        robot_interface.control(
            controller_type="JOINT_POSITION",
            action=arm_start_joints,
            controller_cfg=position_controller_cfg,
        )
        time.sleep(0.5)
    logger.info("Robot initial position sent")
    return

def init_leaphand(redis_client):
    hand_target = [0.0 for _ in range(16)]
    redis_client.set('right_leap_action', pickle.dumps(convert_to_hardware(hand_target)))


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--frequency", type=int, default=30)
    parser.add_argument("--real_robot", type=bool, default=True)
    parser.add_argument("--use_gloves", type=bool, default=True)
    parser.add_argument("--use_camera", type=bool, default=True)
    parser.add_argument("--robot_arm", type=str, default="xarm")
    parser.add_argument("--gripper_type", type=str, default="xarm_g")
    args = parser.parse_args()
    c = pb.connect(pb.GUI)
    vis_sp = []
    hand_ctrl = args.use_gloves
    real_robot = args.real_robot
    robot_arm = args.robot_arm
    gripper = args.gripper_type
    ip_config_file = files("dexcap").joinpath("configs/" + robot_arm + '.yaml')
    ip_config = YamlConfig(str(ip_config_file)).as_easydict()
    arm_config_file = files("dexcap").joinpath("configs/robot_config/" + robot_arm + '_arm.yaml')
    arm_config = YamlConfig(str(arm_config_file)).as_easydict()

    camera_type = "zed"
    camera_kwargs={}
    camera_reader = None
    right_cam = False
    left_cam = False
    wrist_cam = False

    arm_start_joints = arm_config['fixed_joints']
    impedance_controller_cfg = YamlConfig(arm_config['impedance_controller_cfg']).as_easydict()
    position_controller_cfg = YamlConfig(arm_config['position_controller_cfg']).as_easydict()

    c_code = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1]]
    for i in range(4):
        vis_sp.append(create_primitive_shape(pb, 0.1, pb.GEOM_SPHERE, [0.02], color=c_code[i]))

    check_connection("VR_headset", ip_config.TELEOP.VR_HOST)
    if real_robot:
        check_connection("Arm_control", ip_config.CTRL_HOST.IP_ETH)
        if robot_arm == "xarm":
            if gripper == "xarm_g":
                robot_interface = XArmInterface(general_cfg=ip_config, has_gripper=True, control_freq=args.frequency)
                arm_start_joints.append(arm_config['gripper_init'][0])
            else:
                robot_interface = XArmInterface(general_cfg=ip_config, has_gripper=False, control_freq=args.frequency)
        else:
            robot_interface = FrankaInterface(general_cfg=ip_config, use_visualizer=False, has_gripper=False, control_freq=args.frequency)

        init_robot(robot_interface, arm_start_joints, position_controller_cfg)
        if gripper == "leap":
            redis_client = redis.Redis(ip_config.CTRL_HOST.IP_ETH, port=ip_config.CTRL_HOST.HAND_PORT, db=0)
            init_leaphand(redis_client)
        if args.use_camera:
            camera_reader = MultiCameraWrapper(camera_kwargs)
            for key in camera_reader.camera_dict:
                if left_camera_id in key:
                    left_cam = True
                if right_camera_id in key:
                    right_cam = True
                if wrist_camera_id in key:
                    wrist_cam = True
            camera_obs, camera_timestamp = camera_reader.read_cameras()

    if hand_ctrl:
        rokoko = RokokoModule(ip_config)

    if gripper == "leap":
        solver = LeapHandIKSolver(arm_config)
    else:
        solver = ParallelGripperIKSolver(arm_config)
    quest = QuestTeleopModule(ip_config, solver=solver)

    start_time = time.time()
    fps_counter = 0
    packet_counter = 0
    logger.info("Initialization completed... Start app in headset")
    current_ts = time.time()
    restart_app_flag = True
    no_arm_ctrl = True

    recording = False
    traj_count = 0
    while True:
        now = time.time()
        # TODO: May cause communication issues, need to tune on AR side.
        if now - current_ts < 1 / args.frequency:
            continue
        else:
            current_ts = now
        try:

            if hand_ctrl:
                while True:
                    try:
                        left_positions, right_positions = rokoko.receive()
                        break
                    except socket.timeout:
                        logger.warning("Waiting for Rokoko data...")
                        continue
                rokoko.send_joint_data(np.vstack([left_positions, right_positions]))
            status, right_wrist, head_pose= quest.receive()
            if status == StatusCode.SOCKET_TIMEOUT:
                logger.warning("No data from quest received for 5s, check connection...")
            elif status == StatusCode.APP_RESTART:
                if restart_app_flag:
                    logger.warning('Restart app in Quest headset')
                    restart_app_flag = False
            elif status == StatusCode.SUCCESS:
                if right_wrist is not None:
                    right_wrist_pos, right_wrist_rot = right_wrist
                    right_wrist_orn = Rotation.from_quat(right_wrist_rot)
                    head_pos = head_pose[0]
                    head_orn = Rotation.from_quat(head_pose[1])
                    if hand_ctrl:
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
                            current_actions = []
                            current_joint_pos = []
                            current_ee_pos = []
                            current_ee_quat = []
                            current_gripper_qpos = []
                            current_agentview_right_imgs, current_agentview_left_imgs, current_eye_in_hand_imgs = [], [], []

                        if real_robot:
                            if robot_arm == "xarm":
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
                                robot_interface.control(controller_type="JOINT_POSITION", action=robot_q)
                            else:
                                robot_interface.control(controller_type="JOINT_IMPEDANCE", action=right_arm_q, controller_cfg=impedance_controller_cfg)

                            if gripper == "leap":
                                redis_client.set('right_leap_action', pickle.dumps(convert_to_hardware(right_hand_q)))

                        action_vec = np.concatenate([right_arm_q, right_hand_q])
                        current_actions.append(action_vec)

                        if real_robot:
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

                            if camera_reader is not None:
                                camera_obs, camera_timestamp = camera_reader.read_cameras()
                                obs_dict.update(camera_obs)
                                obs_dict["timestamp"]["cameras"] = camera_timestamp
                                image_observations = obs_dict["image"]
                                img_obs = extract_img_observation(
                                    image_observations, left_camera_id, right_camera_id, wrist_camera_id
                                )

                                # Process and collect images
                                if wrist_cam:
                                    current_eye_in_hand_imgs.append(
                                        resize_with_pad(img_obs["wrist_image"], 224, 224)
                                    )

                                if left_cam:
                                    current_agentview_left_imgs.append(
                                        resize_with_pad(img_obs["left_image"], 224, 224)
                                    )

                                if right_cam:
                                    current_agentview_right_imgs.append(
                                        resize_with_pad(img_obs["right_image"], 224, 224)
                                    )
                        else:
                            current_joint_pos.append(np.array(right_arm_q, dtype=np.float32))
                            current_ee_pos.append(np.array(right_wrist_pos, dtype=np.float32))
                            current_ee_quat.append(np.array(right_wrist[1], dtype=np.float32))
                            current_gripper_qpos.append(np.array(right_hand_q, dtype=np.float32))

                    else:
                        if recording:
                            recording = False
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

                                logger.info(f"Trajectory saved ({len(current_actions)} steps) -> {hdf5_path} [traj_{traj_count}]")
                                current_eye_in_hand_imgs.clear()
                                current_agentview_left_imgs.clear()
                                current_agentview_right_imgs.clear()


        except socket.error as e:
            logger.error(e)
            pass
        except KeyboardInterrupt:
            #camera.close()
            if hand_ctrl:
                rokoko.close()
            # quest.close()
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

