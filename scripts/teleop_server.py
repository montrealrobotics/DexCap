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
from dexcap.sensors.cameras.realsense_module import DepthCameraModule
from dexcap.sensors.cameras.zed_module import MultiCameraWrapper
from dexcap.sensors.quest_robot_module import QuestRightArmLeapModule, QuestRightArmXArmGripperModule

# Robot deployment imports
import redis
import pickle
from deoxys.robot_interfaces.xarm_interface import XArmInterface
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from dexcap.utils.utils import check_connection, get_logger, StatusCode, extract_img_observation, resize_with_pad

logger = get_logger("teleop_server")

left_camera_id: str = "29712701"
right_camera_id: str = "23697076"
wrist_camera_id: str = "12715737"

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
    parser.add_argument("--real_robot", type=bool, default=False)
    parser.add_argument("--use_gloves", type=bool, default=False)
    parser.add_argument("--robot_arm", type=str, default="franka")
    parser.add_argument("--gripper_type", type=str, default="leap")
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
        if not args.no_camera:
            camera_reader = MultiCameraWrapper(camera_kwargs)
            for key in camera_reader.camera_dict:
                if left_camera_id in key:
                    left_cam = True
                if right_camera_id in key:
                    right_cam = True

    if hand_ctrl:
        rokoko = RokokoModule(ip_config)
    quest = QuestRightArmXArmGripperModule(ip_config, arm_config, vis_sp=None)

    start_time = time.time()
    fps_counter = 0
    packet_counter = 0
    logger.info("Initialization completed... Start app in headset")
    current_ts = time.time()
    restart_app_flag = True
    while True:
        now = time.time()
        # TODO: May cause communication issues, need to tune on AR side.
        if now - current_ts < 1 / args.frequency:
            continue
        else:
            current_ts = now
        try:

            if hand_ctrl:
                left_positions, right_positions = rokoko.receive()
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
                    right_wrist_orn = Rotation.from_quat(right_wrist[1])
                    right_wrist_pos = right_wrist[0]
                    head_pos = head_pose[0]
                    head_orn = Rotation.from_quat(head_pose[1])
                    if hand_ctrl:
                        hand_tip_pose = right_wrist_orn.apply(right_positions) + right_wrist_pos
                        hand_tip_pose = hand_tip_pose[[1,2,3,0]]
                        right_arm_q, right_hand_q, wrist_pos, wrist_orn = quest.solve_system_world(right_wrist_pos, right_wrist_orn, hand_tip_pose)

                    else:
                        right_arm_q, right_hand_q = quest.solve_system_world(right_wrist_pos, right_wrist_orn)
                    quest.send_ik_result(right_arm_q, right_hand_q)
                    obs_dict = {"timestamp": {}}
                    if quest.data_dir is not None:
                        if real_robot:
                            if robot_arm == "xarm":
                                if gripper == 'xarm_g':
                                    right_arm_q = right_arm_q + (right_hand_q[0],)
                                robot_interface.control(controller_type="JOINT_POSITION", action=right_arm_q)

                            else:
                                robot_interface.control(controller_type="JOINT_IMPEDANCE", action=right_arm_q, controller_cfg=impedance_controller_cfg)

                            if gripper == "leap":
                                redis_client.set('right_leap_action', pickle.dumps(convert_to_hardware(right_hand_q)))
                        if camera_reader is not None:
                            camera_obs, camera_timestamp = camera_reader.read_cameras()
                            obs_dict.update(camera_obs)
                            obs_dict["timestamp"]["cameras"] = camera_timestamp
                            img_obs = extract_img_observation(obs_dict, left_camera_id, right_camera_id, wrist_camera_id)
                            obs = {
                                "observation/wrist_image_left": resize_with_pad(
                                    img_obs["wrist_image"], 224, 224),
                            }
                            if left_cam:
                                obs["observation/exterior_image_1_left"] = resize_with_pad(
                                    img_obs["left_image"], 224, 224
                                    )
                            if right_cam:
                                obs["observation/exterior_image_2_left"] = resize_with_pad(
                                    img_obs["right_image"], 224, 224
                                    )

                        np.savez(f"{quest.data_dir}/right_data_{time.time()}.npz", right_wrist_pos=wrist_pos, right_wrist_orn=wrist_orn, 
                                                                                head_pos=head_pos, head_orn=head_orn.as_quat(),
                                                                                right_arm_q=right_arm_q, right_hand_q=right_hand_q,raw_hand_q=hand_q,
                                                                                right_tip_poses=hand_tip_pose, point_cloud=point_cloud)
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

