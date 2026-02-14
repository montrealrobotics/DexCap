import socket
import time
from argparse import ArgumentParser
import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as pb
import yaml
from rigidbodySento import create_primitive_shape
from ip_config import *
from rokoko_module import RokokoModule
#from realsense_module import DepthCameraModule
from quest_robot_module import QuestRightArmLeapModule

# Robot deployment imports
import redis
import pickle
from deoxys.xarm_interface import XArmInterface
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from utils import check_connection, get_logger

logger = get_logger("teleop_server")

def convert_to_hardware(joint_angles):
    real_right_robot_hand_q = np.zeros(16)
    real_right_robot_hand_q[0:16] = joint_angles[0:16]
    real_right_robot_hand_q[0:2] = real_right_robot_hand_q[0:2][::-1]
    real_right_robot_hand_q[4:6] = real_right_robot_hand_q[4:6][::-1]
    real_right_robot_hand_q[8:10] = real_right_robot_hand_q[8:10][::-1]
    real_right_robot_hand_q[:16] += np.pi
    return real_right_robot_hand_q.tolist()


def init_robot(redis_client, robot_interface, arm_start_joints):
    hand_target = [0.0 for _ in range(16)]
    redis_client.set('right_leap_action', pickle.dumps(convert_to_hardware(hand_target)))

    robot_interface._state_buffer = []

    paper_q = [0.0 for _ in range(16)]
    for _ in range(10):
        robot_interface.control(
            controller_type="JOINT_POSITION",
            action=arm_start_joints
        )
        time.sleep(0.5)
        redis_client.set('right_leap_action', pickle.dumps(convert_to_hardware(paper_q)))
    logger.info("Robot initial position sent")
    state = robot_interface.get_state()
    return


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--frequency", type=int, default=30)
    parser.add_argument("--real_robot", type=bool, default=False)
    parser.add_argument("--use_gloves", type=bool, default=False)
    parser.add_argument("--robot_arm", type=str, default="franka")
    args = parser.parse_args()
    c = pb.connect(pb.GUI)
    vis_sp = []
    hand_ctrl = args.use_gloves
    real_robot = args.real_robot
    robot_arm = args.robot_arm
    arm_config = YamlConfig("configs/" + robot_arm + '_arm.yaml').as_easydict()
    arm_start_joints = arm_config['fixed_joints']

    c_code = c_code = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1]]
    for i in range(4):
        vis_sp.append(create_primitive_shape(pb, 0.1, pb.GEOM_SPHERE, [0.02], color=c_code[i]))

    check_connection("VR_headset", VR_HOST)
    if real_robot:
        check_connection("Arm_control", CONTROL_HOST)
        redis_client = redis.Redis(host=CONTROL_HOST,port=HAND_PORT, db=0)
        if robot_arm == "xarm":
            robot_interface = XArmInterface(robot_ctrl_ip=CONTROL_HOST, cmd_port=ARM_PORT, control_freq=args.frequency)
        else:
            robot_interface = FrankaInterface(robot_ctrl_ip=CONTROL_HOST, cmd_port=ARM_PORT, control_freq=args.frequency)
        init_robot(redis_client, robot_interface, arm_start_joints)
    #camera = DepthCameraModule(is_decimate=False, visualize=False)

    if hand_ctrl:
        rokoko = RokokoModule(VR_HOST, HAND_INFO_PORT, ROKOKO_PORT)
    quest = QuestRightArmLeapModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, arm_config, vis_sp=None)

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
            right_wrist, head_pose= quest.receive()
            if right_wrist == 1:
                if restart_app_flag:
                    logger.warning('Restart app in Quest headset')
                    restart_app_flag = False
                continue
            #point_cloud = camera.receive()
            if hand_ctrl:
                left_positions, right_positions = rokoko.receive()
                rokoko.send_joint_data(np.vstack([left_positions, right_positions]))
            if right_wrist is not None:
                right_wrist_orn = Rotation.from_quat(right_wrist[1])
                right_wrist_pos = right_wrist[0]
                head_pos = head_pose[0]
                head_orn = Rotation.from_quat(head_pose[1])
                if hand_ctrl:
                    hand_tip_pose = right_wrist_orn.apply(right_positions) + right_wrist_pos
                    hand_tip_pose = hand_tip_pose[[1,2,3,0]]
                    right_arm_q, right_hand_q = quest.solve_system_world(right_wrist_pos, right_wrist_orn, hand_tip_pose)

                else:
                    right_arm_q, right_hand_q = quest.solve_system_world(right_wrist_pos, right_wrist_orn)

                quest.send_ik_result(right_arm_q, right_hand_q)
                if quest.data_dir is not None:
                    if real_robot:
                        robot_interface.control(controller_type="JOINT_POSITION", action=right_arm_q)
                        # state = robot_interface.get_state()
                        redis_client.set('right_leap_action', pickle.dumps(convert_to_hardware(right_hand_q)))
        except socket.error as e:
            logger.error(e)
            pass
        except KeyboardInterrupt:
            #camera.close()
            rokoko.close()
            quest.close()
            break
        else:
            packet_time = time.time()
            fps_counter += 1
            packet_counter += 1

            if (packet_time - start_time) > 1.0:
                logger.info(f"received {fps_counter} packets in a second")
                start_time += 1.0
                fps_counter = 0

