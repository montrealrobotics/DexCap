# ffmpeg -framerate 10 -i saved_current_frames/%d.jpg -c:v mpeg4 -pix_fmt yuv420p saved_run_video.mp4

from argparse import ArgumentParser
import h5py
import time
import traceback
import numpy as np
import redis
import torch
import pickle
from transforms3d.euler import quat2mat
from dexcap.sensors.cameras.usb_camera_module import MultiCameraWrapper
import pybullet as pb
from deoxys.robot_interfaces.xarm_interface import XArmInterface
from deoxys.franka_interface import FrankaInterface
from dexcap.utils.ik_solver import LeapHandIKSolver, ParallelGripperIKSolver, BaseIKSolver

from glob import glob
from scipy.spatial.transform import Rotation
from deoxys.franka_interface import FrankaInterface
# from gprs.utils.io_devices import SpaceMouse
# from gprs.utils.input_utils import input2action
from deoxys.utils import YamlConfig
from importlib.resources import files

import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.tensor_utils as TensorUtils
import matplotlib.pyplot as plt
# from deoxys_spacemouse.input_utils import input2action
# from deoxys_spacemouse.spacemouse import SpaceMouse

camera_poses = [np.array([-0.4, -0.25, 0.3]),np.array([-0.6, -0.25, 0.3]), np.array([-0.2, -0.25, 0.3])]
look_ats = [np.array([-0.4, 0.2, 0.1]), np.array([-0.4, 0.2, 0.1]), np.array([-0.4, 0.2, 0.1])]
up_dirs = [np.array([0.0, 0.0, 1.0]),np.array([0.0, 0.0, 1.0]),np.array([0.0, 0.0, 1.0])]
fovs = [65, 65, 65]

pb.connect(pb.GUI)

class LeapHand:
    def __init__(self, ip, port):
        self.redis_client = redis.Redis(ip, port=int(port), db=0)

    def convert_to_hardware(self, joint_angles):
        real_right_robot_hand_q = np.zeros(16)
        real_right_robot_hand_q[0:16] = joint_angles[0:16]
        real_right_robot_hand_q[0:2] = real_right_robot_hand_q[0:2][::-1]
        real_right_robot_hand_q[4:6] = real_right_robot_hand_q[4:6][::-1]
        real_right_robot_hand_q[8:10] = real_right_robot_hand_q[8:10][::-1]
        real_right_robot_hand_q[:16] += np.pi
        return real_right_robot_hand_q.tolist()

    def reverse_conversion(self, joint_angles):
        real_right_robot_hand_q = np.zeros(16)
        real_right_robot_hand_q[0:16] = joint_angles[0:16]
        real_right_robot_hand_q[0:2] = real_right_robot_hand_q[0:2][::-1]
        real_right_robot_hand_q[4:6] = real_right_robot_hand_q[4:6][::-1]
        real_right_robot_hand_q[8:10] = real_right_robot_hand_q[8:10][::-1]
        real_right_robot_hand_q[:16] -= np.pi
        return real_right_robot_hand_q.tolist()


class RobotEnv:
    def __init__(self, robot_arm, gripper_type, action_type,  camera_kwargs={}, frequency=30):
        self.gripper_type = gripper_type
        self.robot_arm = robot_arm
        self.action_type = action_type
        pkg_path = str(files("dexcap").joinpath("configs/"))
        self.arm_config = YamlConfig(pkg_path + "/robot_config/" + self.robot_arm + '_arm.yaml').as_easydict()
        self.ip_config = YamlConfig(pkg_path + "/" + self.robot_arm + '.yaml').as_easydict()
        self.camera_reader = MultiCameraWrapper(camera_kwargs)

        self.arm_init_joints = self.arm_config["rest_position"]

        if gripper_type == "leap":
            self.leap_hand = LeapHand(self.ip_config.CTRL_HOST.IP_ETH, self.ip_config.CTRL_HOST.HAND_PORT)

        self.solver = BaseIKSolver(self.arm_config)
        #self.pcd_idx = np.random.choice(10000, 10000, replace=False)

        self.device = TorchUtils.get_torch_device(try_to_use_cuda=True)

        self.impedance_controller_cfg = YamlConfig(self.arm_config['impedance_controller_cfg']).as_easydict()
        self.position_controller_cfg = YamlConfig(self.arm_config['position_controller_cfg']).as_easydict()

        if self.robot_arm == "xarm":
            if gripper_type == "xarm_g":
                self.robot_interface = XArmInterface(general_cfg=self.ip_config, has_gripper=True, control_freq=frequency)
            else:
                self.robot_interface = XArmInterface(general_cfg=self.ip_config, has_gripper=False, control_freq=frequency)
        else:
            self.robot_interface = FrankaInterface(general_cfg=self.ip_config, use_visualizer=False, has_gripper=False, control_freq=frequency)

        # self.REALROBOT_RIGHT_HAND_OFFSET_CONFIG_PATH = "./config/realrobot_right_hand_offset.yml"
        # self.REALROBOT_RIGHT_HAND_OFFSET = None
        # with open(self.REALROBOT_RIGHT_HAND_OFFSET_CONFIG_PATH) as f:
        #     self.REALROBOT_RIGHT_HAND_OFFSET = yaml.safe_load(f)
        self.reset_cnt = 0

    def init_robot(self):
        # robot_interface._state_buffer = []
        # first reset the arm to a initial pose

        hand_target = np.array([0.0 for _ in range(16)])
        self.solver.set_joint_positions(self.solver.right_arm, self.arm_init_joints)
        if self.gripper_type == "leap":
            initial_hand_q = pickle.loads(self.leap_hand.redis_client.get("right_leap_joints"))
            a = self.arm_start_joints
        elif self.gripper_type =="xarm_g":
            initial_hand_q = self.arm_config['gripper_init'][0]
            a = self.arm_init_joints + [initial_hand_q]
        for i in range(10):
            self.robot_interface.control(
                controller_type="JOINT_POSITION",
                action=a,
                controller_cfg=self.position_controller_cfg,
            )
            time.sleep(0.5)

            if self.gripper_type == "leap":
                self.leap_hand.redis_client.set('right_leap_action', pickle.dumps(i/10*self.leap_hand.convert_to_hardware(hand_target)+(10-i)/10*initial_hand_q))

        input("Press Enter to continue...")

    def get_observation(self):
        obs_dict = {"timestamp": {}}

        # Robot State #
        state_dict, timestamp_dict = self.get_robot_state()
        obs_dict["robot_state"] = state_dict

        # Camera Readings #
        camera_obs, camera_timestamp = self.read_cameras()
        obs_dict.update(camera_obs)
        obs_dict["timestamp"]["cameras"] = camera_timestamp
        obs_dict["timestamp"]["robot_state"] = timestamp_dict

        return obs_dict

    def get_robot_state(self):
        read_start = time.time_ns()
        robot_states_dict = {}
        timestamp_dict = {}
        robot_last_state = self.robot_interface.last_state()
        right_hand_joints = None
        ee_pose_full = None

        if self.robot_arm == "franka":
            right_arm_joints = np.array(robot_last_state.q)
            ee_pose_full = np.zeros(6, dtype=np.float32)
        else:
            right_arm_joints = np.array(robot_last_state["joint_positions"], dtype=np.float32)
            ee_pose = np.array(robot_last_state["ee_pos"], dtype=np.float32)
            ee_quat = np.array(robot_last_state["ee_quat"], dtype=np.float32)
            # quat_scipy = np.roll(ee_quat, -1)
            rotation = Rotation.from_quat(ee_quat)
            ee_rot = rotation.as_euler('xyz', degrees=True)
            ee_pose_full = np.concatenate([ee_pose, ee_rot])

            if self.gripper_type == "xarm_g":
                right_hand_joints = robot_last_state["gripper_pos"]

        if self.gripper_type == "leap":
            raw_leap_data = self.leap_hand.redis_client.get("right_leap_joints")
            if raw_leap_data is not None:
                right_hand_joints = self.leap_hand.reverse_conversion(pickle.loads(raw_leap_data))

        robot0_arm_joints = right_arm_joints
        robot0_hand_joints = right_hand_joints
        robot_states_dict["robot0_arm_joints"] = robot0_arm_joints
        robot_states_dict["robot0_hand_joints"] = robot0_hand_joints
        robot_states_dict["ee_pose_full"] = ee_pose_full
        timestamp_dict["read_start"] = read_start
        timestamp_dict["read_end"] = time.time_ns()

        return robot_states_dict, timestamp_dict

    def read_cameras(self):
        return self.camera_reader.read_cameras()

    def convert_delta_to_joints(self, delta_arm, delta_gripper):
        states = self.get_robot_states()

        arm_joints = states["robot0_arm_joints"]
        gripper_joints = states["robot0_hand_joints"]
        ee_pose = ["ee_pose_full"]
        R_robot_to_pb = Rotation.from_euler('z', 90, degrees=True)

        arm_t_robot = ee_pose[:3] + delta_arm[:3]

        rot_curr = Rotation.from_euler('xyz', ee_pose[3:], degrees=True)
        rot_delta = Rotation.from_euler('xyz', delta_arm[3:], degrees=True)
        rot_target_robot = rot_delta * rot_curr

        arm_t_pb = R_robot_to_pb.apply(arm_t_robot)
        rot_target_pb = R_robot_to_pb * rot_target_robot

        # Use ee index one less that the grasp target used for quest etc, this is robot ee.
        arm_q = self.solver.solve_arm_ik(arm_t_pb, rot_target_pb, ee_idx=self.arm_config["end_effector_index"][0]-1)
        self.solver.set_joint_positions(self.solver.right_arm, arm_q)

        hand_q = gripper_joints + delta_gripper
        return arm_q, hand_q

    def get_state(self, first=False):
        return None

    def check_arrived(self, goal_arm, goal_hand):
        # threshold = 0.05
        # threshold_hand = 0.3

        # robot0_arm_joints, robot0_hand_joints = self.get_robot_states()

        # diff_arm = np.abs(goal_arm - robot0_arm_joints)
        # diff_hand = np.abs(goal_hand - robot0_hand_joints)
        # print("Tracking error:", diff_arm, diff_hand)

        #return np.all(diff_arm <= threshold) #and np.all(diff_hand <= threshold_hand)
        # if self.reset_cnt > 0:
        #     self.reset_cnt = 0
        #     return True
        # else:
        #     self.reset_cnt += 1
        #     return False
        return True          

def run_trained_agent(args):
    # load ckpt dict and get algo name for sanity checks

    # device
    device = torch.device("cpu")

    # Load trajectory
    # traj = np.load(f"trajs/{args.trajectory}.npz")
    # arm_q_traj = traj["arm_qs"][args.start_idx::args.stride] # 3 is harded coded stride.
    # wrist_poses = traj["wrist_poses"][args.start_idx::args.stride]
    # wrist_orns = traj["wrist_orns"][args.start_idx::args.stride]
    # hand_q_traj = traj["hand_qs"][args.start_idx::args.stride]

    robot_arm = args.robot_arm
    gripper_type = args.gripper_type

    action_space = "cart_pose_delta"

    if action_space == "joint_positions":
        arm_q_traj = [[0.0, -1.92, -0.39460, 0.0, 1.51, -0.10435],
                    [0.0, -1.92, -0.39460, 0.0, 1.61, -0.20435],
                    [0.0, -1.92, -0.39460, 0.0, 1.71, -0.30435],
                    [0.0, -1.92, -0.39460, 0.0, 1.61, -0.20435],
                    [0.0, -1.92, -0.39460, 0.0, 1.51, -0.10435],
                    [0.0, -1.92, -0.39460, 0.0, 1.51, -0.00435]]
        hand_q_traj = [[0.0], [0.5], [0.0], [0.0], [0.0], [0.0]]
    elif action_space == "joint_deltas":
        arm_q_traj = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        hand_q_traj = [[0.0], [0.1], [0.1], [-0.1], [-0.1], [0.0]]
    elif action_space == "cart_pose":
        pass
    elif action_space == "cart_pose_delta":
        # Pose deltas instead of joint angles
        arm_q_traj = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        hand_q_traj = [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]

    # create environment
    env = RobotEnv(robot_arm, gripper_type, action_type=action_space)

    if "delta" not in action_space:
        env.init_robot(init_arm=arm_q_traj[0], init_hand=hand_q_traj[0])

    # maybe set seed
    if args.seed is not None:
        np.random.seed(args.seed)
        torch.manual_seed(args.seed)

    substep = 0
    step = 0
    
    arrived = True
    time_ts = time.time()
    with torch.no_grad():
        while True:
            if arrived:
                if step >= len(arm_q_traj):
                    break
                goal_arm = arm_q_traj[step]
                goal_hand = hand_q_traj[step]
                print(f"Step: {step}")
                arrived = False
                step += 1
            
            goal_arm_ = goal_arm.copy()
            goal_hand_ = goal_hand.copy()

            if "delta" in action_space:
                goal_arm_j_, goal_hand_j_ = env.convert_delta_to_joints(goal_arm_, goal_hand_)
            else:
                goal_arm_j_ = goal_arm.copy()
                goal_hand_j_ = goal_hand.copy()

            if env.robot_arm == "franka":
                a = goal_arm_j_
                env.robot_interface.control(controller_type="JOINT_IMPEDANCE", action=a, controller_cfg=env.impedance_controller_cfg)
            else:
                if env.gripper_type == "xarm_g":
                    a = np.concatenate([goal_arm_j_, goal_hand_j_])
                env.robot_interface.control(controller_type="JOINT_POSITION", action=a)
            if env.gripper_type == "leap":
                goal_hand_ = env.leap_hand.convert_to_hardware(goal_hand_)
                env.leap_hand.redis_client.set('right_leap_action', pickle.dumps(goal_hand_))

            arrived = env.check_arrived(goal_arm_j_, goal_hand_j_)

            substep += 1
            # print control frequency
            print("Frequency:", 1 / (time.time() - time_ts))
            time_ts = time.time()


if __name__ == "__main__":
    parser = ArgumentParser()

    # Trajectory to load and replay
    # parser.add_argument(
    #     "--trajectory",
    #     type=str,
    #     required=True,
    #     help="path to npz file containing trajectory to replay",
    # )

    # for seeding before starting rollouts
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="(optional) set seed for rollouts",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1
    )
    parser.add_argument(
        "--action_space",
        type=str,
        default="joint")

    parser.add_argument(
        "--start_idx",
        type=int,
        default=0
    )
    parser.add_argument(
        "--robot_arm",
        type=str,
        default="xarm"
    )
    parser.add_argument(
        "--gripper_type",
        type=str,
        default="xarm_g"
    )
    parser.add_argument(
        "--error_path",
        type=str,
        default="error.log"
    )
    args = parser.parse_args()

    try:
        run_trained_agent(args)
    except Exception as e:
        res_str = "run failed with error:\n{}\n\n{}".format(e, traceback.format_exc())
        if args.error_path is not None:
            # write traceback to file
            f = open(args.error_path, "w")
            f.write(res_str)
            f.close()
        raise e
