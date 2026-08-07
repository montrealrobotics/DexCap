# envs/real_robot_env.py
import time
import threading
import numpy as np
from deoxys.robot_interfaces.xarm_interface import XArmInterface
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from realsense_module import DepthCameraModule
import redis
import pickle

def convert_to_hardware(joint_angles):
    q = np.array(joint_angles[:16], dtype=np.float64)
    q[0:2] = q[0:2][::-1]
    q[4:6] = q[4:6][::-1]
    q[8:10] = q[8:10][::-1]
    q += np.pi
    return q.tolist()

class Gripper:
    def __init__(self, gripper_type, dof=1):
        self.dof = dof
        self.grasp_qpos = {1: np.ones(dof), -1: -np.ones(dof)}
        self.gripper_type = gripper_type
        self.control_client = None

    def init_gripper(self, ip, port):
        if self.gripper_type == "leap":
            self.control_client = redis.Redis(ip, port=int(port), db=0)
            hand_target = [0.0 for _ in range(self.dof)]
            self.control_client.set('right_leap_action', pickle.dumps(convert_to_hardware(hand_target)))

    def control(self, right_hand_q):
        self.control_client.set('right_leap_action', pickle.dumps(convert_to_hardware(right_hand_q)))


class Robot:
    SUPPORTED_ARMS = ["franka", "xarm"]
    SUPPORTED_GRIPPERS = ["leap", "xarm_g", "none"]

    def __init__(
        self,
        robot_arm="franka",
        robot_name="robot0",
        gripper_type="leap",
        frequency=30,
        arms=["right"],
    ):
        if robot_arm not in self.SUPPORTED_ARMS:
            raise NotImplementedError(
                f"Robot arm '{robot_arm}' is not supported. Choose from {self.SUPPORTED_ARMS}"
            )
        if gripper_type not in self.SUPPORTED_GRIPPERS:
            raise NotImplementedError(
                f"Gripper type '{gripper_type}' is not supported. Choose from {self.SUPPORTED_GRIPPERS}"
            )

        self.robot_arm = robot_arm
        self.robot_name = robot_name
        self.gripper_type = gripper_type
        self.arms = arms
        self.frequency = frequency

        self.arm_config = YamlConfig(f"configs/{robot_arm}_arm.yaml").as_easydict()
        self.ip_config = YamlConfig(f"robot_config/{robot_arm}.yaml").as_easydict()

        if self.robot_arm == "xarm":
            has_gripper = (gripper_type == "xarm_g")
            self.robot_interface = XArmInterface(
                general_cfg=self.ip_config,
                has_gripper=has_gripper,
                control_freq=frequency
            )
            self.arm_dof = 6
        elif self.robot_arm == "franka":
            self.robot_interface = FrankaInterface(
                general_cfg=self.ip_config,
                use_visualizer=False,
                has_gripper=False,
                control_freq=frequency
            )
            self.arm_dof = 7

        gripper_dof_map = {"leap": 16, "xarm_g": 1, "none": 0}
        self.gripper = {}
        for arm in self.arms:
            g = Gripper(gripper_type=gripper_type, dof=gripper_dof_map[gripper_type])

            if gripper_type == "leap":
                g.init_gripper(
                    ip=self.ip_config.CTRL_HOST.IP_ETH,
                    port=self.ip_config.CTRL_HOST.HAND_PORT
                )
            self.gripper[arm] = g

        self.part_controllers = {
            arm: type("DummyController", (), {"input_type": "absolute", "name": "JOINT_POSITION"})()
            for arm in self.arms
        }
        self.is_mobile = False

    def reset(self):
        arm_start_joints = self.arm_config['fixed_joints']
        position_cfg = YamlConfig(self.arm_config['position_controller_cfg']).as_easydict()

        self.robot_interface._state_buffer = []
        for _ in range(10):
            self.robot_interface.control(
                controller_type="JOINT_POSITION",
                action=arm_start_joints,
                controller_cfg=position_cfg,
            )
            time.sleep(0.1)

    def control(self, action):
        if self.robot_arm == "xarm":
            self.robot_interface.control(controller_type="JOINT_POSITION", action=action)
        elif self.robot_arm == "franka":
            impedance_cfg = YamlConfig(self.arm_config['impedance_controller_cfg']).as_easydict()
            self.robot_interface.control(
                controller_type="JOINT_IMPEDANCE",
                action=action[:7],
                controller_cfg=impedance_cfg
            )

    def get_obs(self):
        robot_last_state = self.robot_interface.last_state()
        if self.robot_arm == "franka":
            joint_positions = np.array(robot_last_state.q, dtype=np.float32)
            gripper_qpos = np.array(self.robot_interface.last_gripper_q(), dtype=np.float32)
            ee_quat, ee_pose = self.robot_interface.last_eef_quat_and_pos()
        else:
            joint_positions = np.array(robot_last_state["joint_positions"], dtype=np.float32)
            ee_pose = np.array(robot_last_state["ee_pos"], dtype=np.float32)
            ee_quat = np.array(robot_last_state["ee_quat"], dtype=np.float32)
            gripper_qpos = np.array(robot_last_state["gripper_pos"], dtype=np.float32)

        return {
            f"{self.robot_name}_joint_pos": joint_positions[:self.arm_dof],
            f"{self.robot_name}_eef_pos": np.array(ee_pose, dtype=np.float32),
            f"{self.robot_name}_eef_quat": np.array(ee_quat, dtype=np.float32),
            f"{self.robot_name}_gripper_qpos": gripper_qpos,
            f"{self.robot_name}_proprio-state": np.concatenate([joint_positions, ee_pose, ee_quat]),
        }

    def close(self):
        self.robot_interface.close()


class RealRobotEnv:
    def __init__(
        self,
        robots=None,
        robot_arm="franka",
        robot_name="robot0",
        gripper_type="leap",
        use_camera=True,
        img_size=(128, 128),
        frequency=30
    ):
        self.frequency = frequency
        self.dt = 1.0 / frequency
        self.use_camera = use_camera
        self.img_size = img_size

        if robots is not None:
            self.robots = robots
        else:
            self.robots = [
                Robot(
                    robot_arm=robot_arm,
                    robot_name=robot_name,
                    gripper_type=gripper_type,
                    frequency=frequency
                )
            ]

        self.action_dim = sum(r.arm_dof + r.gripper[r.arms[0]].dof for r in self.robots)

        self.camera_module = None
        self._latest_rgb = np.zeros((img_size[1], img_size[0], 3), dtype=np.uint8)
        self._cam_lock = threading.Lock()
        self._cam_running = False

        if self.use_camera:
            self.camera_module = DepthCameraModule(is_decimate=False, visualize=False)
            self._cam_running = True
            self._cam_thread = threading.Thread(target=self._poll_camera, daemon=True)
            self._cam_thread.start()
            time.sleep(1.0)

    def _poll_camera(self):
        """Background loop fetching RGB frames into buffer."""
        while self._cam_running:
            try:
                rgb = self.camera_module.receive_rgb(resize_dim=self.img_size)
                if rgb is not None:
                    with self._cam_lock:
                        self._latest_rgb = rgb.copy()
            except Exception:
                pass

    def reset(self):
        for robot in self.robots:
            robot.reset()
        return self._get_obs()

    def _check_success(self):
        return False

    def render(self):
        pass

    def step(self, action):
        start_time = time.time()

        idx = 0
        for robot in self.robots:
            dof_total = robot.arm_dof + robot.gripper[robot.arms[0]].dof
            robot_action = action[idx : idx + dof_total]
            robot.control(robot_action)
            idx += dof_total

        elapsed = time.time() - start_time
        if elapsed < self.dt:
            time.sleep(self.dt - elapsed)

        obs = self._get_obs()
        return obs, 0.0, False, {}

    def _get_obs(self):
        obs = {}
        for robot in self.robots:
            obs.update(robot.get_obs())

        if self.use_camera:
            with self._cam_lock:
                obs["agentview_image"] = self._latest_rgb.copy()

        return obs

    def close(self):
        self._cam_running = False
        if self.camera_module:
            self.camera_module.close()
        for robot in self.robots:
            robot.close()

    def get_ep_meta(self):
        return {"lang": "Real Robot Teleoperation Data Collection"}

    @property
    def model(self):
        """Provides mock model object expected by DataCollectionWrapper."""
        class DummyModel:
            def get_xml(self):
                return "<robot_model>Real Hardware</robot_model>"
        return DummyModel()

    def _get_observations(self, force_caching=False):
        return self._get_obs()