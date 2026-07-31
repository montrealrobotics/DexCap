import numpy as np
from robosuite.devices import Device
from scipy.spatial.transform import Rotation

from quest_module import QuestRightArmXArmGripperModule, QuestRightArmLeapModule
from utils import StatusCode


class QuestRokokoDevice(Device):
    def __init__(self, env, ip_config, robot_config, gripper_type="leap", vis_sp=None):
        super().__init__(env)
        self.gripper_type = gripper_type

        if gripper_type == "leap":
            self.quest_module = QuestRightArmLeapModule(
                ip_config=ip_config,
                robot_config=robot_config,
                vis_sp=vis_sp
            )
        else:
            self.quest_module = QuestRightArmXArmGripperModule(
                ip_config=ip_config,
                robot_config=robot_config,
                vis_sp=vis_sp
            )

        self.prev_ee_pos = None
        self.prev_ee_quat = None
        self.discard_traj = False

    def start_control(self):
        self.prev_ee_pos = None
        self.prev_ee_quat = None
        self.discard_traj = False

    def input2action(self, mirror_actions=False):
        status, wrist_tf, head_tf = self.quest_module.receive()

        if status == StatusCode.STOP:
            print("[Quest] Stop signal received -> Saving trajectory.")
            self.discard_traj = False
            return None

        if status == StatusCode.REMOVE:
            print("[Quest] Remove signal received -> Discarding trajectory.")
            self.discard_traj = True
            self.quest_module.set_joint_positions(
                self.quest_module.right_arm,
                self.quest_module.rest_position
            )
            return None

        active_robot = self.env.robots[self.active_robot]
        active_arm = self.active_arm

        ac_dict = {}

        if status == StatusCode.APP_RESTART or self.quest_module.world_frame is None:
            for arm in active_robot.arms:
                gripper_dof = active_robot.gripper[arm].dof
                ac_dict[f"{arm}_abs"] = np.array(self.quest_module.rest_position, dtype=np.float32)
                ac_dict[f"{arm}_delta"] = np.zeros(6, dtype=np.float32)
                ac_dict[f"{arm}_gripper"] = np.zeros(gripper_dof, dtype=np.float32)
            return ac_dict

        if status == StatusCode.SOCKET_TIMEOUT:
            return None

        if status == StatusCode.SUCCESS and wrist_tf is not None:
            rel_wrist_pos, rel_wrist_rot = wrist_tf
            rel_wrist_orn = Rotation.from_quat(rel_wrist_rot)

            arm_q, hand_q = self.quest_module.solve_system_world(
                wrist_pos=rel_wrist_pos,
                wrist_orn=rel_wrist_orn
            )

            self.quest_module.send_ik_result(right_arm_q=arm_q, right_hand_q=hand_q)

            if self.prev_ee_pos is None:
                delta_pos = np.zeros(3)
                delta_rot = np.zeros(3)
            else:
                delta_pos = rel_wrist_pos - self.prev_ee_pos
                rot_curr = rel_wrist_orn
                rot_prev = Rotation.from_quat(self.prev_ee_quat)
                delta_rot = (rot_curr * rot_prev.inv()).as_rotvec()

            self.prev_ee_pos = rel_wrist_pos.copy()
            self.prev_ee_quat = rel_wrist_rot.copy()

            for arm in active_robot.arms:
                if arm == active_arm:
                    ac_dict[f"{arm}_abs"] = np.array(arm_q, dtype=np.float32)
                    ac_dict[f"{arm}_delta"] = np.concatenate([delta_pos, delta_rot]).astype(np.float32)
                    ac_dict[f"{arm}_gripper"] = np.array(hand_q, dtype=np.float32)
                else:
                    gripper_dof = active_robot.gripper[arm].dof
                    ac_dict[f"{arm}_abs"] = np.array(self.quest_module.rest_position, dtype=np.float32)
                    ac_dict[f"{arm}_delta"] = np.zeros(6, dtype=np.float32)
                    ac_dict[f"{arm}_gripper"] = np.zeros(gripper_dof, dtype=np.float32)

            return ac_dict

        return None

    def close(self):
        self.quest_module.close()