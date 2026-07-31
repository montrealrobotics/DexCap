import numpy as np
import pybullet as pb
from scipy.spatial.transform import Rotation
from importlib.resources import files


class BaseIKSolver:
    """Base PyBullet kinematics loader and helper methods."""

    def __init__(self, robot_config):
        urdf_path = robot_config["urdf"][0]
        urdf = str(files("dexcap").joinpath(urdf_path))
        self.rest_position = robot_config["rest_position"]
        self.end_effector_index = robot_config["end_effector_index"][0]

        self.right_arm = pb.loadURDF(
            urdf,
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=[0, 0, 0.7071068, 0.7071068],
            useFixedBase=True,
        )
        self.set_joint_positions(self.right_arm, self.rest_position)
        (
            self.right_lower_limits,
            self.right_upper_limits,
            self.right_joint_ranges,
        ) = self.get_joint_limits(self.right_arm)

    def set_joint_positions(self, robot, joint_positions):
        jid = 0
        for i in range(len(joint_positions)):
            if pb.getJointInfo(robot, jid)[2] != pb.JOINT_FIXED:
                pb.resetJointState(robot, jid, joint_positions[i])
            else:
                jid += 1
                pb.resetJointState(robot, jid, joint_positions[i])
            jid += 1

    def get_joint_limits(self, robot):
        joint_lower_limits = []
        joint_upper_limits = []
        joint_ranges = []
        for i in range(pb.getNumJoints(robot)):
            joint_info = pb.getJointInfo(robot, i)
            if joint_info[2] == pb.JOINT_FIXED:
                continue
            joint_lower_limits.append(joint_info[8])
            joint_upper_limits.append(joint_info[9])
            joint_ranges.append(joint_info[9] - joint_info[8])
        return joint_lower_limits, joint_upper_limits, joint_ranges

    def solve_arm_ik(self, wrist_pos, wrist_orn, wrist_offset=None, ee_idx = None):
        if wrist_offset is not None:
            wrist_pos_ = wrist_orn.apply(wrist_offset[:3]) + wrist_pos
            wrist_orn_ = wrist_orn * Rotation.from_euler("xyz", wrist_offset[3:])
        else:
            wrist_pos_, wrist_orn_ = wrist_pos, wrist_orn

        if ee_idx is None:
            ee_idx = self.end_effector_index

        return pb.calculateInverseKinematics(
            self.right_arm,
            ee_idx,
            wrist_pos_,
            wrist_orn_.as_quat(),
            lowerLimits=self.right_lower_limits,
            upperLimits=self.right_upper_limits,
            jointRanges=self.right_joint_ranges,
            restPoses=self.rest_position,
            maxNumIterations=40,
            residualThreshold=0.001,
        )


class LeapHandIKSolver(BaseIKSolver):
    """
    IK Solver for LEAP Hand attached to ANY arm (Franka, xArm, etc.)
    """

    RIGHT_HAND_Q = [
        np.pi / 6, -np.pi / 6, np.pi / 3, np.pi / 6,
        np.pi / 6, 0.0, np.pi / 3, np.pi / 6,
        np.pi / 6, np.pi / 6, np.pi / 3, np.pi / 6,
        np.pi / 6, np.pi / 6, np.pi / 3, np.pi / 6,
    ]
    fingertip_idx = [4, 9, 14, 19]

    def __init__(self, robot_config, vis_sp=None):
        super().__init__(robot_config)
        self.vis_sp = vis_sp

        # Mount offsets for LEAP hand
        self.right_hand_mount_offset = [0.05, -0.05, 0.1]
        self.right_hand_pos_offset = np.array([0.0, 0.0, 0.0])
        self.right_hand_orn_offset = Rotation.from_euler("xyz", [-np.pi, 0.0, 0.0])
        self.right_palm_orn_offset = np.array([-0.1, -0.05, 0.05, 0.0, 0.0, -np.pi / 2])

        self.right_hand = pb.loadURDF("assets/leap_hand/robot_pybullet.urdf")
        self.set_joint_positions(self.right_hand, self.RIGHT_HAND_Q)
        (
            self.right_hand_lower_limits,
            self.right_hand_upper_limits,
            self.right_hand_joint_ranges,
        ) = self.get_joint_limits(self.right_hand)

    def solve_fingertip_ik(self, fingertip_pos):
        tip_poses = []
        for i, fid in enumerate(self.fingertip_idx):
            tip_pos = fingertip_pos[i]
            if self.vis_sp is not None:
                pb.resetBasePositionAndOrientation(self.vis_sp[i], tip_pos, (0, 0, 0, 1))
            tip_poses.append(tip_pos)

        target_q = []
        for i in range(4):
            q_slice = pb.calculateInverseKinematics(
                self.right_hand,
                self.fingertip_idx[i],
                tip_poses[i],
                lowerLimits=self.right_hand_lower_limits,
                upperLimits=self.right_hand_upper_limits,
                jointRanges=self.right_hand_joint_ranges,
                restPoses=self.RIGHT_HAND_Q,
                maxNumIterations=40,
                residualThreshold=0.001,
            )
            target_q += list(q_slice[4 * i : 4 * (i + 1)])
        return target_q

    def solve_system_world(self, wrist_pos, wrist_orn, tip_poses=None):
        arm_q = self.solve_arm_ik(
            wrist_pos + wrist_orn.apply(self.right_hand_pos_offset),
            wrist_orn * self.right_hand_orn_offset.inv(),
            self.right_palm_orn_offset,
        )
        self.set_joint_positions(self.right_arm, arm_q)

        # Attach hand to arm end-effector link dynamically
        hand_xyz = np.asarray(pb.getLinkState(self.right_arm, self.end_effector_index)[0])
        hand_orn = Rotation.from_quat(pb.getLinkState(self.right_arm, self.end_effector_index)[1])

        pb.resetBasePositionAndOrientation(
            self.right_hand,
            hand_xyz + (hand_orn * self.right_hand_orn_offset).apply(self.right_hand_mount_offset),
            (hand_orn * self.right_hand_orn_offset).as_quat(),
        )

        hand_q = (
            self.solve_fingertip_ik(tip_poses)
            if tip_poses is not None
            else self.RIGHT_HAND_Q
        )
        self.set_joint_positions(self.right_hand, hand_q)

        return arm_q, hand_q


class ParallelGripperIKSolver(BaseIKSolver):
    """
    IK Solver for xArm Parallel Gripper attached to an arm.
    """

    RIGHT_HAND_Q = [0.5, 0.5, 0.5]
    INDEX_TIP_IDX, THUMB_TIP_IDX = 0, 3
    HUMAN_MAX_SPAN, HUMAN_MIN_SPAN = 0.12, 0.02

    def __init__(self, robot_config, vis_sp=None):
        super().__init__(robot_config)
        self.vis_sp = vis_sp

        self.right_hand_mount_offset = [0.0, 0.08, -0.057]
        self.right_hand_pos_offset = np.array([0.0, 0.0, 0.0])
        self.right_controller_orn_offset = Rotation.from_euler("xyz", [np.pi, 0.0, 0.0])
        self.right_hand_orn_offset = Rotation.from_euler("xyz", [0.0, 0.0, 0.0])
        self.right_palm_orn_offset = np.array([-0.1, -0.05, 0.05, 0.0, 0.0, -np.pi / 2])
        config_file = str(files("dexcap").joinpath("robot_descriptions/xarm_gripper/xarm6_with_gripper.urdf"))
        self.right_hand = pb.loadURDF(config_file, useFixedBase=False)
        self.set_joint_positions(self.right_hand, self.RIGHT_HAND_Q)

        self.gripper_joint_idx = 0
        self.gripper_lower_limit, self.gripper_upper_limit = 0.0, 0.85
        for i in range(pb.getNumJoints(self.right_hand)):
            info = pb.getJointInfo(self.right_hand, i)
            if b"left_knuckle_joint" in info[1]:
                self.gripper_joint_idx = i
                self.gripper_lower_limit, self.gripper_upper_limit = info[8], info[9]
                break

    def solve_gripper_joint(self, fingertip_poses):
        index_pos = np.array(fingertip_poses[self.INDEX_TIP_IDX])
        thumb_pos = np.array(fingertip_poses[self.THUMB_TIP_IDX])

        if self.vis_sp is not None:
            pb.resetBasePositionAndOrientation(self.vis_sp[0], index_pos, (0, 0, 0, 1))
            pb.resetBasePositionAndOrientation(self.vis_sp[1], thumb_pos, (0, 0, 0, 1))

        human_span = np.linalg.norm(index_pos - thumb_pos)
        normalized_span = np.clip(
            (human_span - self.HUMAN_MIN_SPAN) / (self.HUMAN_MAX_SPAN - self.HUMAN_MIN_SPAN),
            0.0,
            1.0,
        )
        gripper_q = self.gripper_upper_limit - (
            normalized_span * (self.gripper_upper_limit - self.gripper_lower_limit)
        )
        return [gripper_q, gripper_q, gripper_q]

    def solve_system_world(self, wrist_pos, wrist_orn, tip_poses=None):
        arm_q = self.solve_arm_ik(
            wrist_pos + wrist_orn.apply(self.right_hand_pos_offset),
            wrist_orn * self.right_controller_orn_offset.inv(),
            self.right_palm_orn_offset,
        )
        self.set_joint_positions(self.right_arm, arm_q)

        hand_xyz = np.asarray(pb.getLinkState(self.right_arm, self.end_effector_index)[0])
        hand_orn = Rotation.from_quat(pb.getLinkState(self.right_arm, self.end_effector_index)[1])

        pb.resetBasePositionAndOrientation(
            self.right_hand,
            hand_xyz + (hand_orn * self.right_hand_orn_offset).apply(self.right_hand_mount_offset),
            (hand_orn * self.right_hand_orn_offset).as_quat(),
        )

        hand_q = (
            self.solve_gripper_joint(tip_poses)
            if tip_poses is not None
            else self.RIGHT_HAND_Q
        )
        pb.resetJointState(self.right_hand, self.gripper_joint_idx, hand_q[0])

        return arm_q, hand_q
