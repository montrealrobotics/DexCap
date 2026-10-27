import datetime
import os
import shutil
import socket
import numpy as np
from scipy.spatial.transform import Rotation
from dexcap.utils.utils import StatusCode


class QuestTeleopModule:
    def __init__(self, ip_config, solver, timeout=5, data_root="data"):
        self.solver = solver
        self.data_root = data_root
        # Optional StreamStats (dexcap.utils.status) for the incoming pose stream and outgoing IK results
        self.stats_in = None
        self.stats_out = None
        self.vr_ip = ip_config.TELEOP.VR_HOST
        self.pose_cmd_port = ip_config.TELEOP.POSE_CMD_PORT
        self.ik_result_port = ip_config.TELEOP.IK_RESULT_PORT

        self.wrist_listener_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.wrist_listener_s.bind(("", self.pose_cmd_port))
        self.timeout = timeout
        self.wrist_listener_s.settimeout(timeout)
        # Default receive buffer: receive() drains it every call, so poses stay fresh without the
        # kernel dropping one-off control messages such as WorldFrame
        self.wrist_listener_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.world_frame = None
        self.wf_receive_ts = None
        self.data_dir = None
        self.prev_data_dir = None
        self.last_arm_q = None
        self.last_hand_q = None

        if self.ik_result_port is not None:
            self.ik_result_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ik_result_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.ik_result_dest = (self.vr_ip, self.ik_result_port)
        else:
            self.ik_result_s = None

    def compute_rel_transform(self, pose):
        world_frame = self.world_frame.copy()
        world_frame[:3] = np.array([world_frame[0], world_frame[2], world_frame[1]])
        pose[:3] = np.array([pose[0], pose[2], pose[1]])

        Q = np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0.0]])
        rot_base = Rotation.from_quat(world_frame[3:]).as_matrix()
        rot = Rotation.from_quat(pose[3:]).as_matrix()

        rel_rot = Rotation.from_matrix(Q @ (rot_base.T @ rot) @ Q.T)
        rel_pos = Rotation.from_matrix(Q @ rot_base.T @ Q.T).apply(pose[:3] - world_frame[:3])
        return rel_pos, rel_rot.as_quat()

    def check_delta_joints(self, this_q, prev_q, threshold=0.1):
        if prev_q is None:
            return True
        return np.all(np.abs(np.array(this_q) - np.array(prev_q)) < threshold)

    def receive(self):
        """Handle every message queued since the last call, in order, and return the newest pose.

        Control messages (WorldFrame, Start, Stop, Remove) are sent once, so none may be skipped;
        older poses are dropped so the robot follows the latest one. Without a pose in the batch,
        the result of the last message is returned.
        """
        try:
            data, addr = self.wrist_listener_s.recvfrom(1024)
        except socket.timeout:
            return StatusCode.SOCKET_TIMEOUT, None, None
        result = self._handle(data, addr)
        newest_pose = result if result[1] is not None else None

        self.wrist_listener_s.setblocking(False)
        try:
            while True:
                try:
                    data, addr = self.wrist_listener_s.recvfrom(1024)
                except BlockingIOError:
                    break
                result = self._handle(data, addr)
                if result[1] is not None:
                    newest_pose = result
        finally:
            self.wrist_listener_s.settimeout(self.timeout)
        return newest_pose if newest_pose is not None else result

    def _handle(self, data, addr):
        data_string = data.decode()
        if self.stats_in is not None:
            self.stats_in.mark(sender=addr[0], last_msg=data_string.split(":")[0][:12],
                               world_frame="set" if self.world_frame is not None else "not set")
        now = datetime.datetime.now()

        if data_string.startswith("WorldFrame"):
            data_list = [float(d) for d in data_string[11:].split(",")]
            self.world_frame = np.array(data_list)
            self.wf_receive_ts = now.strftime("%Y-%m-%d-%H-%M-%S")

            # Reset solvers
            self.solver.set_joint_positions(self.solver.right_arm, self.solver.rest_position)
            self.solver.set_joint_positions(self.solver.right_hand, self.solver.RIGHT_HAND_Q)

            os.makedirs(os.path.join(self.data_root, self.wf_receive_ts), exist_ok=True)
            np.save(os.path.join(self.data_root, self.wf_receive_ts, "WorldFrame.npy"), self.world_frame)
            return StatusCode.SUCCESS, None, None

        elif data_string.startswith("Start") and self.wf_receive_ts is not None:
            formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
            self.data_dir = os.path.join(self.data_root, self.wf_receive_ts, formatted_time)
            os.makedirs(self.data_dir, exist_ok=True)
            return StatusCode.SUCCESS, None, None

        elif data_string.startswith("Stop"):
            if self.data_dir is not None:
                self.prev_data_dir = self.data_dir
            self.data_dir = None
            return StatusCode.STOP, None, None

        elif data_string.startswith("Remove"):
            for d in [self.data_dir, self.prev_data_dir]:
                if d and os.path.exists(d):
                    shutil.rmtree(d)
            self.data_dir = self.prev_data_dir = None
            return StatusCode.REMOVE, None, None

        elif "RHand" in data_string and self.world_frame is not None:
            data_list = [float(d) for d in data_string[7:].split(",")]
            rel_wrist_pos, rel_wrist_rot = self.compute_rel_transform(np.array(data_list[:7]))
            rel_head_pos, rel_head_rot = self.compute_rel_transform(np.array(data_list[7:]))

            if self.data_dir is None and data_string[0] == "Y":
                formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
                self.data_dir = os.path.join(self.data_root, self.wf_receive_ts, formatted_time)
                os.makedirs(self.data_dir, exist_ok=True)

            return StatusCode.SUCCESS, (rel_wrist_pos, rel_wrist_rot), (rel_head_pos, rel_head_rot)

        return StatusCode.APP_RESTART, None, None

    def send_ik_result(self, right_arm_q, right_hand_q=None):
        if not self.ik_result_s:
            return

        delta_arm = self.check_delta_joints(right_arm_q, self.last_arm_q)
        delta_hand = self.check_delta_joints(right_hand_q, self.last_hand_q, 0.2) if right_hand_q else True
        delta_result = "G" if self.data_dir is None else ("Y" if (delta_arm and delta_hand) else "N")

        arm_str = ",".join(f"{q:.3f}" for q in right_arm_q)
        msg = f"{delta_result},{arm_str}"

        if right_hand_q is not None:
            hand_str = ",".join(f"{q:.3f}" for q in right_hand_q)
            msg += f",{hand_str}"
            msg += f",{hand_str}"

        self.ik_result_s.sendto(msg.encode(), self.ik_result_dest)
        if self.stats_out is not None:
            self.stats_out.mark(dest=f"{self.ik_result_dest[0]}:{self.ik_result_dest[1]}")
        self.last_arm_q, self.last_hand_q = right_arm_q, right_hand_q

    def close(self):
        self.wrist_listener_s.close()
        if self.ik_result_s:
            self.ik_result_s.close()
