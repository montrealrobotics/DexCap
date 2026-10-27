import socket
import json
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as pb
from dexcap.sensors.rigidbodySento import create_primitive_shape

class RokokoModule:
    hand_link_names = ["Hand",
                   "ThumbProximal","ThumbMedial","ThumbDistal","ThumbTip",
                   "IndexProximal","IndexMedial","IndexDistal","IndexTip",
                   "MiddleProximal","MiddleMedial","MiddleDistal","MiddleTip",
                   "RingProximal","RingMedial","RingDistal","RingTip",
                   "LittleProximal","LittleMedial","LittleDistal","LittleTip"]
    
    first_kps = [0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19]
    second_kps = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
    scale = [1,1,0.7,0.7,1,0.7,0.7,0.7,1,0.7,0.7,0.7,1,0.7,0.7,0.7,1,0.7,0.7,0.7]

    tip_id = [[4, 8, 12, 16, 20],[3,7,11,15,19],[2,6,10,14,18],[1,5,9,13,17],[0,0,0,0,0]]
    #tip_id = list(range(21))
    def __init__(self, config, visualization=False):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 0)
        self.sock.bind(("", config.TELEOP.ROKOKO_PORT))
        # self.sock.setblocking(1)
        self.sock.settimeout(5)
        self.tip_vis_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tip_vis_dest = (config.TELEOP.VR_HOST, config.TELEOP.HAND_INFO_PORT)
        self.visualization = visualization
        self.cnt = 0
        self.hands_present = (False, False)
        # Last time each hand's raw data changed; a powered-off glove can stay in the
        # stream with frozen values, while a live glove always jitters slightly.
        self._last_raw = [None, None]
        self._last_change = [None, None]
        self._latest = None
        self._latest_ts = None
        self._lock = threading.Lock()
        self._running = False
        if self.visualization:
            pb.connect(pb.GUI)
            self.vis_sp_left = [create_primitive_shape(pb, 0.01, pb.GEOM_SPHERE, (0.01,), collidable=False) for _ in range(21)]
            self.vis_sp_right = [create_primitive_shape(pb, 0.01, pb.GEOM_SPHERE, (0.01,), collidable=False) for _ in range(21)]

    def round_floats(self, o):
        if isinstance(o, float):
            return round(o, 5)
        if isinstance(o, dict):
            return {k: self.round_floats(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [self.round_floats(x) for x in o]
        return o

    def Unpack(self, data):
        info = json.loads(data)
        left_link_positions = []
        if "leftHand" in info["scene"]["actors"][0]["body"].keys():
            raw_wrist_orn = info["scene"]["actors"][0]["body"]["leftHand"]["rotation"]
            wrist_orn = Rotation.from_quat(np.array([raw_wrist_orn["x"], raw_wrist_orn["y"], raw_wrist_orn["z"], raw_wrist_orn["w"]]))
            raw_wrist_position = info["scene"]["actors"][0]["body"]["leftHand"]["position"]
            wrist_position = np.array([raw_wrist_position["x"], raw_wrist_position["y"], raw_wrist_position["z"]])
            for link_name in RokokoModule.hand_link_names:
                link_name = "left" + link_name
                raw_pos = info["scene"]["actors"][0]["body"][link_name]["position"]
                pos = np.array([raw_pos["x"], raw_pos["y"], raw_pos["z"]])
                rel_pos = wrist_orn.inv().apply(pos - wrist_position)
                left_link_positions.append(rel_pos)
            left_link_positions = -np.array(left_link_positions)
        right_link_positions = []
        if "rightHand" in info["scene"]["actors"][0]["body"].keys():
            raw_wrist_orn = info["scene"]["actors"][0]["body"]["rightHand"]["rotation"]
            wrist_orn = Rotation.from_quat(np.array([raw_wrist_orn["x"], raw_wrist_orn["y"], raw_wrist_orn["z"], raw_wrist_orn["w"]]))
            raw_wrist_position = info["scene"]["actors"][0]["body"]["rightHand"]["position"]
            wrist_position = np.array([raw_wrist_position["x"], raw_wrist_position["y"], raw_wrist_position["z"]])
            for link_name in RokokoModule.hand_link_names:
                link_name = "right" + link_name
                raw_pos = info["scene"]["actors"][0]["body"][link_name]["position"]
                pos = np.array([raw_pos["x"], raw_pos["y"], raw_pos["z"]])
                rel_pos = wrist_orn.inv().apply(pos - wrist_position)
                right_link_positions.append(rel_pos)
            right_link_positions = -np.array(right_link_positions)
        # A glove that is not linked to the actor is missing from the packet; keep the
        # array shapes fixed so downstream indexing works, and report it via hands_present.
        self.hands_present = (len(left_link_positions) == 21, len(right_link_positions) == 21)
        now = time.time()
        for i, positions in enumerate((left_link_positions, right_link_positions)):
            raw = np.asarray(positions).tobytes()
            if raw != self._last_raw[i]:
                self._last_raw[i] = raw
                self._last_change[i] = now
        if not self.hands_present[0]:
            left_link_positions = np.zeros((21, 3))
        if not self.hands_present[1]:
            right_link_positions = np.zeros((21, 3))
        left_link_positions, right_link_positions = self.adjust_bone_length(left_link_positions, right_link_positions, RokokoModule.scale)
        tip_id = RokokoModule.tip_id[self.cnt]
        # self.cnt += 1
        # if self.cnt == 4:
        #     self.cnt = 0
        return left_link_positions[tip_id], right_link_positions[tip_id]

    def adjust_bone_length(self, left_positions, right_positions, factors = [1.0]*20):
        factors = np.array(factors) * 1.3
        if left_positions.shape[0] == 21:
            diff = left_positions[RokokoModule.second_kps] - left_positions[RokokoModule.first_kps]
            diff = diff * factors.reshape((-1,1))
            diff = diff.reshape((5,4,3))
            new_left_positions = diff.cumsum(axis=1).reshape(-1,3)
            left_positions = np.vstack([left_positions[0], new_left_positions])
        if right_positions.shape[0] == 21:
            diff = right_positions[RokokoModule.second_kps] - right_positions[RokokoModule.first_kps]
            diff = diff * factors.reshape((-1,1))
            diff = diff.reshape((5,4,3))
            new_right_positions = diff.cumsum(axis=1).reshape(-1,3)
            right_positions = np.vstack([right_positions[0], new_right_positions])
        return left_positions, right_positions

    def send_joint_data(self, tip_positions):
        """
        tip_positions: list of lists of 3 floats
        conn: socket connector
        """
        msg = {}
        msg["lt"] = tip_positions[0].tolist()
        msg["li"] = tip_positions[1].tolist()
        msg["lm"] = tip_positions[2].tolist()
        msg["lr"] = tip_positions[3].tolist()
        msg["rt"] = tip_positions[5].tolist()
        msg["ri"] = tip_positions[6].tolist()
        msg["rm"] = tip_positions[7].tolist()
        msg["rr"] = tip_positions[8].tolist()
        json_message = json.dumps(self.round_floats(msg))
        self.tip_vis_sock.sendto(json_message.encode(), self.tip_vis_dest)

    def receive(self):
        try:
            data, _ = self.sock.recvfrom(40000)
        except socket.timeout as e:
            raise
        left_positions, right_positions = self.Unpack(data.decode())
        return left_positions, right_positions
    
    def start_background(self, stats=None):
        """Receive glove packets on a thread so a silent glove never blocks the caller."""
        self._running = True
        self.sock.settimeout(1.0)

        def loop():
            while self._running:
                try:
                    left_positions, right_positions = self.receive()
                except socket.timeout:
                    continue
                except OSError:
                    break
                except Exception as e:
                    if stats is not None:
                        stats.set_detail(last_error=repr(e))
                    continue
                with self._lock:
                    self._latest = (left_positions, right_positions)
                    self._latest_ts = time.time()
                if stats is not None:
                    stats.mark(left_glove=self._glove_state(0), right_glove=self._glove_state(1))

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def _glove_state(self, hand, frozen_after=2.0):
        if not self.hands_present[hand]:
            return "MISSING"
        frozen = self.frozen_for(hand)
        if frozen is not None and frozen > frozen_after:
            return f"frozen {frozen:.0f}s (powered off?)"
        return "live"

    def frozen_for(self, hand):
        """Seconds since hand 0 (left) / 1 (right) last changed, or None if never received."""
        last = self._last_change[hand]
        return None if last is None else time.time() - last

    def latest(self, max_age=0.5):
        """Most recent (left, right) tip positions, or None if older than max_age seconds."""
        with self._lock:
            if self._latest_ts is None or time.time() - self._latest_ts > max_age:
                return None
            return self._latest

    def visualize_hand(self, left_positions, right_positions):
        for i in range(len(left_positions)):
            pb.resetBasePositionAndOrientation(self.vis_sp_left[i], left_positions[i], (0,0,0,1))
        for i in range(len(right_positions)):
            pb.resetBasePositionAndOrientation(self.vis_sp_right[i], right_positions[i], (0,0,0,1))
    
    def close(self):
        self._running = False
        self.sock.close()

if __name__ == "__main__":
    rokoko = RokokoModule("10.5.70.193",65432, 14043, visualization=True)
    scale = [1,0.7,0.7,0.7,1,0.7,0.7,0.7,1,0.7,0.7,0.7,1,0.7,0.7,0.7,1,0.7,0.7,0.7]
    while True:
        left_positions, right_positions = rokoko.receive()
        left_positions, right_positions = rokoko.adjust_bone_length(left_positions, right_positions, scale)
        rokoko.visualize_hand([], right_positions)
        #rokoko.send_joint_data(np.vstack([left_positions, right_positions]))