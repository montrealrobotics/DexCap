"""Drive the LEAP hand straight from the right glove with a trained GeoRT model, or with the teleop's fingertip
targets when no model is given, to try a hand mapping without the Quest, the arm or a recording (the launcher's
Hand mapping tab, step 4).

Sends the same command as the teleop ('right_leap_action' in the LEAP hand's redis). The teleop must not be
running: both listen on the same glove port. Stop with Ctrl+C; the launcher then sets the hand to neutral.

Usage: python scripts/test_hand_mapping.py --model /workspace/GeoRT/checkpoint/<folder> --redis_host 192.168.42.10
"""
import json
import pickle
import socket
import time
from argparse import ArgumentParser
from importlib.resources import files

import numpy as np
import pybullet as pb
import redis
from deoxys.utils import YamlConfig
from scipy.spatial.transform import Rotation

from dexcap.sensors.rokoko_module import RokokoModule
from dexcap.utils.geort_mapping import GeoRTHand
from dexcap.utils.ik_solver import LeapHandIKSolver
from dexcap.utils.utils import convert_to_hardware


class FingertipHand:
    """The teleop's hand mapping without a model (scripts/teleop_server.py): fingertip targets from the glove,
    scaled like RokokoModule does, then the LEAP fingertip IK, with the hand where the arm holds it at rest."""

    def __init__(self, arm, hand_tuning=None):
        pb.connect(pb.DIRECT)
        config = YamlConfig(str(files("dexcap").joinpath(f"configs/robot_config/{arm}_arm.yaml"))).as_easydict()
        self.solver = s = LeapHandIKSolver(config)
        self.hand_tuning, self.finger_length, self.last_read = hand_tuning, 1.0, 0.0
        # The wrist pose that solve_system_world turns into the arm's rest pose, so the arm IK is exact
        ee_pos, ee_quat = pb.getLinkState(s.right_arm, s.end_effector_index, computeForwardKinematics=True)[:2]
        palm = s.right_palm_orn_offset
        orn = Rotation.from_quat(ee_quat) * Rotation.from_euler("xyz", palm[3:]).inv()
        self.wrist_orn = orn * s.right_hand_orn_offset
        self.wrist_pos = np.asarray(ee_pos) - orn.apply(palm[:3])

    def forward(self, points):
        if self.hand_tuning and time.time() - self.last_read > 0.5:  # the launcher's finger length, live
            self.last_read = time.time()
            try:
                with open(self.hand_tuning) as f:
                    factor = float(json.load(f)["finger_length"])
                if 0.5 <= factor <= 3.0:
                    self.finger_length = factor
            except (OSError, ValueError, KeyError):
                pass
        scale = [s if i % 4 == 0 else s * self.finger_length for i, s in enumerate(RokokoModule.scale)]
        _, right = RokokoModule.adjust_bone_length(None, np.zeros((0, 3)), -points, scale)
        tips = self.wrist_orn.apply(right[RokokoModule.tip_id[0]]) + self.wrist_pos
        return list(self.solver.solve_system_world(self.wrist_pos, self.wrist_orn, tips[[1, 2, 3, 0]])[1])


def main():
    parser = ArgumentParser()
    parser.add_argument("--model", default=None, help="GeoRT model folder (none: the teleop's fingertip targets)")
    parser.add_argument("--arm", default="xarm", help="Arm the hand sits on, for where the fingertip targets are")
    parser.add_argument("--hand_tuning", default=None, help="Launcher's finger length file, for the fingertip targets")
    parser.add_argument("--port", type=int, default=24043, help="UDP port the Rokoko stream arrives on")
    parser.add_argument("--redis_host", required=True, help="LEAP hand's control computer")
    parser.add_argument("--redis_port", type=int, default=6669)
    parser.add_argument("--rate", type=float, default=30, help="Hand commands per second")
    args = parser.parse_args()

    hand = GeoRTHand(args.model) if args.model else FingertipHand(args.arm, args.hand_tuning)
    client = redis.Redis(args.redis_host, port=args.redis_port, db=0, socket_timeout=1.0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", args.port))
    except OSError:
        print("The glove port is already in use by another program (the teleop, the hand test or the hand viewer). Stop it and try again.", flush=True)
        return
    sock.settimeout(1.0)
    what = f"{args.model.rstrip('/').split('/')[-1]} ({hand.device})" if args.model else "fingertip targets (no model)"
    print(f"Testing {what}: the LEAP hand follows the right glove.", flush=True)

    sent, last, last_sent, last_report = 0, None, 0.0, 0.0
    glove = "waiting for glove data"
    try:
        while True:
            try:
                # Only the newest packet matters: drain the ones that queued up while the model ran
                sock.setblocking(True); sock.settimeout(1.0)
                data, _ = sock.recvfrom(65535)
                sock.setblocking(False)
                try:
                    while True:
                        data, _ = sock.recvfrom(65535)
                except BlockingIOError:
                    pass
                right = RokokoModule.hand_points(json.loads(data.decode())["scene"]["actors"][0]["body"], "right")
                if right is None:
                    glove = "right glove missing from the stream"
                elif last is not None and np.array_equal(right, last):
                    glove = "right glove data not changing (powered off?)"
                else:
                    glove, last = "live", right
                    if time.time() - last_sent >= 1 / args.rate:
                        last_sent = time.time()
                        client.set("right_leap_action", pickle.dumps(convert_to_hardware(hand.forward(right))))
                        sent += 1
            except socket.timeout:
                glove = "no data from Rokoko Studio (is it streaming?)"
            except (ValueError, KeyError, IndexError):
                glove = "could not read a packet from Rokoko Studio"
            except redis.RedisError as e:
                glove = f"LEAP hand not answering ({e})"
            if time.time() - last_report >= 1:
                last_report = time.time()
                print(f"PROGRESS sent={sent} glove={glove}", flush=True)
    except KeyboardInterrupt:
        print("Stopped.", flush=True)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
