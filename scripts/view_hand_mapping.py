"""Show trained GeoRT hand mappings in simulation, driven live by the right Rokoko glove.

Each model's robot hand (LEAP, Allegro, ...) is drawn side by side in a pybullet window, next to the glove's
own hand points (an Allegro model also drives a LEAP hand next to it, through the Allegro -> LEAP conversion), all in GeoRT's hand frame (fingers up, palm facing the camera). Nothing real moves. The
teleop must not be running: both listen on the same glove port.

Usage: python scripts/view_hand_mapping.py --models /workspace/GeoRT/checkpoint/<folder> [<folder> ...]
Without --models it shows the model chosen in the launcher's Hand mapping tab (launcher/view_hand.bat, the
"Hand Mapping Viewer" desktop icon, starts it that way).
"""
import json
import os
import socket
import time
from argparse import ArgumentParser

import numpy as np
import pybullet as pb
from scipy.spatial.transform import Rotation

from dexcap.sensors.rokoko_module import RokokoModule
from dexcap.utils.geort_mapping import GEORT_DIR, LEAP_JOINT_ORDER, GeoRTHand, to_canonical

# GeoRT's hand frame -> world: fingers (+Z) up, palm normal (+X) towards -Y (the camera), thumb (+Y) along +X
HAND_TO_WORLD = Rotation.from_matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
SPACING = 0.3
# The LEAP hand is drawn with our AnySkin fingertips (GeoRT/assets/leap_right/leap_right_anyskin.urdf) when present
LEAP_ANYSKIN_URDF = os.path.join(GEORT_DIR, "assets/leap_right/leap_right_anyskin.urdf")
FINGER_COLORS = [(1, 0, 0), (1, 0.6, 0), (0, 0.7, 0), (0, 0.3, 1), (0.6, 0, 0.8)]  # thumb .. little


class SimHand:
    """One model's robot hand in the window, placed so its GeoRT base frame matches HAND_TO_WORLD.

    leap=True draws the LEAP hand driven by the model's LEAP angles (for an Allegro model: converted)."""

    def __init__(self, model, model_dir, x, leap=False):
        self.model, self.leap = model, leap
        config = json.load(open(os.path.join(GEORT_DIR, "geort/config/leap_right.json") if leap
                                else os.path.join(model_dir, "config.json")))
        self.joint_order = LEAP_JOINT_ORDER if leap else model.joint_order
        urdf = os.path.normpath(os.path.join(GEORT_DIR, config["urdf_path"]))
        if config["name"].startswith("leap") and os.path.exists(LEAP_ANYSKIN_URDF):
            urdf = LEAP_ANYSKIN_URDF
        self.robot = pb.loadURDF(urdf, useFixedBase=True)
        joints = [pb.getJointInfo(self.robot, j) for j in range(pb.getNumJoints(self.robot))]
        self.joint_index = {j[1].decode(): j[0] for j in joints}
        links = {j[12].decode(): j[0] for j in joints}
        for link in [-1] + list(links.values()):
            pb.changeVisualShape(self.robot, link, rgbaColor=(0.75, 0.75, 0.78, 1))
        # Root -> GeoRT base link transform, then place the root so the base link sits where we want it
        if config["base_link"] in links:
            state = pb.getLinkState(self.robot, links[config["base_link"]])
            base_p, base_q = np.array(state[4]), Rotation.from_quat(state[5])
        else:
            base_p, base_q = np.zeros(3), Rotation.identity()
        root_in_base_q = base_q.inv()
        root_in_base_p = -(base_q.inv().apply(base_p))
        world_q = HAND_TO_WORLD * root_in_base_q
        world_p = np.array([x, 0, 0]) + HAND_TO_WORLD.apply(root_in_base_p)
        pb.resetBasePositionAndOrientation(self.robot, world_p, world_q.as_quat())
        name = self.model.meta.get("recording", os.path.basename(model_dir))
        robot = "leap from allegro" if leap and model.allegro else config["name"].replace("_right", "")
        pb.addUserDebugText(f"{robot}: {name}", [x - 0.08, 0, -0.06], textSize=1.2)

    def update(self, raw_points):
        q_all = self.model.forward(raw_points) if self.leap else self.model.forward_native(raw_points)
        self.last_q = dict(zip(self.joint_order, q_all))
        for name, q in zip(self.joint_order, q_all):
            pb.resetJointState(self.robot, self.joint_index[name], q)


def main():
    parser = ArgumentParser()
    parser.add_argument("--models", nargs="*", help="GeoRT model folders to show side by side "
                        "(default: the model chosen in the launcher)")
    parser.add_argument("--port", type=int, default=24043, help="UDP port the Rokoko stream arrives on")
    args = parser.parse_args()
    if not args.models:
        settings = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "launcher", "launcher_settings.json")
        try:
            chosen = json.load(open(settings)).get("geort_model")
        except (OSError, ValueError):
            chosen = None
        if not chosen or not os.path.exists(os.path.join(GEORT_DIR, "checkpoint", chosen, "last.pth")):
            print("No hand mapping model chosen: in the launcher's Hand mapping tab, press Use this one next to a model.", flush=True)
            return
        args.models = [os.path.join(GEORT_DIR, "checkpoint", chosen)]
        print(f"Model chosen in the launcher: {chosen}", flush=True)

    pb.connect(pb.GUI)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
    hands = []
    for model_dir in args.models:
        model = GeoRTHand(model_dir)
        for leap in ([False, True] if model.allegro else [False]):
            hands.append(SimHand(model, model_dir, SPACING * (len(hands) + 1), leap))
    pb.addUserDebugText("your glove", [-0.06, 0, -0.06], textSize=1.2)
    mirror = hands[0].model.mirror
    center = SPACING * len(hands) / 2
    pb.resetDebugVisualizerCamera(0.45 + 0.18 * len(hands), 0, -10, [center, 0, 0.08])
    lines = [[-1] * 4 for _ in range(5)]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", args.port))
    except OSError:
        print("The glove port is already in use by another program (the teleop, the hand test or the hand viewer). Stop it and try again.", flush=True)
        return
    sock.settimeout(1.0)
    print(f"Showing {len(hands)} model(s); move your right hand.", flush=True)
    last_report, frames = 0.0, 0
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            sock.setblocking(False)
            try:
                while True:  # keep only the newest packet
                    data, _ = sock.recvfrom(65535)
            except BlockingIOError:
                pass
            sock.settimeout(1.0)
            right = RokokoModule.hand_points(json.loads(data.decode())["scene"]["actors"][0]["body"], "right")
        except socket.timeout:
            right = None
        except (ValueError, KeyError, IndexError):
            continue
        if right is not None:
            frames += 1
            for hand in hands:
                hand.update(right)
            # The glove's own hand in the same frame, at x = 0
            pts = HAND_TO_WORLD.apply(to_canonical(right, mirror))
            for f in range(5):
                chain = [0] + [1 + 4 * f + j for j in range(4)]
                for k in range(4):
                    lines[f][k] = pb.addUserDebugLine(pts[chain[k]], pts[chain[k + 1]], FINGER_COLORS[f], 3,
                                                      replaceItemUniqueId=lines[f][k])
        if time.time() - last_report >= 2:
            last_report = time.time()
            print(f"PROGRESS frames={frames} glove={'live' if right is not None else 'no data'}", flush=True)
            if right is not None and os.environ.get("VIEW_ANGLES"):  # joint angles each hand is showing
                for hand in hands:
                    print("  ", " ".join(f"{n}:{v:+.2f}" for n, v in hand.last_q.items()), flush=True)


if __name__ == "__main__":
    main()
