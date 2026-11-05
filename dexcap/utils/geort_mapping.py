"""Glove -> LEAP hand joint mapping with GeoRT (https://github.com/facebookresearch/GeoRT, CC BY-NC 4.0).

GeoRT is cloned next to this repo (/workspace/GeoRT in the dexcap container) with a leap_right hand config.
scripts/record_hand_data.py records Rokoko right-hand points into GeoRT's data folder,
scripts/train_hand_mapping.py trains a model on them, and the teleop (--hand_mapping geort) uses it here.
A model trained for GeoRT's Allegro hand can drive the LEAP hand too: LEAP's motors match the Allegro's joints
one to one (LEAP_Hand_API's allegro_to_LEAPhand), except the sideways finger joints, which are held straight.

GeoRT works on 21 hand points in its canonical right-hand frame: origin at the wrist, +Z towards the middle
finger's knuckle, +Y towards the thumb side, +X out of the palm (so curling fingers move towards +X).
"""
import json
import os
import sys
from pathlib import Path

import numpy as np

GEORT_DIR = "/workspace/GeoRT"
HAND_NAME = "leap_right"
MODEL_PREFIXES = ("leap_right_", "allegro_right_")  # checkpoint folders that can drive the LEAP hand
# LEAP joint names in pybullet order, the order of the IK solvers' and leap_right models' output.
# A joint's name is also its LEAP motor number and its Allegro joint number.
LEAP_JOINT_ORDER = ["1", "0", "2", "3", "5", "4", "6", "7", "9", "8", "10", "11", "12", "13", "14", "15"]
LEAP_SIDEWAYS = {"0", "4", "8"}
TIPS, KNUCKLES = [8, 12, 16, 20], [5, 9, 13, 17]
META_FILE = "dexcap_meta.json"  # written next to each recording and copied into its trained models


def to_canonical(points, mirror=False):
    """Points [..., 21, 3] (wrist first, Rokoko/MediaPipe order) in GeoRT's canonical hand frame.

    mirror flips one axis first, for when the glove's coordinates turn a right hand into a left one.
    Same frame as GeoRT's hand_to_canonical (geort/mocap/manus_mocap_core.py).
    """
    points = np.array(points, dtype=np.float64)
    if mirror:
        points[..., 0] *= -1
    z = points[..., 9, :] - points[..., 0, :]
    z /= np.linalg.norm(z, axis=-1, keepdims=True)
    y_aux = points[..., 5, :] - points[..., 13, :]
    x = np.cross(y_aux, z)
    x /= np.linalg.norm(x, axis=-1, keepdims=True)
    y = np.cross(z, x)
    rotation = np.stack([x, y, z], axis=-1)  # columns are the hand axes
    return (points - points[..., :1, :]) @ rotation


def curl_direction(canonical):
    """Mean of (fingertip - knuckle) along X: positive for a right hand, whose fingers curl towards +X."""
    canonical = np.asarray(canonical)
    return float((canonical[..., TIPS, 0] - canonical[..., KNUCKLES, 0]).mean())


def needs_mirror(raw_points):
    """Whether recorded raw points [N, 21, 3] must be mirrored to look like a right hand to GeoRT.

    Fingers bend towards the palm, so across a recording with any finger movement the tips sit on the
    palm side of the knuckles on average. Returns (mirror, score); |score| under ~5 mm means too little
    movement to tell.
    """
    score = curl_direction(to_canonical(raw_points))
    return score < 0, score


def list_models(geort_dir=GEORT_DIR):
    """Trained leap_right and allegro_right models (newest first) as dicts: id (folder name), name, epochs, meta."""
    root = Path(geort_dir) / "checkpoint"
    models = []
    for d in [d for prefix in MODEL_PREFIXES for d in root.glob(f"{prefix}*")]:
        if not d.is_dir() or d.name.endswith("_last") or not (d / "last.pth").exists():
            continue
        meta = {}
        try:
            meta = json.loads((d / META_FILE).read_text())
        except (OSError, ValueError):
            pass
        models.append({"id": d.name, "name": meta.get("recording", d.name.split("_", 4)[-1]),
                       "epochs": len(list(d.glob("epoch_*.pth"))), "trained": d.stat().st_mtime, "meta": meta})
    return sorted(models, key=lambda m: -m["trained"])


class GeoRTHand:
    """A trained GeoRT model: raw Rokoko right-hand points -> 16 LEAP joint angles (pybullet joint order).

    forward() always gives LEAP angles; forward_native() gives the model's own robot's angles (in joint_order).
    """

    def __init__(self, model_dir, geort_dir=GEORT_DIR):
        if geort_dir not in sys.path:
            sys.path.insert(0, geort_dir)
        import torch
        from geort.model import IKModel
        from geort.formatter import HandFormatter
        from geort.utils.config_utils import load_json, parse_config_keypoint_info, parse_config_joint_limit

        model_dir = Path(model_dir)
        config = load_json(model_dir / "config.json")
        keypoints = parse_config_keypoint_info(config)
        self.human_ids = keypoints["human_id"]
        self.joint_order = config["joint_order"]
        self.allegro = config["name"].startswith("allegro")
        try:
            self.meta = json.loads((model_dir / META_FILE).read_text())
        except (OSError, ValueError):
            self.meta = {}
        self.mirror = bool(self.meta.get("mirror", False))
        # The teleop's Python may not have a CUDA build that matches the driver; the model is small enough for CPU
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = IKModel(keypoint_joints=keypoints["joint"]).to(self.device)
        self.model.load_state_dict(torch.load(model_dir / "last.pth", map_location=self.device))
        self.model.eval()
        self.formatter = HandFormatter(*parse_config_joint_limit(config))
        self.torch = torch

    def forward(self, raw_points):
        """raw_points: [21, 3] from RokokoModule.right_points. Returns 16 LEAP joint angles as a list."""
        q = self.forward_native(raw_points)
        if not self.allegro:
            return q
        # Allegro joint i -> LEAP joint "i" (same direction; LEAP's +pi motor offset is in convert_to_hardware)
        return [0.0 if name in LEAP_SIDEWAYS else q[int(name)] for name in LEAP_JOINT_ORDER]

    def forward_native(self, raw_points):
        """The model's own output: 16 angles for its robot, in joint_order."""
        keypoints = to_canonical(raw_points, self.mirror)[self.human_ids]
        with self.torch.no_grad():
            x = self.torch.from_numpy(keypoints).float().reshape(1, -1, 3).to(self.device)
            q = self.model(x).cpu().numpy()
        return self.formatter.unnormalize(q)[0].tolist()


def model_dir(model_id, geort_dir=GEORT_DIR):
    return os.path.join(geort_dir, "checkpoint", model_id)
