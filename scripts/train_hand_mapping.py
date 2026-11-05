"""Train a GeoRT glove -> LEAP hand mapping on a recording from scripts/record_hand_data.py.

Runs GeoRT's trainer with its own Python (/opt/geort-venv, a torch build that works with this laptop's GPU
driver), then copies the recording's details into the new model folder so the teleop knows how to read the
glove. A model is saved after every epoch, so stopping early (Ctrl+C) still leaves a usable model.

Usage: python3 scripts/train_hand_mapping.py --name kirsty
"""
import json
import shutil
import signal
import subprocess
import sys
from argparse import ArgumentParser
from pathlib import Path

GEORT_PYTHON = "/opt/geort-venv/bin/python"
GEORT_DIR = Path("/workspace/GeoRT")
HAND_NAME = "leap_right"
META_FILE = "dexcap_meta.json"


def main():
    parser = ArgumentParser()
    parser.add_argument("--name", required=True, help="Recording name; the model gets the same name")
    parser.add_argument("--python", default=GEORT_PYTHON)
    args = parser.parse_args()

    if not (GEORT_DIR / "data" / f"{args.name}.npy").exists():
        sys.exit(f"No recording called {args.name} in {GEORT_DIR / 'data'}")
    before = set(p.name for p in (GEORT_DIR / "checkpoint").glob(f"{HAND_NAME}_*"))
    # The same as `geort/trainer.py -hand leap_right -human_data <name> -ckpt_tag <name>` with its default loss
    # weights, but with the exact data file: the trainer's -human_data takes the first file in data/ whose
    # name merely contains <name> (e.g. <name>.json, or another recording called <name>_2).
    data = (GEORT_DIR / "data" / f"{args.name}.npy").as_posix()
    code = ("from pathlib import Path; from geort.trainer import GeoRTTrainer; "
            "from geort.utils.config_utils import get_config; "
            f"GeoRTTrainer(get_config({HAND_NAME!r})).train(Path({data!r}), tag={args.name!r}, "
            "w_chamfer=80.0, w_curvature=0.1, w_collision=0.0, w_pinch=1.0)")
    cmd = [args.python, "-u", "-W", "ignore", "-c", code]
    print(f"Running GeoRT's trainer: -hand {HAND_NAME} -human_data {args.name} on {data}", flush=True)
    proc = subprocess.Popen(cmd, cwd=GEORT_DIR)
    try:
        proc.wait()
    except KeyboardInterrupt:
        print("Stopped: keeping the model from the last finished epoch.", flush=True)
        proc.send_signal(signal.SIGINT)  # in case only this script got the Ctrl+C
        proc.wait()

    new = [p for p in (GEORT_DIR / "checkpoint").glob(f"{HAND_NAME}_*_{args.name}")
           if p.name not in before and (p / "last.pth").exists()]
    if not new:
        sys.exit("Training did not save a model, see the messages above.")
    model = max(new, key=lambda p: p.stat().st_mtime)
    meta = GEORT_DIR / "data" / f"{args.name}.json"
    if meta.exists():
        shutil.copy(meta, model / META_FILE)
    else:
        (model / META_FILE).write_text(json.dumps({"recording": args.name, "mirror": False}))
    epochs = len(list(model.glob("epoch_*.pth")))
    print(f"MODEL {model.name} epochs={epochs}", flush=True)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
