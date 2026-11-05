"""Record right-glove hand shapes from Rokoko Studio for training a GeoRT hand mapping.

Saves <GeoRT>/data/<name>.npy ([N, 21, 3] points in GeoRT's canonical hand frame) and <name>.json (how it was
recorded). The file is rewritten every few seconds, so stopping with Ctrl+C (or the launcher's Stop) keeps
everything recorded so far. The teleop must not be running: both listen on the same glove port.

Usage: python scripts/record_hand_data.py --name kirsty --port 24043
"""
import json
import socket
import time
from argparse import ArgumentParser
from pathlib import Path

import numpy as np

from dexcap.sensors.rokoko_module import RokokoModule
from dexcap.utils.geort_mapping import GEORT_DIR, META_FILE, needs_mirror, to_canonical


def save(points, args, started):
    raw = np.array(points)
    mirror, score = needs_mirror(raw)
    out_dir = Path(args.geort_dir) / "data"
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp = out_dir / f"{args.name}.tmp.npy"
    np.save(tmp, to_canonical(raw, mirror).astype(np.float32))
    tmp.replace(out_dir / f"{args.name}.npy")
    meta = {"recording": args.name, "frames": len(raw), "seconds": round(time.time() - started, 1),
            "created": time.strftime("%Y-%m-%d %H:%M", time.localtime(started)), "mirror": mirror,
            "curl_score_mm": round(score * 1000, 1), "source": "rokoko right glove"}
    (out_dir / f"{args.name}.json").write_text(json.dumps(meta, indent=1))
    np.save(out_dir / f"{args.name}_raw.npy", raw.astype(np.float32))  # kept in case the frame needs redoing
    return meta


def main():
    parser = ArgumentParser()
    parser.add_argument("--name", required=True, help="Recording name (letters, numbers, - and _)")
    parser.add_argument("--port", type=int, default=24043, help="UDP port the Rokoko stream arrives on")
    parser.add_argument("--rate", type=float, default=50, help="Frames kept per second at most")
    parser.add_argument("--max_seconds", type=float, default=600, help="Stops by itself after this long")
    parser.add_argument("--geort_dir", default=GEORT_DIR)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", args.port))
    except OSError:
        print("The glove port is already in use by another program (the teleop, the hand test or the hand viewer). Stop it and try again.", flush=True)
        return
    sock.settimeout(1.0)
    print(f"Recording '{args.name}' from the Rokoko stream on port {args.port}. Move your right hand; "
          "stop when you have covered everything (2-3 minutes is plenty).", flush=True)

    points, last, last_kept, last_save, last_report = [], None, 0.0, time.time(), 0.0
    started = time.time()
    glove = "waiting for glove data"
    try:
        while time.time() - started < args.max_seconds:
            try:
                data, _ = sock.recvfrom(65535)
                body = json.loads(data.decode())["scene"]["actors"][0]["body"]
                right = RokokoModule.hand_points(body, "right")
                if right is None:
                    glove = "right glove missing from the stream"
                elif last is not None and np.array_equal(right, last):
                    glove = "right glove data not changing (powered off?)"
                else:
                    glove = "live"
                    last = right
                    if time.time() - last_kept >= 1 / args.rate:
                        last_kept = time.time()
                        points.append(right)
            except socket.timeout:
                glove = "no data from Rokoko Studio (is it streaming?)"
            except (ValueError, KeyError, IndexError):
                glove = "could not read a packet from Rokoko Studio"
            now = time.time()
            if now - last_report >= 1:
                last_report = now
                print(f"PROGRESS frames={len(points)} seconds={now - started:.0f} glove={glove}", flush=True)
            if points and now - last_save >= 5:
                last_save = now
                save(points, args, started)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    if not points:
        print("Nothing recorded: no live right-glove data arrived.", flush=True)
        return
    meta = save(points, args, started)
    print(f"Saved {meta['frames']} frames ({meta['seconds']:.0f} s) to {args.geort_dir}/data/{args.name}.npy", flush=True)
    if abs(meta["curl_score_mm"]) < 5:
        print("Warning: very little finger movement, so the hand's handedness is uncertain; record again with "
              "more opening and closing.", flush=True)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
