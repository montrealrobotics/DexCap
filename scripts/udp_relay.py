"""Long-running UDP relay inside the dexcap container.

Docker Desktop on Windows stops forwarding a published UDP port into the container once the
socket bound to it inside the container is closed (see the docker-udp-bug repro): the first
teleop run works, later runs receive nothing until the container is restarted. This relay
binds the published ports once and keeps them open, and passes every packet on to local ports
the teleop server listens on, so restarting the teleop never closes the forwarded socket.

Usage: python3 scripts/udp_relay.py 12346:22346 14043:24043   (published_port:local_port ...)
Run teleop_server.py with --pose_port 22346 --rokoko_port 24043.

The launcher checks that Docker is still forwarding by sending PROBE packets to the published ports from
the laptop; they are not passed on, only recorded in STATUS_FILE with the time of the last real packet.
"""
import json
import os
import selectors
import socket
import sys
import time

PROBE = b"udp-relay-probe"
STATUS_FILE = "/tmp/udp_relay_status.json"


def write_status(stats):
    tmp = STATUS_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({str(port): st for port, st in stats.items()}, f)
    os.replace(tmp, STATUS_FILE)


def main(pairs):
    sel = selectors.DefaultSelector()
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stats = {}
    for pair in pairs:
        listen_port, target_port = (int(p) for p in pair.split(":"))
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", listen_port))
        sel.register(s, selectors.EVENT_READ, (listen_port, target_port))
        stats[listen_port] = {"count": 0, "sender": None, "last_packet": None, "last_probe": None}
        print(f"relaying udp {listen_port} -> 127.0.0.1:{target_port}", flush=True)

    last_report = last_status = time.time()
    write_status(stats)
    while True:
        for key, _ in sel.select(timeout=1.0):
            listen_port, target_port = key.data
            try:
                data, addr = key.fileobj.recvfrom(65535)
                if data == PROBE:
                    stats[listen_port]["last_probe"] = time.time()
                    continue
                out.sendto(data, ("127.0.0.1", target_port))
            except OSError as e:
                print(f"port {listen_port}: {e}", flush=True)
                continue
            stats[listen_port]["count"] += 1
            stats[listen_port]["sender"] = addr[0]
            stats[listen_port]["last_packet"] = time.time()
        if time.time() - last_status >= 1:
            last_status = time.time()
            write_status(stats)
        if time.time() - last_report >= 30:
            last_report = time.time()
            print(time.strftime("%H:%M:%S ") + "  ".join(
                f"{port}: {st['count']} packets from {st['sender']}" for port, st in stats.items()), flush=True)
            for st in stats.values():
                st["count"] = 0


if __name__ == "__main__":
    main(sys.argv[1:])
