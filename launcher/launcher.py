"""Teleop launcher for the Windows laptop.

Starts the pieces of the teleop system in order and shows their health and logs
on one page (http://localhost:8090):

  0. Rokoko      - opens Rokoko Studio and turns on its livestream (Studio Command API)
  1. LEAP hand   - leap-hand-ros2 container on the control computer (the Jetson, over ssh)
  2. xArm        - deoxys_xarm_cont container on this laptop, runs xarm_client.py
  3. Teleop      - dexcap container on this laptop, runs scripts/teleop_server.py

The ARCap app in the headset is still started by hand.

Run with:  python launcher\\launcher.py   (standard library only)
"""
import json
import math
import pickle
import os
import re
import shlex
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]  # ...\src\DexCap (or DexCap_test)
SRC_DIR = REPO_DIR.parent  # mounted at /workspace in the dexcap container

CONFIG = {
    "port": 8090,
    "jetson_host": "192.168.42.10",
    "jetson_user": "real",
    "arm_ip": "192.168.42.227",
    # X display (VcXsrv on this laptop) for pybullet. Passed to every program the launcher runs, because
    # containers keep the DISPLAY they were created with, which may be empty.
    "display": "host.docker.internal:0.0",
    "vcxsrv": r"C:\Program Files\VcXsrv\vcxsrv.exe",
    "docker_desktop": r"C:\Program Files\Docker\Docker\Docker Desktop.exe",
    "rokoko_studio": r"C:\Program Files (x86)\Rokoko Electronics ApS\Rokoko Studio\Rokoko Studio.exe",
    # adb that comes with Meta Quest Developer Hub; used to keep the headset awake during a session
    "adb": r"C:\Program Files\Meta Quest Developer Hub\resources\bin\adb.exe",
    # Mirrors the headset view in a window (https://github.com/Genymobile/scrcpy), over the same adb
    "scrcpy": r"C:\Users\droid\tools\scrcpy-win64-v5.0.1\scrcpy.exe",
    # Studio settings > Command API; the livestream targets (custom JSONv3 to this laptop) are set up in Studio
    "rokoko_api": "http://127.0.0.1:14053/v1/1234",
    "xarm_compose": str(SRC_DIR / "deoxys_control" / "compose.yaml"),
    "xarm_service": "deoxys_xarm_gpu",
    "xarm_container": "deoxys_xarm_cont",
    "xarm_cmd": "cd /home/hw_ctrl_ws/src/deoxys_control && "
                "exec python -u deoxys/deoxys/robot_interfaces/xarm_interface/xarm_client.py {arm_ip} {gripper_flags}",
    "leap_compose_dir": "~/arcap_ws/src/LEAP_Hand_API",
    "leap_container": "leap-hand-ros2",
    "leap_redis_port": 6669,
    "teleop_compose": str(REPO_DIR / "docker-compose.yml"),
    "teleop_container": "dexcap",
    # The container's PYTHONPATH already points at deoxys; prepend this checkout so its dexcap package wins
    "teleop_cmd": "source /home/.venv/bin/activate && cd /workspace/{repo} && "
                  "PYTHONPATH=/workspace/{repo}:$PYTHONPATH exec python -u scripts/teleop_server.py {args}",
    "teleop_status_port": 8080,
    # usbipd attaches USB devices through a running WSL Linux; Docker's own WSL system can't load the driver
    "wsl_distro": "Ubuntu",
    # USB IDs (VID:PID) never offered as cameras: the laptop's built-in webcam
    "ignore_usb_ids": ["3277:0036"],
    # Attaches USB cameras to WSL so Docker can see them (https://github.com/dorssel/usbipd-win)
    "usbipd": r"C:\Program Files\usbipd-win\usbipd.exe",
    # deoxys compose file on the control computer, for running the xArm controller there
    "xarm_compose_remote": "~/arcap_ws/src/deoxys_control/compose.yaml",
    # Published port -> local port; scripts/udp_relay.py stays running in the container so restarting the
    # teleop never closes the published UDP ports (Docker Desktop stops forwarding after that)
    "udp_relay": {12346: 22346, 14043: 24043},
}

NO_WINDOW = 0x08000000 if sys.platform == "win32" else 0


def config_value(arm, section, key):
    """A value from dexcap/configs/<arm>.yaml (simple two-level lookup; no yaml package on Windows)."""
    try:
        text = (REPO_DIR / "dexcap" / "configs" / f"{arm}.yaml").read_text()
    except OSError:
        return None
    m = re.search(rf"^{section}:\s*\n((?:[ \t]+.*\n?)*)", text, re.M)
    if not m:
        return None
    v = re.search(rf"^[ \t]+{key}:\s*\"?([^\"#\n]+?)\"?\s*(?:#.*)?$", m.group(1), re.M)
    return v.group(1).strip() if v else None


def config_set(arm, section, key, value):
    """Change one value in dexcap/configs/<arm>.yaml in place, keeping quotes, comments and layout."""
    path = REPO_DIR / "dexcap" / "configs" / f"{arm}.yaml"
    text = path.read_text()
    block = re.search(rf"^{section}:\s*\n((?:[ \t]+.*\n?)*)", text, re.M)
    if not block:
        raise ValueError(f"{section} not found in {path.name}")
    line = re.search(rf"^([ \t]+{key}:[ \t]*)(\"?)([^\"#\n]*?)(\"?)([ \t]*(?:#.*)?)$", block.group(1), re.M)
    if not line:
        raise ValueError(f"{section}.{key} not found in {path.name}")
    start = block.start(1) + line.start(3)
    end = block.start(1) + line.end(3)
    path.write_text(text[:start] + value + text[end:])


# Settings page. IPs the teleop also reads live in the dexcap configs (one source of truth);
# everything only the launcher uses is kept in launcher_config.json next to this file.
LAUNCHER_CONFIG_FILE = Path(__file__).with_name("launcher_config.json")
CONFIG_FIELDS = [
    # key, group, label, stored as (arm config, section, key) / several of them / "launcher", kind
    # kind "host" and "text" are editable; "port" is shown read-only (see the note on the page)
    ("quest_ip", "Headset", "Quest IP", ("xarm", "TELEOP", "VR_HOST"), "host"),
    ("pose_port", "Headset", "Headset → laptop (hand and head poses)", ("xarm", "TELEOP", "POSE_CMD_PORT"), "port"),
    ("ik_port", "Headset", "Laptop → headset (robot pose)", ("xarm", "TELEOP", "IK_RESULT_PORT"), "port"),
    ("tip_port", "Headset", "Laptop → headset (fingertips)", ("xarm", "TELEOP", "HAND_INFO_PORT"), "port"),
    ("rokoko_port", "Rokoko gloves", "Rokoko Studio → laptop", ("xarm", "TELEOP", "ROKOKO_PORT"), "port"),
    ("xarm_ip", "xArm setup", "xArm controller IP", ("xarm", "ROBOT", "IP"), "host"),
    ("jetson_ip", "xArm setup", "Control computer IP", ("xarm", "CTRL_HOST", "CONTROL_COMPUTER_IP"), "host"),
    ("jetson_user", "xArm setup", "Control computer login", "launcher", "text"),
    ("xarm_ports", "xArm setup", "Arm commands / state", [("xarm", "CTRL_HOST", "ARM_SUB_PORT"),
                                                         ("xarm", "CTRL_HOST", "ARM_PUB_PORT")], "port"),
    ("xarm_hand_port", "xArm setup", "LEAP hand (redis)", ("xarm", "CTRL_HOST", "HAND_PORT"), "port"),
    ("nuc_ip", "Franka setup", "Control computer IP", ("franka", "CTRL_HOST", "IP_ETH"), "host"),
    ("franka_ip", "Franka setup", "Franka robot IP", ("franka", "ROBOT", "IP"), "host"),
    ("franka_ports", "Franka setup", "Arm commands / state", [("franka", "CTRL_HOST", "ARM_SUB_PORT"),
                                                             ("franka", "CTRL_HOST", "ARM_PUB_PORT")], "port"),
    ("franka_hand_port", "Franka setup", "LEAP hand (redis)", ("franka", "CTRL_HOST", "HAND_PORT"), "port"),
]
GROUP_NOTES = {
    "Headset": "The ports are built into the ARCap app and published in docker-compose.yml.",
    "Rokoko gloves": "Set in Rokoko Studio's custom stream and published in docker-compose.yml.",
    "xArm setup": "The LEAP hand runs on the control computer; the xArm controller runs on this laptop or the "
                  "control computer (chosen on the Data collection tab). The launcher starts both, over ssh where needed.",
    "Franka setup": "The Franka controller and its LEAP hand run on the control computer and are started there by hand.",
}
HOST_RE = re.compile(r"^[A-Za-z0-9]([A-Za-z0-9.-]{0,251}[A-Za-z0-9])?$")


def load_launcher_config():
    try:
        saved = json.loads(LAUNCHER_CONFIG_FILE.read_text())
    except (OSError, ValueError):
        return
    for key, _, _, where, _ in CONFIG_FIELDS:
        if where == "launcher" and isinstance(saved.get(key), str) and saved[key].strip():
            CONFIG[key] = saved[key].strip()


def config_fields():
    fields = []
    for key, group, label, where, kind in CONFIG_FIELDS:
        if where == "launcher":
            value = CONFIG.get(key, "")
        elif isinstance(where, list):
            value = " / ".join(config_value(*w) or "?" for w in where)
        else:
            value = config_value(*where) or ""
        fields.append({"key": key, "group": group, "label": label, "value": value, "editable": kind != "port"})
    return fields


def save_config(values):
    """Validate everything first, then write; returns a list of problems (empty when saved)."""
    errors, changes = [], []
    for key, _, label, where, kind in CONFIG_FIELDS:
        if key not in values or kind == "port":
            continue
        value = str(values[key]).strip()
        if kind == "host" and not HOST_RE.match(value):
            errors.append(f"{label}: '{value}' is not an IP address or host name")
        elif not value:
            errors.append(f"{label} can't be empty")
        else:
            changes.append((key, where, value))
    if errors:
        return errors
    launcher_values = {}
    try:
        launcher_values = json.loads(LAUNCHER_CONFIG_FILE.read_text())
    except (OSError, ValueError):
        pass
    for key, where, value in changes:
        if where == "launcher":
            CONFIG[key] = value
            launcher_values[key] = value
        elif config_value(*where) != value:
            config_set(*where, value)
    LAUNCHER_CONFIG_FILE.write_text(json.dumps(launcher_values, indent=1))
    return []


def xarm_ip():
    return config_value("xarm", "ROBOT", "IP") or CONFIG["arm_ip"]


def jetson_ip():
    """The control computer for the xArm setup (runs the LEAP hand, and the xArm controller if chosen)."""
    return config_value("xarm", "CTRL_HOST", "CONTROL_COMPUTER_IP") or CONFIG["jetson_host"]


def read_vr_host():
    cfg = REPO_DIR / "dexcap" / "configs" / "xarm.yaml"
    try:
        m = re.search(r'VR_HOST:\s*"?([\d.]+)', cfg.read_text())
        return m.group(1) if m else None
    except OSError:
        return None


def run(cmd, timeout=30, env=None):
    """Run a command, return (returncode, output). Never raises and never blocks past the timeout.

    stdin is closed so docker/ssh never wait on the launcher window's keyboard, and after a timeout the
    output pipes are abandoned rather than waited on (a leftover child process can hold them open).
    """
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                             text=True, encoding="utf-8", errors="replace", env=env, creationflags=NO_WINDOW)
    except OSError as e:
        return -1, str(e)
    try:
        out, _ = p.communicate(timeout=timeout)
        return p.returncode, out.strip()
    except subprocess.TimeoutExpired:
        p.kill()
        try:
            p.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            pass
        return -1, f"timed out after {timeout}s: {' '.join(cmd)}"


def ssh_cmd(remote):
    return ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
            f"{CONFIG['jetson_user']}@{jetson_ip()}", remote]


def _match_in_container(container, pattern, action, remote=False):
    """Run `action` (using $p) for every python process in the container whose command line matches pattern.

    Walks /proc in bash rather than using pkill/pgrep, which some container images don't have.
    """
    script = ('for d in /proc/[0-9]*; do p=${d#/proc/}; [ "$p" = "$$" ] || [ "$p" = "$PPID" ] && continue; '
              'grep -qa python "$d/cmdline" 2>/dev/null && '
              f'grep -qaE {shlex.quote(pattern)} "$d/cmdline" 2>/dev/null && {action}; '
              'done; true')
    cmd = ["docker", "exec", container, "bash", "-c", script]
    return run(ssh_cmd(shlex.join(cmd)) if remote else cmd, timeout=10)


def stop_in_container(container, pattern, remote=False):
    """SIGINT the matching python processes, the same as Ctrl+C in their terminal."""
    rc, out = _match_in_container(container, pattern, 'kill -INT "$p" && echo "stopped pid $p"', remote)
    return out or ("nothing was running" if rc == 0 else "could not reach container")


def running_in_container(container, pattern, remote=False):
    """True if a matching python process is running, e.g. one started by hand or by an earlier launcher."""
    rc, out = _match_in_container(container, pattern, 'echo "$p"', remote)
    return rc == 0 and bool(out.strip())


def tcp_open(host, port, timeout=1.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def redis_ping(host, port, timeout=1.0):
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.sendall(b"PING\r\n")
            return s.recv(64).startswith(b"+PONG")
    except OSError:
        return False


def redis_set(host, port, key, value, timeout=2.0):
    """SET a bytes value over the raw redis protocol (no redis package needed on Windows)."""
    key = key.encode()
    msg = b"*3\r\n$3\r\nSET\r\n$%d\r\n%s\r\n$%d\r\n%s\r\n" % (len(key), key, len(value), value)
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.sendall(msg)
            return s.recv(64).startswith(b"+OK")
    except OSError:
        return False


def ping(host):
    flag = "-n" if sys.platform == "win32" else "-c"
    wait = ["-w", "800"] if sys.platform == "win32" else ["-W", "1"]
    return run(["ping", flag, "1", *wait, host], timeout=5)[0] == 0


def process_running(name_fragment):
    if sys.platform != "win32":
        return False
    _, out = run(["tasklist", "/FO", "CSV", "/NH"], timeout=10)
    return name_fragment.lower() in out.lower()


def docker_ok():
    return run(["docker", "info", "--format", "{{.ServerVersion}}"], timeout=10)[0] == 0


class Step:
    """One part of the system, with notes for its row on the page."""

    def __init__(self, key, label, machine):
        self.key = key
        self.label = label
        self.machine = machine
        self.log = deque(maxlen=100)
        self.lock = threading.Lock()

    def say(self, line):
        self.log.append(time.strftime("%H:%M:%S ") + line)

    def running(self):
        return False

    def alive(self):
        return self.running()

    def start(self):
        with self.lock:
            if self.running():
                self.say("already running")
                return True
            return self._start()

    def stop(self):
        with self.lock:
            self._stop()

    def restart(self):
        self.stop()
        time.sleep(1)
        return self.start()

    def output_lines(self):
        return []

    def health(self):
        return ("ok" if self.alive() else "dead"), ""


class ContainerStep(Step):
    """A program inside a container, run detached with its output going to a log file in the container.

    It keeps running whether or not this launcher is open (an earlier version piped the output through
    the launcher, so closing the launcher window broke the programs with "broken pipe"). The page follows
    the log file, and a reopened launcher finds running programs and their logs again.
    """
    remote = False  # True: the container is on the control computer, reached over ssh
    shell = "-c"

    def __init__(self, key, label, machine, container, pattern):
        super().__init__(key, label, machine)
        self.container = container
        self.pattern = pattern
        self.log_file = f"/tmp/launcher/{key}.log"
        self.up = False
        self.output = deque(maxlen=400)
        self._follower = None
        self.jpeg_notices = 0  # "Corrupt JPEG data" lines from the camera decoder, counted instead of shown

    def _wrap(self, cmd):
        return ssh_cmd(shlex.join(cmd)) if self.remote else cmd

    def running(self):
        return self.up

    def refresh(self, reachable=True):
        """Called every couple of seconds: is the program running, and keep following its log."""
        if not reachable:
            self.up = False
            return
        rc, out = _match_in_container(self.container, self.pattern, 'echo "$p"', self.remote)
        self.up = rc == 0 and bool(out.strip())
        if rc == 0:  # the container is up, so its log can be followed
            self._follow()

    def _follow(self):
        if self._follower is not None and self._follower.poll() is None:
            return
        cmd = self._wrap(["docker", "exec", self.container, "tail", "-n", "150", "-F", self.log_file])
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                text=True, encoding="utf-8", errors="replace", bufsize=1, creationflags=NO_WINDOW)
        self._follower = proc
        lines = deque(maxlen=400)
        self.output = lines  # tail -n 150 re-sends recent lines, so start from an empty view

        self.jpeg_notices = 0

        def pump():
            for line in proc.stdout:
                if "tail:" in line and self.log_file in line:
                    continue  # tail's own "file appeared/truncated" notices
                if line.startswith("Corrupt JPEG data"):
                    # Expected over usbipd: the camera code drops those frames; just count them
                    self.jpeg_notices += 1
                    continue
                lines.append(line.rstrip())

        threading.Thread(target=pump, daemon=True).start()

    def output_lines(self):
        notice = ([f"({self.jpeg_notices} damaged camera frames from the USB link were noticed by the decoder; "
                   "they are skipped)"] if self.jpeg_notices else [])
        return list(self.output) + notice

    def _ensure_container(self, up_cmd, env=None):
        rc, out = run(self._wrap(["docker", "inspect", "-f", "{{.State.Running}}", self.container]))
        state, out = out.strip(), ""
        if rc != 0:
            self.say(f"container {self.container} not found, creating it")
            rc, out = run(self._wrap(up_cmd), timeout=300, env=env)
        elif state != "true":
            self.say(f"starting container {self.container}")
            rc, out = run(self._wrap(["docker", "start", self.container]), timeout=60)
        if out:
            self.say(out)
        return rc == 0

    def _launch(self, command, env=()):
        """Start command detached in the container, with its output in self.log_file."""
        script = (f"mkdir -p /tmp/launcher && exec > {self.log_file} 2>&1; "
                  f"echo \"=== started $(date '+%F %T') by the launcher\"; {command}")
        env_args = [a for e in env for a in ("-e", e)]
        rc, out = run(self._wrap(["docker", "exec", "-d", *env_args, self.container, "bash", self.shell, script]))
        if rc != 0:
            self.say(f"could not start: {out}")
            return False
        deadline = time.time() + 5
        while time.time() < deadline:
            self.refresh()
            if self.up:
                return True
            time.sleep(0.5)
        self.say("it exited straight away, see its output below")
        return False

    def start(self):
        with self.lock:
            self.refresh()
            if self.up:
                self.say("already running (its output is below)")
                return True
            return self._start()

    def stop(self):
        with self.lock:
            self.refresh()
            if self.up:
                # SIGINT, the same as Ctrl+C, so the programs shut down cleanly
                self.say(stop_in_container(self.container, self.pattern, self.remote))
                deadline = time.time() + 10
                while time.time() < deadline and self.up:
                    time.sleep(0.5)
                    self.refresh()
                if self.up:
                    self.say("still running 10 s after Ctrl+C; use Restart, or restart its container")
            self._after_stop()

    def _after_stop(self):
        pass


# What the person running the session picked on the page; kept between launches in launcher_settings.json
SETTINGS_FILE = Path(__file__).with_name("launcher_settings.json")
SETTINGS = {"arm": "xarm", "xarm_on": "laptop", "hand": "leap", "robot": "arm_hand",
            "device_roles": {},  # USB port (usbipd busid) -> role, e.g. {"1-20": "left", "1-5": "tactile_left"}
            "data_dir": str(REPO_DIR / "data")}
XARM_LOCATIONS = {"laptop": "This laptop", "control": "Control computer"}
CAMERA_ROLES = {"wrist": "Wrist", "left": "Left", "right": "Right"}
TACTILE_ROLES = {"tactile_left": "Left", "tactile_right": "Right"}
ROLE_ARGS = {"wrist": "--wrist_camera", "left": "--left_camera", "right": "--right_camera",
             "tactile_left": "--tactile_left", "tactile_right": "--tactile_right"}
ARMS = {"xarm": "xArm6", "franka": "Franka"}
# teleop_server.py drives the Franka without a gripper (panda_leap model), so only the LEAP hand for it
HANDS_BY_ARM = {"xarm": {"leap": "LEAP hand", "xarm_g": "xArm gripper"}, "franka": {"leap": "LEAP hand"}}
ALL_HANDS = {k: v for hands in HANDS_BY_ARM.values() for k, v in hands.items()}


def hands():
    return HANDS_BY_ARM[SETTINGS["arm"]]
# "arm_only" tests the arm without gloves (the hand / gripper stays in a fixed pose);
# "hand_only" runs the LEAP hand without the arm, e.g. to debug the hand on its own
ROBOT_MODES = {"arm_hand": "Arm + hand", "arm_only": "Arm only", "hand_only": "Hand only", "sim": "Simulation only"}


def container_path(path):
    """Path of a laptop folder inside the dexcap container, which only sees SRC_DIR (as /workspace)."""
    rel = Path(path).resolve().relative_to(SRC_DIR.resolve())  # ValueError if outside
    return "/workspace/" + rel.as_posix() if rel.parts else "/workspace"


def check_settings():
    try:
        container_path(SETTINGS["data_dir"])
    except (ValueError, OSError, TypeError):
        SETTINGS["data_dir"] = str(REPO_DIR / "data")
    if SETTINGS["arm"] not in ARMS:
        SETTINGS["arm"] = "xarm"
    if SETTINGS["xarm_on"] not in XARM_LOCATIONS:
        SETTINGS["xarm_on"] = "laptop"
    roles = SETTINGS["device_roles"] if isinstance(SETTINGS["device_roles"], dict) else {}
    SETTINGS["device_roles"] = {busid: role for busid, role in roles.items() if role in ROLE_ARGS}
    if SETTINGS["hand"] not in hands():
        SETTINGS["hand"] = "leap"
    if SETTINGS["robot"] not in ROBOT_MODES:
        SETTINGS["robot"] = "arm_hand"
    if SETTINGS["robot"] == "hand_only" and SETTINGS["hand"] != "leap":
        # The xArm gripper is driven through the arm's controller, so it cannot run without the arm
        SETTINGS["robot"] = "arm_hand"


def load_settings():
    try:
        saved = json.loads(SETTINGS_FILE.read_text())
        SETTINGS.update({k: v for k, v in saved.items() if k in SETTINGS})
    except (OSError, ValueError):
        pass
    check_settings()


def save_settings():
    try:
        SETTINGS_FILE.write_text(json.dumps(SETTINGS))
    except OSError:
        pass


def teleop_args():
    mode = SETTINGS["robot"]
    real_robot = mode != "sim"
    use_arm = mode in ("arm_hand", "arm_only")
    # The hand / gripper follows the Rokoko gloves; without them (arm only) it stays in a fixed pose
    use_hand = mode != "arm_only"
    relay = CONFIG["udp_relay"]
    # One Quest IP for both arms (xarm.yaml), so the arm configs can't disagree about the headset
    vr_host = read_vr_host()
    arm_host = ""
    if SETTINGS["arm"] == "xarm":
        arm_host = f"--arm_host {jetson_ip() if SETTINGS['xarm_on'] == 'control' else 'host.docker.internal'} "
    # Device numbers found when the launcher connected the devices (see connect_devices)
    cameras = "".join(f"{ROLE_ARGS[role]} {DEVICES['video'][busid]} " for busid, role in assigned_devices()
                      if busid in DEVICES["video"])
    return (f"--real_robot {real_robot} --use_arm {use_arm} --use_hand {use_hand} --use_gloves {use_hand} "
            f"--pose_port {relay[12346]} --rokoko_port {relay[14043]} "
            + (f"--vr_host {vr_host} " if vr_host else "") + arm_host + cameras +
            f"--robot_arm {SETTINGS['arm']} --gripper_type {SETTINGS['hand']} "
            f"--data_dir {shlex.quote(container_path(SETTINGS['data_dir']))}")


def browse_folder(initial):
    """Folder picker dialog on the laptop's screen; returns the chosen path or None."""
    import tkinter
    from tkinter import filedialog
    root = tkinter.Tk()
    root.withdraw()
    root.attributes("-topmost", True)  # otherwise it can open behind the browser
    try:
        chosen = filedialog.askdirectory(initialdir=initial, title="Save teleop recordings to", parent=root)
    finally:
        root.destroy()
    return chosen or None


def xarm_gripper_flags():
    """xarm_client.py gripper flags matching the chosen hand.

    With -g the client expects a gripper value in every command; the LEAP hand is driven over redis
    and the teleop sends arm joints only, so -g must be left off for it or every command is rejected.
    """
    return "" if SETTINGS["hand"] == "leap" else "-g"


def plan():
    """The steps the current settings need, in start order."""
    mode, franka = SETTINGS["robot"], SETTINGS["arm"] == "franka"
    steps = []
    if mode != "arm_only":
        steps.append("rokoko")
    if mode in ("arm_hand", "hand_only") and SETTINGS["hand"] == "leap":
        steps.append("leap_nuc" if franka else "leap")
    if mode in ("arm_hand", "arm_only"):
        steps.append("franka" if franka else "xarm")
    steps.append("teleop")
    return steps


def headset_serial():
    """(adb serial of the headset, None) or (None, why not)."""
    adb = CONFIG["adb"]
    if not Path(adb).exists():
        return None, f"adb not found at {adb}"

    def devices():
        _, out = run([adb, "devices"], timeout=10)
        # "emulator-…" entries are adb probing local ports, not the headset
        return [line.split("\t") for line in out.splitlines()[1:] if "\t" in line and not line.startswith("emulator-")]

    found = devices()
    vr_host = read_vr_host()
    if not any(state == "device" for _, state in found) and vr_host:
        run([adb, "connect", f"{vr_host}:5555"], timeout=10)
        found = devices()
    ready = [serial for serial, state in found if state == "device"]
    if ready:
        return ready[0], None
    if any(state == "unauthorized" for _, state in found):
        return None, "headset not authorized for adb: put it on and accept 'Allow USB debugging' (tick 'Always allow')"
    if any(state == "offline" for _, state in found):
        return None, "headset not answering adb: it is probably asleep (put it on to wake it)"
    return None, "headset not connected to adb: plug in the USB cable (or enable wireless adb)"


HEADSET_VIEW = {"proc": None}



def show_headset_view():
    """Open a window mirroring the headset (view only: no input is sent to the headset)."""
    proc = HEADSET_VIEW["proc"]
    if proc is not None and proc.poll() is None:
        return "the headset view is already open"
    if not Path(CONFIG["scrcpy"]).exists():
        return f"scrcpy not found at {CONFIG['scrcpy']}"
    serial, problem = headset_serial()
    if not serial:
        return problem
    # scrcpy brings its own adb; use Developer Hub's so the two don't keep restarting each other's server
    env = dict(os.environ, ADB=CONFIG["adb"])
    proc = subprocess.Popen([CONFIG["scrcpy"], "-s", serial, "--no-audio", "--no-control", "--max-size=1280",
                             "--max-fps=30", "--window-title=Headset view"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            text=True, encoding="utf-8", errors="replace", env=env, creationflags=NO_WINDOW)
    HEADSET_VIEW["proc"] = proc

    def report():
        for line in proc.stdout:
            if "ERROR" in line:
                say("headset view: " + line.strip())
        say(f"headset view closed (exit code {proc.wait()})")

    threading.Thread(target=report, daemon=True).start()
    return "headset view opening (it takes a few seconds)"


def headset_keep_awake(on):
    """Tell the Quest it is being worn (on) so it does not sleep when taken off, or restore normal sleep.

    Same as the proximity sensor toggle in Meta Quest Developer Hub; lasts until the headset reboots.
    Works over USB, or over WiFi once wireless adb was enabled (adb tcpip 5555 with the cable in).
    Returns a message for the launcher log.
    """
    adb = CONFIG["adb"]
    serial, problem = headset_serial()
    if not serial:
        return problem
    action = "prox_close" if on else "automation_disable"
    rc, out = run([adb, "-s", serial, "shell", "am", "broadcast", "-a", f"com.oculus.vrpowermanager.{action}"],
                  timeout=15)
    if rc != 0:
        return f"adb command failed: {out}"
    message = "headset kept awake" if on else "headset back to normal sleep"
    if on and ":" not in serial:
        # On USB: also turn on wireless adb (the headset turns it off at every reboot), so this keeps
        # working over WiFi after the cable is unplugged. The headset still only accepts authorized computers.
        rc, out = run([adb, "-s", serial, "tcpip", "5555"], timeout=15)
        message += "; wireless adb on, so the cable can be unplugged" if rc == 0 else f"; wireless adb failed: {out}"
    return message


def rokoko_command(command, args=None, timeout=5):
    """POST to the Rokoko Studio Command API; returns the decoded reply or None."""
    req = urllib.request.Request(f"{CONFIG['rokoko_api']}/{command}", data=json.dumps(args or {}).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except (OSError, ValueError):
        return None


def rokoko_gloves():
    """Glove device ids Studio currently sees, or None if the Command API is not answering."""
    reply = rokoko_command("info", {"devices_info": True, "actors_info": True}, timeout=2)
    if not reply or reply.get("response_code") != "OK":
        return None
    return [p for p in reply.get("parameters", []) if isinstance(p, str) and p.startswith("Smartgloves")]


class RokokoStep(Step):
    def __init__(self):
        super().__init__("rokoko", "Rokoko gloves", "Rokoko Studio, this laptop")
        self.streaming = False

    def _start(self):
        if not process_running("rokoko studio"):
            self.say("opening Rokoko Studio")
            subprocess.Popen([CONFIG["rokoko_studio"]])
        self.say("waiting for Studio's Command API (a scene must be open)")
        deadline = time.time() + 120
        while time.time() < deadline and rokoko_gloves() is None:
            time.sleep(2)
        gloves = rokoko_gloves()
        if gloves is None:
            self.say("Studio is not answering: open a scene, and check Settings > Command API is enabled")
            return False
        # Studio lists the gloves assigned to the scene whether or not they are powered on; whether data
        # is actually arriving shows on the teleop's "Rokoko gloves" card.
        self.say(f"gloves in the Studio scene: {', '.join(gloves) or 'none'}")
        reply = rokoko_command("livestream", {"enabled": True})
        self.streaming = bool(reply and reply.get("response_code") == "OK")
        self.say("livestream on" if self.streaming else f"could not turn on livestream: {reply}")
        return self.streaming

    def _stop(self):
        reply = rokoko_command("livestream", {"enabled": False})
        self.streaming = False
        self.say("livestream off" if reply and reply.get("response_code") == "OK" else f"livestream off failed: {reply}")

    def running(self):
        return self.streaming

    def health(self):
        gloves = rokoko_gloves()
        if gloves is None:
            return "dead", "Rokoko Studio not answering"
        return ("ok", "streaming") if self.streaming else ("stale", "not streaming")


LEAP_PATTERN = "launch_leap_redis|leap_hand_redis"
XARM_PATTERN = "xarm_client[.]py"
TELEOP_PATTERN = "teleop_server[.]py"
RELAY_PATTERN = "udp_relay[.]py"


def franka_hosts():
    """(NUC / arm control machine, Franka robot) from franka.yaml."""
    return config_value("franka", "CTRL_HOST", "IP_ETH"), config_value("franka", "ROBOT", "IP")


class ManualStep(Step):
    """A program started by hand on another machine (the Franka setup on the NUC): the launcher shows
    what to run there and waits until it answers, but cannot start or stop it itself."""
    manual = True

    def __init__(self, key, label, machine, instructions):
        super().__init__(key, label, machine)
        self.instructions = instructions
        self.started = False

    def running(self):
        return self.started

    def _start(self):
        for line in self.instructions():
            self.say(line)
        self.started = True
        return True

    def _stop(self):
        self.started = False


class FrankaStep(ManualStep):
    def __init__(self):
        super().__init__("franka", "Franka", "control computer (started by hand)", lambda: [
            f"On the control computer ({franka_hosts()[0]}): power on the Franka, unlock it, release the user stop, "
            "activate FCI,",
            "then in deoxys_control/deoxys run:  ./auto_scripts/auto_arm.sh config/franka.yml",
        ])

    def _stop(self):
        super()._stop()
        self.say("stop auto_arm.sh on the control computer (Ctrl+C) when you are done with the arm")

    def health(self):
        nuc, robot = franka_hosts()
        if not (nuc and ping(nuc)):
            return "dead", f"control computer ({nuc}) not reachable"
        if not (robot and ping(robot)):
            return "dead", f"Franka ({robot}) not reachable"
        # Whether auto_arm.sh is running shows up as the teleop's arm state stream
        return ("ok" if self.started else "stale"), "control computer and Franka reachable"


class LeapNucStep(ManualStep):
    def __init__(self):
        super().__init__("leap_nuc", "LEAP hand", "control computer (started by hand)", lambda: [
            f"On the control computer ({franka_hosts()[0]}): start the leap-hand-ros2 container and run",
            "  ros2 launch leap_hand launch_leap_redis.py",
        ])

    def _start(self):
        super()._start()
        if reset_leap_command(franka_hosts()[0]):
            self.say("hand command in redis reset to neutral (all joints 0)")
        return True

    def _stop(self):
        super()._stop()
        if reset_leap_command(franka_hosts()[0]):
            self.say("hand command in redis reset to neutral (all joints 0)")

    def health(self):
        nuc = franka_hosts()[0]
        if nuc and redis_ping(nuc, CONFIG["leap_redis_port"]):
            return "ok", f"redis answering on {nuc}:{CONFIG['leap_redis_port']}"
        return ("stale" if self.started else "dead"), "waiting for the LEAP hand on the control computer"




# Same as teleop_server.init_leaphand: every joint at 0, i.e. pi in LEAP hardware units
LEAP_NEUTRAL = pickle.dumps([math.pi] * 16)


def reset_leap_command(host):
    """Overwrite the last teleop hand command kept in the LEAP machine's redis.

    redis-server outlives the LEAP nodes (leap_hand_redis.py starts it in the background), so after a
    restart the bridge would otherwise send the hand straight back to the previous session's last pose.
    """
    return redis_set(host, CONFIG["leap_redis_port"], "right_leap_action", LEAP_NEUTRAL)


class LeapStep(ContainerStep):
    remote = True

    def __init__(self):
        super().__init__("leap", "LEAP hand", "control computer", CONFIG["leap_container"], LEAP_PATTERN)

    def reset_command(self):
        if reset_leap_command(jetson_ip()):
            self.say("hand command in redis reset to neutral (all joints 0)")
            return True
        return False

    def _start(self):
        up = ["bash", "-lc", f"cd {CONFIG['leap_compose_dir']} && docker compose up -d {self.container}"]
        if not self._ensure_container(up):
            self.say("could not start the container on the control computer, see above (is the ssh key set up?)")
            return False
        self.reset_command()
        if not self._launch("exec /ros_entrypoint.sh ros2 launch leap_hand launch_leap_redis.py",
                            env=["PYTHONUNBUFFERED=1"]):
            return False
        # If redis was not running it is started by the bridge, possibly reloading an old saved value
        deadline = time.time() + 20
        while time.time() < deadline and not self.reset_command():
            time.sleep(0.5)
        return True

    def _after_stop(self):
        self.reset_command()

    def health(self):
        if redis_ping(jetson_ip(), CONFIG["leap_redis_port"]):
            return "ok", f"redis answering on {jetson_ip()}:{CONFIG['leap_redis_port']}"
        if self.alive():
            return "stale", "launched, redis not answering yet"
        return "dead", "redis not answering"


class XArmStep(ContainerStep):
    shell = "-ic"  # interactive, so the container's .bashrc sets up its Python environment

    def __init__(self):
        super().__init__("xarm", "xArm", "deoxys_xarm_cont, this laptop", CONFIG["xarm_container"], XARM_PATTERN)

    @property
    def remote(self):
        # Same container name and layout on both machines; only where it runs differs
        return SETTINGS["xarm_on"] == "control"

    def location_changed(self):
        """Settings changed where the controller runs: follow the log on the new machine."""
        self.machine = "deoxys_xarm_cont, " + ("control computer" if self.remote else "this laptop")
        if self._follower is not None and self._follower.poll() is None:
            self._follower.kill()
        self._follower = None
        self.output = deque(maxlen=400)
        self.up = False

    def _start(self):
        if self.remote:
            up = ["bash", "-lc", f"docker compose -f {CONFIG['xarm_compose_remote']} up -d {CONFIG['xarm_service']}"]
        else:
            up = ["docker", "compose", "-f", CONFIG["xarm_compose"], "up", "-d", CONFIG["xarm_service"]]
        if not self._ensure_container(up, env=dict(os.environ, DISPLAY=CONFIG["display"])):
            return False
        return self._launch(CONFIG["xarm_cmd"].format(arm_ip=xarm_ip(), gripper_flags=xarm_gripper_flags()),
                            env=["PYTHONUNBUFFERED=1", f"DISPLAY={CONFIG['display']}"])

    def health(self):
        if self.running():
            return "ok", f"xarm_client.py running, arm {xarm_ip()}" + (" with gripper" if xarm_gripper_flags() else "")
        return "dead", "xarm_client.py not running"


class TeleopStep(ContainerStep):
    def __init__(self):
        super().__init__("teleop", "Teleop server", "dexcap container, this laptop",
                         CONFIG["teleop_container"], TELEOP_PATTERN)
        self.status = None
        self._status_via_exec = False

    def ensure_container(self):
        up = ["docker", "compose", "-f", CONFIG["teleop_compose"], "up", "-d"]
        return self._ensure_container(up, env=dict(os.environ, DISPLAY=CONFIG["display"]))

    def _start(self):
        if not self.ensure_container():
            return False
        if not self._ensure_relay(self.container):
            return False
        cmd = CONFIG["teleop_cmd"].format(repo=REPO_DIR.name, args=teleop_args())
        return self._launch(cmd, env=[f"DISPLAY={CONFIG['display']}"])

    def _ensure_relay(self, c):
        """Start the UDP relay once per container; it is never stopped, that is the point of it."""
        if running_in_container(c, RELAY_PATTERN):
            return True
        pairs = " ".join(f"{a}:{b}" for a, b in CONFIG["udp_relay"].items())
        script = f"exec python3 -u /workspace/{REPO_DIR.name}/scripts/udp_relay.py {pairs} >> /tmp/udp_relay.log 2>&1"
        rc, out = run(["docker", "exec", "-d", c, "bash", "-c", script])
        time.sleep(1)
        if rc != 0 or not running_in_container(c, RELAY_PATTERN):
            _, log = run(["docker", "exec", c, "tail", "-n", "5", "/tmp/udp_relay.log"])
            self.say(f"could not start the UDP relay: {out or log}")
            if "in use" in log:
                self.say("ports 12346/14043 are held by another program in the container (an older teleop?); "
                         "stop it, or restart the dexcap container")
            return False
        self.say(f"started UDP relay ({pairs}); it keeps the Quest and Rokoko ports open across teleop restarts")
        return True


    def fetch_status(self):
        """Teleop status JSON, from the published port or, if the container doesn't publish it, via docker exec."""
        port = CONFIG["teleop_status_port"]
        if not self._status_via_exec:
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/status.json", timeout=1) as r:
                    self.status = json.loads(r.read())
                    return
            except OSError:
                self._status_via_exec = True
        rc, out = run(["docker", "exec", CONFIG["teleop_container"], "python3", "-c",
                       "import urllib.request,sys;sys.stdout.write(urllib.request.urlopen("
                       f"'http://localhost:{port}/status.json',timeout=1).read().decode())"], timeout=5)
        try:
            self.status = json.loads(out) if rc == 0 else None
        except ValueError:
            self.status = None
        if self.status is None:
            self._status_via_exec = False  # try the published port again next time

    def health(self):
        if not self.alive():
            return "dead", "not running"
        if self.status is None:
            return "stale", "running, status page not answering yet"
        stage = self.status.get("info", {}).get("stage", "?")
        if PORTS.stuck():
            return "dead", "Docker is not forwarding the headset port, see above"
        if PORTS.quest_age is None or PORTS.quest_age > 3:
            # The port works (probes arrive), so the silence is on the headset side
            since = "yet" if PORTS.quest_age is None else f"for {PORTS.quest_age:.0f} s"
            return ("ok" if stage == "running" else "stale"), f"{stage}; no headset data {since} (port OK)"
        return ("ok" if stage == "running" else "stale"), stage


class PortCheck:
    """Checks that Docker Desktop still forwards the published UDP ports into the dexcap container.

    Docker Desktop can stop forwarding a UDP port (see scripts/udp_relay.py), and from inside the container
    that looks exactly like the headset not sending. So every few seconds a probe packet is sent to the
    published ports from this laptop; it takes the same path into the container as the headset's packets,
    and the relay records it in its status file without passing it on.
    """
    PROBE = b"udp-relay-probe"

    def __init__(self):
        self.result = {}  # port -> True (forwarding) / False (stuck); empty when unknown
        self.quest_age = None  # seconds since a real packet reached the relay on the Quest port
        self._prev = {}
        self._fails = {}
        self._sent = False
        self._last_run = 0

    def reset(self):
        self.__init__()

    def run(self):
        if time.time() - self._last_run < 5:
            return
        self._last_run = time.time()
        rc, out = run(["docker", "exec", CONFIG["teleop_container"], "bash", "-c",
                       "cat /tmp/udp_relay_status.json 2>/dev/null; echo; date +%s.%N"], timeout=10)
        lines = out.strip().splitlines()
        try:
            status, now = json.loads(lines[0]), float(lines[-1])
        except (ValueError, IndexError):
            self.reset()  # relay not running (yet)
            return
        ports = CONFIG["udp_relay"]
        if self._sent:
            for port in ports:
                arrived = status.get(str(port), {}).get("last_probe") not in (None, self._prev.get(port))
                self._fails[port] = 0 if arrived else self._fails.get(port, 0) + 1
                self.result[port] = self._fails[port] < 2  # two missed probes in a row: stuck
        self._prev = {port: status.get(str(port), {}).get("last_probe") for port in ports}
        last = status.get("12346", {}).get("last_packet")
        self.quest_age = None if last is None else max(0.0, now - last)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            for port in ports:
                sock.sendto(self.PROBE, ("127.0.0.1", port))
        self._sent = True

    def stuck(self):
        return [port for port, ok in self.result.items() if not ok]


PORTS = PortCheck()
# ---- Cameras and tactile sensors -------------------------------------------------------------------------
# USB video devices are plugged into the laptop; Docker Desktop only sees them once usbipd has bound (once,
# admin) and attached (after every replug/reboot) them to WSL. Roles are kept per USB port, because a ZED's
# video stream has no serial number; the /dev/video number is learned each time a device is attached.
TACTILE_RE = re.compile(r"gelsight|digit|tactile", re.I)
CAMERA_RE = re.compile(r"cam|zed|realsense|video|uvc|c9\d\d", re.I)
BUILT_IN_RE = re.compile(r"integrated|ir camera|dfu", re.I)  # laptop webcams
DEVICES = {"list": [], "t": 0, "video": {}}  # detected video devices; busid -> /dev/video number when connected
WSL_KEEPER = {"proc": None}
LIVE = {}  # busid -> process streaming that device's live view


def usb_devices():
    """Connected USB devices from `usbipd list`: busid, usb id, name, state (Not shared/Shared/Attached)."""
    if not Path(CONFIG["usbipd"]).exists():
        return []
    rc, out = run([CONFIG["usbipd"], "list"], timeout=15)
    devices = []
    for line in out.split("Persisted:")[0].splitlines():
        m = re.match(r"^(\d+-[\d.]+)\s+([0-9a-fA-F]{4}:[0-9a-fA-F]{4})\s+(.*?)\s{2,}(Not shared|Shared|Attached)\s*$", line)
        if m:
            devices.append({"busid": m.group(1), "usb_id": m.group(2).lower(), "name": m.group(3), "state": m.group(4)})
    return devices


def refresh_devices(force=False):
    if not force and time.time() - DEVICES["t"] < 5:
        return
    DEVICES["t"] = time.time()
    ignored = {i.lower() for i in CONFIG["ignore_usb_ids"]}
    found = []
    for dev in usb_devices():
        if dev["usb_id"] in ignored or BUILT_IN_RE.search(dev["name"]):
            continue
        kind = "tactile" if TACTILE_RE.search(dev["name"]) else "camera" if CAMERA_RE.search(dev["name"]) else None
        if kind:
            found.append(dict(dev, kind=kind))
    DEVICES["list"] = found
    DEVICES["video"] = video_map()


def assigned_devices():
    """(busid, role) for roles whose device is plugged in, in a fixed order."""
    plugged = {d["busid"] for d in DEVICES["list"]}
    order = list(ROLE_ARGS)
    return sorted(((b, r) for b, r in SETTINGS["device_roles"].items() if b in plugged), key=lambda x: order.index(x[1]))


def video_map():
    """usbipd port (busid) -> /dev/video number in the dexcap container, read from Linux's device records.

    Each attached device's usbip record holds its Windows port as (bus << 16) | port, e.g. 00010006 = 1-6, and
    each video node's sysfs path names the attached device it belongs to. Only image nodes (index 0) count.
    """
    script = ('cat /sys/devices/platform/vhci_hcd.0/status 2>/dev/null; echo ===; '
              'for d in /sys/class/video4linux/video*; do [ -e "$d" ] || break; '
              'echo "${d##*video} $(cat $d/index 2>/dev/null) $(readlink -f $d/device)"; done')
    rc, out = run(["docker", "exec", CONFIG["teleop_container"], "bash", "-c", script], timeout=10)
    if rc != 0 or "===" not in out:
        return {}
    status, videos = out.split("===", 1)
    remote = {}  # local usb busid inside Linux -> Windows busid
    for line in status.splitlines():
        parts = line.split()
        if len(parts) == 7 and parts[6] != "0-0" and re.fullmatch(r"[0-9a-fA-F]{8}", parts[4]):
            devid = int(parts[4], 16)
            remote[parts[6]] = f"{devid >> 16}-{devid & 0xFFFF}"
    mapping = {}
    for line in videos.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[0].isdigit() and parts[1] == "0":
            m = re.search(r"/usb\d+/(\d+-[\d.]+)/", parts[2] + "/")
            if m and m.group(1) in remote:
                busid = remote[m.group(1)]
                mapping[busid] = min(mapping.get(busid, int(parts[0])), int(parts[0]))
    return mapping


def keep_wsl_running():
    proc = WSL_KEEPER["proc"]
    if proc is None or proc.poll() is not None:
        WSL_KEEPER["proc"] = subprocess.Popen(["wsl", "-d", CONFIG["wsl_distro"], "-e", "sleep", "infinity"],
                                              stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                              stderr=subprocess.DEVNULL, creationflags=NO_WINDOW)
    # Wait until it is really running, or usbipd falls back to Docker's WSL and the attach fails
    deadline = time.time() + 20
    while time.time() < deadline:
        _, out = run(["wsl", "-l", "--running", "--quiet"], timeout=10)
        if CONFIG["wsl_distro"].lower() in out.replace("\x00", "").lower():
            return
        time.sleep(1)


def ensure_teleop_container():
    """The cameras' /dev/video numbers are read inside the dexcap container (and the teleop runs there), so it
    has to be running; Docker Desktop doesn't restart it after the laptop reboots."""
    rc, out = run(["docker", "inspect", "-f", "{{.State.Running}}", CONFIG["teleop_container"]], timeout=15)
    if rc == 0 and out.strip() == "true":
        return True
    progress("Starting the teleop container (dexcap)…")
    return STEPS["teleop"].ensure_container()


def connect_devices(busids):
    """Bind (once, with Windows' admin prompt) and attach the devices, then wait until each is a /dev/video."""
    stop_live()  # attaching changes the devices under any open live view
    if not docker_ok():
        progress("Starting Docker Desktop (this can take a minute)…")
        fix("docker")
        if not wait_for(docker_ok, 180, "Docker Desktop"):
            return
    if not ensure_teleop_container():
        say("could not start the dexcap container, see the Teleop server row")
        return
    usbipd = CONFIG["usbipd"]
    todo = []
    for busid in busids:
        dev = next((d for d in usb_devices() if d["busid"] == busid), None)
        if dev is None:
            say(f"USB port {busid}: nothing plugged in")
            continue
        if dev["state"] == "Not shared":
            progress(f"Allow the Windows prompt to share {dev['name']} with the robot software (once per device)…")
            run(["powershell", "-NoProfile", "-Command",
                 f"Start-Process -FilePath '{usbipd}' -ArgumentList 'bind','--busid','{busid}' -Verb RunAs -Wait"],
                timeout=180)
            dev = next((d for d in usb_devices() if d["busid"] == busid), dev)
            if dev["state"] == "Not shared":
                say(f"{dev['name']} (port {busid}): not shared (the Windows prompt was declined?)")
                continue
        todo.append(dev)
    if any(d["state"] != "Attached" for d in todo):
        progress("Starting the Ubuntu WSL system (needed to connect USB devices)…")
        keep_wsl_running()
    for dev in todo:
        if dev["state"] != "Attached":
            progress(f"Connecting {dev['name']}…")
            # Attaching sometimes fails once (e.g. a camera still resetting after it was detached): retry
            for attempt in range(3):
                rc, out = run([usbipd, "attach", "--wsl", "--busid", dev["busid"]], timeout=60)
                if rc == 0:
                    break
                time.sleep(3)
            if rc != 0:
                say(f"{dev['name']} (port {dev['busid']}): could not connect: {out.splitlines()[-1] if out else rc}")
    progress("Waiting for the devices to appear…")
    deadline = time.time() + 15
    mapping = video_map()
    while time.time() < deadline and any(d["busid"] not in mapping for d in todo):
        time.sleep(1)
        mapping = video_map()
    for dev in todo:
        n = mapping.get(dev["busid"])
        say(f"{dev['name']} (port {dev['busid']}): " + (f"connected as /dev/video{n}" if n is not None
                                                         else "connected, but no camera appeared in the teleop container"))
    refresh_devices(force=True)


# Runs inside the dexcap container: streams small JPEGs of one camera to stdout (4-byte length + JPEG) until
# the launcher closes stdin, then releases the camera. So a closed page or tab never leaves a camera held.
LIVE_CODE = r"""
import cv2, numpy as np, select, struct, sys, time
cap = cv2.VideoCapture(int(sys.argv[1]))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # uncompressed webcam video is too much for usbipd
# Decode MJPEG here so frames that arrive incomplete over usbipd are skipped, not shown half grey
raw = int(cap.get(cv2.CAP_PROP_FOURCC)) == cv2.VideoWriter_fourcc(*"MJPG")
if raw:
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
out = sys.stdout.buffer
end = time.time() + 900
try:
    while time.time() < end:
        if select.select([sys.stdin], [], [], 0)[0] and not sys.stdin.buffer.read(1):
            break
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        if raw:
            data = frame.reshape(-1)
            if not (data[:2].tobytes() == b"\xff\xd8" and data.tobytes().rfind(b"\xff\xd9") > 2):
                continue
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if frame is None or (np.ptp(frame[-16:, -128:].reshape(-1, 3), axis=0) <= 1).all():
                continue  # lost data on the way: its last block is repeated to the end
        h, w = frame.shape[:2]
        if w / h > 2.4:
            frame = frame[:, :w // 2]  # stereo camera: left image
        h, w = frame.shape[:2]
        frame = cv2.resize(frame, (400, int(h * 400 / w)))
        jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
        out.write(struct.pack(">I", len(jpg)) + jpg)
        out.flush()
        time.sleep(0.08)
except (BrokenPipeError, OSError):
    pass
finally:
    cap.release()
"""


def stop_live_proc(proc):
    try:
        proc.stdin.close()  # the streamer sees end-of-input and releases the camera
    except OSError:
        pass
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()


def stop_live():
    for busid in list(LIVE):
        stop_live_proc(LIVE.pop(busid))


def live_frames(busid):
    """Yields JPEG frames from a connected device, or nothing if it can't be streamed now."""
    n = DEVICES["video"].get(busid)
    if n is None or STEPS["teleop"].alive() or BUSY["what"]:
        return
    if busid in LIVE:
        stop_live_proc(LIVE.pop(busid))
    proc = subprocess.Popen(["docker", "exec", "-i", CONFIG["teleop_container"], "/home/.venv/bin/python", "-c",
                             LIVE_CODE, str(n)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, creationflags=NO_WINDOW)
    LIVE[busid] = proc
    try:
        while True:
            header = proc.stdout.read(4)
            if len(header) < 4:
                return
            jpg = proc.stdout.read(struct.unpack(">I", header)[0])
            yield jpg
    finally:
        if LIVE.get(busid) is proc:
            LIVE.pop(busid)
        stop_live_proc(proc)


def restart_dexcap():
    """The fix for stuck UDP forwarding: restart the dexcap container, then the relay and the teleop."""
    progress("Restarting the dexcap container to clear the stuck port…")
    STEPS["teleop"].stop()
    rc, out = run(["docker", "restart", CONFIG["teleop_container"]], timeout=180)
    PORTS.reset()
    if rc != 0:
        say(f"could not restart the dexcap container: {out}")
        return
    if STEPS["teleop"].start():
        say("dexcap container restarted and the teleop server started again")


STEPS = {s.key: s for s in (RokokoStep(), LeapStep(), LeapNucStep(), XArmStep(), FrankaStep(), TeleopStep())}
LAUNCH_LOG = deque(maxlen=200)
CHECKS = {}
BUSY = {"what": None, "step": ""}  # "starting" / "stopping" / "restarting" while a sequence runs
LAST_CHECK = {"t": time.time()}  # when the background checks last completed a pass


def say(line):
    LAUNCH_LOG.append(time.strftime("%H:%M:%S ") + line)


def progress(text):
    """What the start/stop sequence is doing now, shown in the page's banner."""
    BUSY["step"] = text
    say(text)


def run_checks():
    while True:
        try:
            check_once()
        except Exception as e:  # keep checking; a crash here would freeze the whole page
            say(f"check error: {e!r}")
        LAST_CHECK["t"] = time.time()
        time.sleep(2)


def check_once():
    global CHECKS
    vr_host = read_vr_host()
    nuc, franka = franka_hosts()
    steps = plan()
    # key: (label shown when it fails, ok, which plan steps need it, fix button)
    checks = {
        "docker": ("Docker Desktop is not running", docker_ok(), "teleop", "docker"),
        "vcxsrv": ("VcXsrv (window for the robot simulation) is not running",
                   process_running("vcxsrv.exe"), "teleop", "vcxsrv"),
        "quest": (f"Quest headset ({vr_host}) is not reachable: is it on and on the WiFi?",
                  bool(vr_host) and ping(vr_host), "teleop", None),
        "arm": (f"xArm control box ({xarm_ip()}) is not reachable: is the arm powered on?",
                "xarm" not in steps or ping(xarm_ip()), "xarm", None),
        "nuc": (f"Control computer ({nuc}) is not reachable: is the laptop's Ethernet on the Franka network (172.16.0.2)?",
                not {"franka", "leap_nuc"} & set(steps) or bool(nuc and ping(nuc)), ("franka", "leap_nuc"), None),
        "franka": (f"Franka ({franka}) is not reachable: is it powered on?",
                   "franka" not in steps or bool(franka and ping(franka)), "franka", None),
        "jetson": (f"Control computer ({jetson_ip()}) is not reachable",
                   not (set(steps) & {"leap", "xarm"}) or tcp_open(jetson_ip(), 22),
                   ("leap", "xarm") if SETTINGS["xarm_on"] == "control" else "leap", None),
        "rokoko": ("Rokoko Studio is not open (Start opens it)", process_running("rokoko"), "rokoko", None),
    }
    CHECKS = checks
    docker_up = checks["docker"][1]
    STEPS["xarm"].refresh(tcp_open(jetson_ip(), 22) if STEPS["xarm"].remote else docker_up)
    STEPS["teleop"].refresh(docker_up)
    refresh_devices()
    # The control computer is only polled while it is in use or something is still running there
    if "leap" in steps or STEPS["leap"].up:
        STEPS["leap"].refresh(tcp_open(jetson_ip(), 22))
    if STEPS["teleop"].alive():
        STEPS["teleop"].fetch_status()
        PORTS.run()
    else:
        STEPS["teleop"].status = None
        PORTS.reset()


def wait_for(cond, timeout, what):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cond():
            return True
        time.sleep(1)
    say(f"gave up waiting for {what} after {timeout}s")
    return False


def start_all():
    steps = plan()
    say("starting: " + ", ".join(STEPS[k].label for k in steps))
    if not docker_ok():
        progress("Starting Docker Desktop (this can take a minute)…")
        fix("docker")
        if not wait_for(docker_ok, 180, "Docker Desktop"):
            return
    if not process_running("vcxsrv.exe"):
        say("starting VcXsrv")
        fix("vcxsrv")
    if "rokoko" in steps:
        progress("Starting Rokoko streaming…")
    if "rokoko" in steps and not STEPS["rokoko"].start():
        # Not fatal: the teleop server holds the robot while glove data is missing
        say("Rokoko streaming did not start, see the Rokoko row; continuing")
    for leap, timeout in (("leap", 90), ("leap_nuc", 300)):
        if leap in steps:
            progress("Start the LEAP hand on the control computer (see the LEAP hand row); waiting for it…"
                     if leap == "leap_nuc" else "Starting the LEAP hand…")
            if not STEPS[leap].start() or not wait_for(lambda: STEPS[leap].health()[0] == "ok", timeout, "LEAP hand"):
                say("stopped: the LEAP hand did not come up, see the LEAP hand row")
                return
    if "franka" in steps:
        progress("Waiting for the control computer and the Franka…")
        STEPS["franka"].start()
        if not wait_for(lambda: STEPS["franka"].health()[0] == "ok", 60, "control computer and Franka"):
            say("stopped: the control computer or the Franka is not reachable, see the Franka row")
            return
        say("make sure auto_arm.sh is running on the control computer; the teleop waits for the arm controller")
    if "xarm" in steps:
        progress("Starting the xArm controller…")
        if not STEPS["xarm"].start():
            say("stopped: the xArm container did not start, see the xArm row")
            return
        time.sleep(5)
        STEPS["xarm"].refresh()
        if not STEPS["xarm"].alive():
            say("stopped: the xArm client exited (arm powered on? e-stop released?), see the xArm row")
            return
    say(headset_keep_awake(True))
    stop_live()  # the teleop needs the cameras
    if assigned_devices():
        progress("Connecting cameras and tactile sensors…")
        connect_devices([busid for busid, _ in assigned_devices()])
    progress("Starting the teleop server…")
    if not STEPS["teleop"].start() or not wait_for(lambda: STEPS["teleop"].status is not None, 120, "teleop server"):
        say("the teleop server did not come up, see the Teleop server row")
        return
    say("started")


def stop_all():
    progress("Stopping everything…")
    for key in ("teleop", "xarm", "franka", "leap", "leap_nuc", "rokoko"):
        STEPS[key].stop()
    say(headset_keep_awake(False))
    say("stopped")


def run_busy(what, fn):
    if BUSY["what"]:
        return
    BUSY["what"], BUSY["step"] = what, ""
    try:
        fn()
    except Exception as e:  # never leave the page stuck in "Starting…"
        say(f"error: {e!r}")
    finally:
        BUSY["what"], BUSY["step"] = None, ""


def fix(check):
    if check == "docker":
        subprocess.Popen([CONFIG["docker_desktop"]], creationflags=NO_WINDOW)
    elif check == "vcxsrv":
        # Same as XLaunch with "Multiple windows", display 0 and "Disable access control"
        subprocess.Popen([CONFIG["vcxsrv"], ":0", "-multiwindow", "-clipboard", "-wgl", "-ac"], creationflags=NO_WINDOW)


def state():
    steps = plan()
    rows = []
    idle = not BUSY["what"] and not any(STEPS[k].alive() for k in STEPS)
    for key in steps:
        s = STEPS[key]
        h, detail = s.health()
        if idle and h == "dead":
            h, detail = "off", "off"  # nothing started yet: grey, not an error
        rows.append({"key": key, "label": s.label, "where": s.machine, "health": h, "detail": detail,
                     "manual": getattr(s, "manual", False),
                     "log": [f"[launcher] {line}" for line in list(s.log)[-20:]] + s.output_lines()[-200:]})
    # Programs from a different setup that are still running, so they can be stopped
    leftovers = [STEPS[k].label for k in STEPS if k not in steps and STEPS[k].alive()]
    problems = [{"text": label, "fix": fx, "fix_label": "Start it"} for (label, ok, needed_by, fx) in CHECKS.values()
                if not ok and set(needed_by if isinstance(needed_by, tuple) else (needed_by,)) & set(steps)]
    plugged = {d["busid"] for d in DEVICES["list"]}
    for busid, role in SETTINGS["device_roles"].items():
        if busid not in plugged:
            what = ("Tactile sensor " + TACTILE_ROLES[role].lower()) if role in TACTILE_ROLES else f"{CAMERA_ROLES[role]} camera"
            problems.append({"text": f"{what} (USB port {busid}) isn't plugged in, so it won't be recorded",
                             "fix": None, "fix_label": ""})
    if PORTS.stuck():
        problems.insert(0, {"text": f"Docker has stopped forwarding UDP port(s) {', '.join(map(str, PORTS.stuck()))} "
                                    "into the dexcap container, so headset/glove data can't reach the teleop",
                            "fix": "dexcap", "fix_label": "Restart the dexcap container"})
    running = any(STEPS[k].alive() for k in STEPS)
    return {
        "settings": SETTINGS,
        "arms": ARMS,
        "xarm_locations": XARM_LOCATIONS,
        "devices": [dict(d, role=SETTINGS["device_roles"].get(d["busid"], ""), video=DEVICES["video"].get(d["busid"]))
                    for d in DEVICES["list"]],
        "camera_roles": CAMERA_ROLES,
        "tactile_roles": TACTILE_ROLES,
        "missing_roles": {b: r for b, r in SETTINGS["device_roles"].items()
                          if b not in {d["busid"] for d in DEVICES["list"]}},
        "hands": hands(),
        "robot_modes": ROBOT_MODES,
        "src_dir": str(SRC_DIR),
        "rows": rows,
        "problems": problems,
        "leftovers": leftovers,
        "running": running,
        "busy": BUSY["what"],
        "busy_step": BUSY["step"],
        "checks_age": time.time() - LAST_CHECK["t"],
        "ready": all(r["health"] == "ok" for r in rows) and not BUSY["what"],
        "teleop": STEPS["teleop"].status,
        "launch_log": list(LAUNCH_LOG),
        "teleop_args": teleop_args(),
        "config": config_fields(),
        "config_notes": GROUP_NOTES,
    }


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        body = body.encode() if isinstance(body, str) else body
        try:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
            pass  # the page reloaded or closed mid-reply

    def do_GET(self):
        if self.path == "/":
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/api/state":
            self._send(200, json.dumps(state()))
        elif self.path.startswith("/api/live/"):
            busid = urllib.parse.unquote(self.path.split("/api/live/")[1].split("?")[0])
            frames = live_frames(busid)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                for jpg in frames:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg)
                                     + jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass  # the page closed the view
            finally:
                frames.close()
        else:
            self._send(404, "{}")

    def do_POST(self):
        parts = self.path.strip("/").split("/")  # api/<target>/<action>
        if parts == ["api", "settings"]:
            if BUSY["what"] or any(STEPS[k].alive() for k in STEPS):
                return self._send(409, json.dumps({"error": "stop everything before changing the setup"}))
            try:
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            except ValueError:
                return self._send(400, "{}")
            if body.get("arm") in ARMS:
                SETTINGS["arm"] = body["arm"]
            if body.get("xarm_on") in XARM_LOCATIONS and body["xarm_on"] != SETTINGS["xarm_on"]:
                SETTINGS["xarm_on"] = body["xarm_on"]
                STEPS["xarm"].location_changed()
            if isinstance(body.get("device_roles"), dict):  # the whole assignment from the Cameras tab
                roles = {str(b): r for b, r in body["device_roles"].items() if r}
                kinds = {d["busid"]: d["kind"] for d in DEVICES["list"]}
                if (any(r not in ROLE_ARGS for r in roles.values()) or len(set(roles.values())) != len(roles)
                        or any(kinds.get(b, "tactile" if r in TACTILE_ROLES else "camera")
                               != ("tactile" if r in TACTILE_ROLES else "camera") for b, r in roles.items())):
                    return self._send(400, json.dumps({"error": "each role takes one device of the right kind"}))
                SETTINGS["device_roles"] = roles
            if body.get("hand") in ALL_HANDS:
                SETTINGS["hand"] = body["hand"]
            if body.get("robot") in ROBOT_MODES:
                SETTINGS["robot"] = body["robot"]
            if "data_dir" in body:
                try:
                    container_path(body["data_dir"])
                except (ValueError, OSError, TypeError):
                    return self._send(400, json.dumps({"error": f"The folder must be inside {SRC_DIR}, "
                                                                "the part of the laptop the teleop container can see"}))
                SETTINGS["data_dir"] = str(Path(body["data_dir"]).resolve())
            check_settings()
            save_settings()
            return self._send(200, "{}")
        if parts == ["api", "browse"]:
            if BUSY["what"] or any(STEPS[k].alive() for k in STEPS):
                return self._send(409, json.dumps({"error": "stop everything before changing the setup"}))
            chosen = browse_folder(SETTINGS["data_dir"] if Path(SETTINGS["data_dir"]).exists() else str(SRC_DIR))
            return self._send(200, json.dumps({"path": chosen}))
        if parts == ["api", "config"]:
            if BUSY["what"] or any(STEPS[k].alive() for k in STEPS):
                return self._send(409, json.dumps({"errors": ["Stop everything before changing settings"]}))
            try:
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                errors = save_config(body)
            except (ValueError, OSError) as e:
                errors = [str(e)]
            return self._send(400 if errors else 200, json.dumps({"errors": errors}))
        if len(parts) != 3 or parts[0] != "api":
            return self._send(404, "{}")
        target, action = parts[1], parts[2]
        if target == "all" and action == "start":
            threading.Thread(target=run_busy, args=("starting", start_all), daemon=True).start()
        elif target == "all" and action == "stop":
            threading.Thread(target=run_busy, args=("stopping", stop_all), daemon=True).start()
        elif target in STEPS and action == "restart":
            threading.Thread(target=STEPS[target].restart, daemon=True).start()
        elif target == "fix" and action in ("docker", "vcxsrv"):
            fix(action)
        elif target == "devices" and action == "stop_live":
            threading.Thread(target=stop_live, daemon=True).start()
        elif target == "devices" and action == "connect":
            if BUSY["what"] or any(STEPS[k].alive() for k in STEPS):
                return self._send(409, json.dumps({"error": "stop everything first"}))
            threading.Thread(target=run_busy, args=("connecting",
                             lambda: connect_devices([d["busid"] for d in DEVICES["list"]])), daemon=True).start()
        elif target == "headset" and action == "view":
            return self._send(200, json.dumps({"message": show_headset_view()}))
        elif target == "fix" and action == "dexcap":
            threading.Thread(target=run_busy, args=("restarting", restart_dexcap), daemon=True).start()
        else:
            return self._send(404, "{}")
        self._send(200, "{}")

    def log_message(self, *args):
        pass


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Teleop Launcher</title>
<style>
:root { --bg:#f6f7f9; --card:#fff; --text:#1b1f24; --muted:#646d78; --border:#dde1e6; --soft:#eef1f4;
        --ok:#1f8a4c; --stale:#b7791f; --dead:#c53030; --accent:#2b6cb0; --on-accent:#fff; --log:#f0f2f5; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#121417; --card:#1c1f24; --text:#e6e9ed; --muted:#9aa3ad; --border:#2c3138; --soft:#262a30;
          --ok:#48bb78; --stale:#ecc94b; --dead:#fc8181; --accent:#4299e1; --on-accent:#0b0d10; --log:#15171b; }
}
* { box-sizing:border-box; }
body { margin:0; padding:24px 16px; background:var(--bg); color:var(--text); font:15px/1.45 system-ui,"Segoe UI",sans-serif; }
main { max-width:720px; margin:0 auto; }
h1 { font-size:22px; margin:0; }
header { display:flex; align-items:center; gap:20px; margin-bottom:16px; flex-wrap:wrap; }
.tabs { display:flex; gap:4px; }
.tabs button { border:0; background:transparent; color:var(--muted); font:600 15px system-ui,"Segoe UI",sans-serif;
               padding:6px 12px; border-radius:6px; cursor:pointer; }
.tabs button.on { background:var(--soft); color:var(--text); }
.soon { text-align:center; padding:40px 16px; }
.soon b { display:block; font-size:17px; margin-bottom:6px; }
.card { background:var(--card); border:1px solid var(--border); border-radius:12px; padding:16px; margin-bottom:16px; }
.q { font-weight:600; margin:0 0 6px; }
.field + .field { margin-top:14px; }
.seg { display:inline-flex; background:var(--soft); border-radius:8px; padding:3px; gap:3px; flex-wrap:wrap; }
.seg button { border:0; background:transparent; color:var(--text); padding:7px 14px; border-radius:6px; font:inherit; cursor:pointer; }
.seg button.on { background:var(--card); box-shadow:0 1px 2px rgba(0,0,0,.15); font-weight:600; }
.seg button:disabled { cursor:not-allowed; opacity:.6; }
.hint { color:var(--muted); font-size:13px; margin-top:4px; }
.hint.error { color:var(--dead); }
.folder { display:flex; gap:8px; }
.devgrid { display:grid; grid-template-columns:repeat(auto-fill,minmax(220px,1fr)); gap:12px; margin-top:10px; }
.devcard { border:1px solid var(--border); border-radius:10px; padding:10px; display:flex; flex-direction:column; gap:8px; }
.devcard .img { aspect-ratio:4/3; background:var(--soft); border-radius:6px; display:flex; align-items:center;
                justify-content:center; text-align:center; font-size:12px; color:var(--muted); overflow:hidden; padding:6px; }
.devcard img { width:100%; height:100%; object-fit:contain; }
.devcard { cursor:grab; background:var(--card); }
.devcard.dragging { opacity:.5; }
.devcard .img { cursor:pointer; position:relative; }
.devcard .img .badge { position:absolute; top:6px; left:6px; background:var(--dead); color:#fff; font-size:11px;
                       border-radius:4px; padding:1px 6px; }
.devcard .role { font-size:13px; font-weight:600; color:var(--accent); }
.rolechips { display:flex; flex-wrap:wrap; gap:8px; margin-top:6px; }
.rolechip { display:flex; align-items:center; gap:6px; border:1px solid var(--border); border-radius:8px; padding:6px 10px; font-size:13px; }
.rolechip b { font-weight:600; }
.rolechip.ok { border-color:var(--ok); } .rolechip.stale { border-color:var(--stale); }
.rolechip.off { color:var(--muted); }
.slots { display:grid; grid-template-columns:repeat(auto-fill,minmax(170px,1fr)); gap:10px; margin-top:12px; }
.slot { border:2px dashed var(--border); border-radius:10px; padding:10px; min-height:62px; }
.slot.filled { border-style:solid; border-color:var(--accent); }
.slot.over { background:var(--soft); border-color:var(--accent); }
.slot .slotname { font-weight:600; display:flex; justify-content:space-between; align-items:center; }
.slot .slotname button { border:0; background:transparent; color:var(--muted); font-size:16px; cursor:pointer; line-height:1; }
.slot .slotdev { font-size:13px; color:var(--muted); margin-top:4px; }
.folder input { flex:1; min-width:0; font:13px ui-monospace,Consolas,monospace; padding:6px 8px; border:1px solid var(--border);
                border-radius:6px; background:var(--card); color:var(--text); }
.go { display:flex; align-items:center; gap:12px; margin-top:18px; flex-wrap:wrap; }
.big { border:0; border-radius:8px; padding:10px 28px; font:600 16px system-ui,"Segoe UI",sans-serif; cursor:pointer;
       background:var(--accent); color:var(--on-accent); }
.big.stop { background:var(--dead); }
.big.outline { background:transparent; color:var(--accent); border:2px solid var(--accent); padding:8px 20px; }
.big:disabled { opacity:.5; cursor:wait; }
.banner { border-radius:8px; padding:10px 12px; margin-bottom:12px; }
.banner.ok { background:color-mix(in srgb, var(--ok) 15%, transparent); }
.banner.warn { background:color-mix(in srgb, var(--stale) 18%, transparent); }
.card.attention { border:2px solid var(--stale); background:color-mix(in srgb, var(--stale) 12%, var(--card)); }
.card.attention h2 { color:var(--text); }
.problem { display:flex; gap:8px; align-items:center; padding:6px 0; }
.problem button, .small { border:1px solid var(--border); background:var(--card); color:var(--text); border-radius:6px; padding:3px 10px; font:inherit; font-size:13px; cursor:pointer; }
.row { border-top:1px solid var(--border); padding:10px 0; }
.row:first-child { border-top:0; padding-top:0; }
.line { display:flex; align-items:center; gap:10px; }
.dot { width:12px; height:12px; border-radius:50%; flex:none; background:var(--border); }
.ok .dot { background:var(--ok); } .stale .dot { background:var(--stale); } .dead .dot { background:var(--dead); }
.name { font-weight:600; } .detail { color:var(--muted); font-size:13px; }
.line .small { margin-left:auto; }
details summary { cursor:pointer; color:var(--muted); font-size:13px; margin-top:4px; }
pre { background:var(--log); border-radius:6px; padding:8px; max-height:220px; overflow:auto; font-size:12px; margin:6px 0 0; white-space:pre-wrap; word-break:break-all; }
.cfg-group h2 { margin:18px 0 8px; } .cfg-group:first-child h2 { margin-top:0; }
.cfg { display:grid; grid-template-columns:minmax(150px, 34%) 1fr; gap:4px 12px; align-items:center; margin-bottom:8px; }
.cfg label { font-size:14px; }
.cfg input { font:13px ui-monospace,Consolas,monospace; padding:6px 8px; border:1px solid var(--border); border-radius:6px;
             background:var(--card); color:var(--text); min-width:0; width:100%; }
.cfg .port { font:13px ui-monospace,Consolas,monospace; color:var(--muted); padding:6px 0; }
.cfg-note { margin:-4px 0 10px; }
@media (max-width:560px) { .cfg { grid-template-columns:1fr; } }
.streams { display:grid; grid-template-columns:repeat(auto-fill,minmax(150px,1fr)); gap:8px; }
.s { background:var(--soft); border-radius:8px; padding:8px 10px; font-size:13px; }
.s .line { gap:6px; } .s .dot { width:9px; height:9px; }
h2 { font-size:15px; margin:0 0 10px; }
</style>
</head>
<body>
<main>
<header>
  <h1>Robot launcher</h1>
  <nav class="tabs"><button data-tab="teleop">Data collection</button><button data-tab="cameras">Cameras</button><button data-tab="policy">Evaluation</button><button data-tab="settings">Settings</button></nav>
</header>

<div id="message"></div>

<section id="tab-policy" hidden>
  <div class="card soon">
    <b>Coming soon</b>
    <span class="hint">Evaluating a trained policy on the robot will go here.</span>
  </div>
</section>

<section id="tab-cameras" hidden>
  <div class="card">
    <div class="line"><h2 style="margin:0">Cameras</h2>
      <button class="big stop" id="connect-devices" style="margin-left:auto">Connect devices</button></div>
    <div class="hint" id="devices-msg">Click a camera's picture to see it live, then drag it onto what it is used for.
      Choices are remembered by USB port, so keep each camera in the same port.</div>
    <div class="slots" id="cam-slots"></div>
    <div class="devgrid" id="cam-list"></div>
    <h2 style="margin-top:22px">Tactile sensors</h2>
    <div class="hint">On the xArm gripper fingers.</div>
    <div class="slots" id="tactile-slots"></div>
    <div class="devgrid" id="tactile-list"></div>
    <div class="go">
      <button class="big" id="save-devices">Save</button>
      <span class="hint" id="devices-save-msg"></span>
    </div>
  </div>
</section>

<section id="tab-settings" hidden>
  <div class="card">
    <div id="config-form"></div>
    <div class="go">
      <button class="big" id="save-config">Save</button>
      <button class="small" id="reset-config">Undo changes</button>
      <span class="hint" id="config-msg"></span>
    </div>
  </div>
</section>

<section id="tab-teleop">

<div class="card">
  <div class="field">
    <p class="q">Robot arm</p>
    <div class="seg" data-key="arm" id="arms"></div>
    <div class="hint" id="arm-hint"></div>
  </div>
  <div class="field" id="xarm-on-field">
    <p class="q">xArm controller runs on</p>
    <div class="seg" data-key="xarm_on" id="xarm-on"></div>
  </div>
  <div class="field">
    <p class="q">Robot hand</p>
    <div class="seg" data-key="hand" id="hands"></div>
  </div>
  <div class="field">
    <p class="q">Run</p>
    <div class="seg" data-key="robot" id="modes"></div>
    <div class="hint" id="mode-hint"></div>
  </div>
  <div class="field">
    <div class="line"><p class="q" style="margin:0">Recorded cameras and tactile sensors</p>
      <button class="small" id="change-cameras" style="margin-left:auto">Change</button></div>
    <div class="rolechips" id="devices-summary"></div>
  </div>
  <div class="field">
    <p class="q">Save recordings to</p>
    <div class="folder"><input id="data-dir" spellcheck="false"><button class="small" id="browse">Browse…</button></div>
    <div class="hint" id="folder-hint">Each session gets its own dated folder inside this one.</div>
  </div>
  <div class="go">
    <button class="big" id="go"></button>
    <button class="big outline" id="headset-view" title="Mirror what the headset shows in a window on this laptop">&#x1F97D; Show headset view</button>
    <span class="hint" id="plan"></span>
  </div>
  <div class="hint" id="headset-view-msg"></div>
</div>


<div class="card" id="parts-card">
  <h2>Status</h2>
  <div id="rows"></div>
</div>

<div class="card" id="live-card" hidden>
  <h2>Live data</h2>
  <div class="streams" id="streams"></div>
</div>

<details class="card"><summary>Launcher log</summary><pre id="launch"></pre></details>
</section>
</main>
<script>
function showTab(name) {
  // Live camera views only run while the Cameras tab is open; leaving it releases the cameras
  if (name !== "cameras" && liveOn && liveOn.size) stopAllLive();
  document.querySelectorAll(".tabs button").forEach(b => b.classList.toggle("on", b.dataset.tab === name));
  document.querySelectorAll("main > section").forEach(sec => sec.hidden = sec.id !== "tab-" + name);
  try { localStorage.setItem("launcher-tab", name); } catch (e) {}
}
document.querySelectorAll(".tabs button").forEach(b => b.addEventListener("click", () => showTab(b.dataset.tab)));
let savedTab = "teleop";
try { savedTab = localStorage.getItem("launcher-tab") || "teleop"; } catch (e) {}
showTab(["cameras", "policy", "settings"].includes(savedTab) ? savedTab : "teleop");

// Settings tab: built from the server's list; only rebuilt when not being edited
let configDirty = false;
function renderConfig(force) {
  if (!S || (!force && (configDirty || document.getElementById("config-form").contains(document.activeElement)))) return;
  const groups = {};
  S.config.forEach(f => (groups[f.group] = groups[f.group] || []).push(f));
  document.getElementById("config-form").innerHTML = Object.entries(groups).map(([g, fields]) =>
    `<div class="cfg-group"><h2>${esc(g)}</h2>` +
    (S.config_notes[g] ? `<p class="hint cfg-note">${esc(S.config_notes[g])}</p>` : "") +
    fields.map(f => `
      <div class="cfg"><label for="cfg-${f.key}">${esc(f.label)}</label>
        ${f.editable ? `<input id="cfg-${f.key}" data-key="${f.key}" value="${esc(f.value)}" spellcheck="false">`
                     : `<span class="port">${esc(f.value)}</span>`}</div>`).join("") +
    `</div>`).join("") +
    `<p class="hint">IPs are saved in dexcap/configs/xarm.yaml and franka.yaml, which the teleop server reads too. ` +
    `Ports are shown for reference: each is also fixed in the headset app, docker-compose.yml or the LEAP hand code, so they can't be changed here alone.</p>`;
  document.querySelectorAll("#config-form input").forEach(i => {
    i.disabled = S.running || !!S.busy;
    i.addEventListener("input", () => { configDirty = true; });
  });
  configDirty = false;
}
document.getElementById("save-config").addEventListener("click", async () => {
  const values = {};
  document.querySelectorAll("#config-form input").forEach(i => values[i.dataset.key] = i.value);
  const r = await fetch("/api/config", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(values)});
  const {errors} = await r.json();
  const msg = document.getElementById("config-msg");
  msg.className = "hint" + (errors.length ? " error" : "");
  msg.textContent = errors.length ? errors.join(" · ") : "Saved.";
  if (!errors.length) { configDirty = false; await tick(); renderConfig(true); }
});
document.getElementById("reset-config").addEventListener("click", () => { renderConfig(true); document.getElementById("config-msg").textContent = ""; });
const esc = s => String(s).replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const fmtAge = a => a == null ? "never" : a < 1 ? (a*1000).toFixed(0)+" ms" : a < 120 ? a.toFixed(1)+" s" : (a/60).toFixed(0)+" min";
let S = null;
async function post(path, body) {
  const r = await fetch("/api/" + path, {method: "POST", headers: {"Content-Type": "application/json"},
                                          body: body ? JSON.stringify(body) : undefined});
  setTimeout(tick, 200);
  return r;
}
document.querySelectorAll(".seg").forEach(seg => seg.addEventListener("click", e => {
  const b = e.target.closest("button"); if (!b || b.disabled) return;
  post("settings", {[seg.dataset.key]: b.dataset.val});
}));
// ---- Cameras tab: click a picture for its live view, drag cards onto roles, Save to keep the roles
let pending = null;      // busid -> role while editing; null = take what is saved
let deviceKey = "";      // what the cards were last built from
var liveOn = new Set();        // busids showing a live view (var: showTab checks it before this line runs)
const frozen = {};             // busid -> last frame (data URL) after its live view was stopped
let dragging = null;           // the card being dragged
function savedRoles() { return Object.fromEntries(S.devices.filter(d => d.role).map(d => [d.busid, d.role])); }
function rolesChanged() { return pending !== null && JSON.stringify(Object.entries(pending).sort()) !== JSON.stringify(Object.entries(savedRoles()).sort()); }
function deviceById(busid) { return S.devices.find(d => d.busid === busid); }
function roleLabel(role) { return role.startsWith("tactile_") ? "Tactile " + S.tactile_roles[role].toLowerCase() : S.camera_roles[role]; }
function cameraTabOpen() { return !document.getElementById("tab-cameras").hidden; }

function stopLive(busid) {
  const box = document.querySelector(`#tab-cameras .img[data-busid="${CSS.escape(busid)}"]`);
  const img = box && box.querySelector("img.live");
  if (img && img.naturalWidth) {  // keep the last frame so it's still clear which camera this is
    try { const c = document.createElement("canvas"); c.width = img.naturalWidth; c.height = img.naturalHeight;
          c.getContext("2d").drawImage(img, 0, 0); frozen[busid] = c.toDataURL("image/jpeg"); } catch (e) {}
  }
  if (img) img.removeAttribute("src");
  liveOn.delete(busid);
  if (box) box.innerHTML = pictureHtml(deviceById(busid));
}
function stopAllLive() {
  [...liveOn].forEach(stopLive);
  fetch("/api/devices/stop_live", {method: "POST"});
}
function pictureHtml(d) {
  if (!d) return "";
  if (d.video == null) return d.state === "Not shared" ? "Not connected yet: Connect devices asks Windows once to share it."
                       : d.state === "Attached" ? "Attached, but the teleop container doesn't see it yet: press Connect devices."
                       : "Not connected yet: press Connect devices.";
  if (S.running) return "In use by the teleop server.";
  if (liveOn.has(d.busid)) return `<img class="live" src="/api/live/${encodeURIComponent(d.busid)}?${Date.now()}" alt=""><span class="badge">LIVE</span>`;
  if (frozen[d.busid]) return `<img src="${frozen[d.busid]}" alt="">`;
  return "Click to show live view";
}
function buildCards(kind, listId, emptyText) {
  const cards = S.devices.filter(d => d.kind === kind).map(d => `
    <div class="devcard" draggable="true" data-busid="${esc(d.busid)}" data-kind="${kind}">
      <div class="img" data-busid="${esc(d.busid)}">${pictureHtml(d)}</div>
      <div><b>${esc(d.name)}</b><br><span class="hint">USB port ${esc(d.busid)}${d.video != null ? " · /dev/video" + d.video : ""}</span></div>
      <div class="role" data-role-for="${esc(d.busid)}"></div>
    </div>`);
  document.getElementById(listId).innerHTML = cards.join("") || `<div class="hint">${emptyText}</div>`;
}
function renderSlots(kind, slotsId, roles) {
  const el = document.getElementById(slotsId);
  const html = Object.entries(roles).map(([role, label]) => {
    const busid = Object.keys(pending).find(b => pending[b] === role);
    const d = busid && deviceById(busid);
    const text = d ? esc(d.name) : busid ? `USB port ${esc(busid)} (not plugged in)` : `Drop a ${kind === "tactile" ? "sensor" : "camera"} here`;
    return `<div class="slot ${busid ? "filled" : ""}" data-role="${role}" data-kind="${kind}">
      <div class="slotname">${esc(label)}${busid ? `<button data-clear="${role}" title="Clear">×</button>` : ""}</div>
      <div class="slotdev">${text}</div></div>`;
  }).join("");
  // Only redraw when something changed, and never mid-drag (the drop would land on a removed element)
  if (!dragging && el.dataset.html !== html) { el.innerHTML = html; el.dataset.html = html; }
}
function renderDevices() {
  if (!S) return;
  if (pending === null) pending = savedRoles();
  // The cards (and their live views) are only rebuilt when the devices themselves change
  const key = JSON.stringify(S.devices.map(d => [d.busid, d.state, d.video, d.kind])) + S.running;
  if (key !== deviceKey) {
    deviceKey = key;
    if (S.running) liveOn.clear();
    buildCards("camera", "cam-list", "No cameras plugged in.");
    buildCards("tactile", "tactile-list", "No tactile sensors plugged in.");
  }
  renderSlots("camera", "cam-slots", S.camera_roles);
  renderSlots("tactile", "tactile-slots", S.tactile_roles);
  document.querySelectorAll("[data-role-for]").forEach(el => {
    const role = pending[el.dataset.roleFor]; el.textContent = role ? roleLabel(role) : "Not used";
  });
  document.getElementById("save-devices").disabled = S.running || !!S.busy;
  // Red and prominent while anything plugged in isn't connected yet; quiet once everything is
  const connectBtn = document.getElementById("connect-devices");
  const notConnected = S.devices.filter(d => d.video == null);
  connectBtn.className = notConnected.length ? "big stop" : "small";
  connectBtn.textContent = S.busy === "connecting" ? "Connecting…" :
    notConnected.length ? `Connect devices (${notConnected.length} not connected)` : S.devices.length ? "All devices connected ✓" : "Connect devices";
  connectBtn.disabled = S.running || !!S.busy || (S.devices.length > 0 && !notConnected.length);
  const msg = document.getElementById("devices-save-msg");
  if (rolesChanged()) msg.textContent = "Unsaved changes";
  else if (msg.textContent === "Unsaved changes") msg.textContent = "";
  if (S.busy === "connecting") document.getElementById("devices-msg").textContent = S.busy_step || "Connecting…";
  else if (connectAsked && !S.busy) {
    connectAsked = false;
    document.getElementById("devices-msg").textContent = "Done. Click a picture to see it live, then drag it onto what it is used for.";
  }
}
["cam-list", "tactile-list"].forEach(id => {
  const list = document.getElementById(id);
  list.addEventListener("click", e => {
    const box = e.target.closest(".img[data-busid]"); if (!box) return;
    const d = deviceById(box.dataset.busid);
    if (!d || d.video == null || S.running || S.busy) return;
    if (liveOn.has(d.busid)) stopLive(d.busid);
    else { liveOn.add(d.busid); box.innerHTML = pictureHtml(d); }
  });
  list.addEventListener("dragstart", e => {
    const card = e.target.closest(".devcard"); if (!card) return;
    dragging = card; card.classList.add("dragging");
    e.dataTransfer.setData("text/plain", card.dataset.busid);
  });
  list.addEventListener("dragend", () => { if (dragging) dragging.classList.remove("dragging"); dragging = null; });
});
["cam-slots", "tactile-slots"].forEach(id => {
  const slots = document.getElementById(id);
  slots.addEventListener("dragover", e => {
    const slot = e.target.closest(".slot");
    if (slot && dragging && dragging.dataset.kind === slot.dataset.kind && !S.running) { e.preventDefault(); slot.classList.add("over"); }
  });
  slots.addEventListener("dragleave", e => { const slot = e.target.closest(".slot"); if (slot) slot.classList.remove("over"); });
  slots.addEventListener("drop", e => {
    const slot = e.target.closest(".slot"); if (!slot || !dragging) return;
    e.preventDefault();
    const busid = dragging.dataset.busid, role = slot.dataset.role;
    for (const b of Object.keys(pending)) if (pending[b] === role || b === busid) delete pending[b];  // one per role
    pending[busid] = role;
    renderDevices();
  });
  slots.addEventListener("click", e => {
    const role = e.target.dataset && e.target.dataset.clear; if (!role) return;
    for (const b of Object.keys(pending)) if (pending[b] === role) delete pending[b];
    renderDevices();
  });
});
document.getElementById("save-devices").addEventListener("click", async () => {
  const msg = document.getElementById("devices-save-msg");
  const changed = rolesChanged();
  if (changed) {
    const r = await post("settings", {device_roles: pending});
    if (!r.ok) { msg.textContent = (await r.json()).error || "Could not save"; return; }
  }
  stopAllLive();
  await tick();
  pending = savedRoles();
  msg.textContent = (changed ? "Saved." : "No changes to save.") + " Live views stopped.";
  renderDevices();
});
let connectAsked = false;
document.getElementById("change-cameras").addEventListener("click", () => showTab("cameras"));
document.getElementById("connect-devices").addEventListener("click", async () => {
  connectAsked = true;
  const r = await post("devices/connect");
  if (!r.ok) { connectAsked = false; document.getElementById("devices-msg").textContent = "Stop everything first: the teleop is using the cameras."; }
});
document.getElementById("go").addEventListener("click", () => post(S.running ? "all/stop" : "all/start"));
document.getElementById("headset-view").addEventListener("click", async () => {
  const msg = document.getElementById("headset-view-msg");
  msg.textContent = "Connecting to the headset…";
  const r = await post("headset/view");
  msg.textContent = (await r.json()).message;
  setTimeout(() => { msg.textContent = ""; }, 10000);
});
const dirInput = document.getElementById("data-dir"), folderHint = document.getElementById("folder-hint");
async function setDir(path) {
  const r = await post("settings", {data_dir: path});
  const err = r.ok ? null : (await r.json()).error;
  folderHint.textContent = err || "Each session gets its own dated folder inside this one.";
  folderHint.className = "hint" + (err ? " error" : "");
  if (err) dirInput.value = S.settings.data_dir;
}
dirInput.addEventListener("change", () => setDir(dirInput.value.trim()));
document.getElementById("browse").addEventListener("click", async () => {
  const r = await post("browse");
  if (r.ok) { const {path} = await r.json(); if (path) setDir(path); }
});
const openLogs = new Set();
function keepScroll(el, text) {
  const atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 20;
  el.textContent = text;
  if (atBottom) el.scrollTop = el.scrollHeight;
}
async function tick() {
  try { S = await (await fetch("/api/state", {cache: "no-store"})).json(); }
  catch (e) { document.getElementById("message").innerHTML = '<div class="banner warn">The launcher is not responding. Is its window still open?</div>'; return; }
  const locked = S.running || !!S.busy;
  const fill = (el, opts) => { const html = Object.entries(opts).map(([k, v]) => `<button data-val="${k}">${esc(v)}</button>`).join("");
                                if (el.dataset.html !== html) { el.innerHTML = html; el.dataset.html = html; } };
  fill(document.getElementById("arms"), S.arms);
  fill(document.getElementById("xarm-on"), S.xarm_locations);
  document.getElementById("xarm-on-field").hidden = S.settings.arm !== "xarm" || !["arm_hand", "arm_only"].includes(S.settings.robot);
  renderDevices();
  // One chip per role: green = recorded and connected, amber = chosen but not connected yet, grey = not used
  const chips = [...Object.keys(S.camera_roles), ...Object.keys(S.tactile_roles)].map(role => {
    const d = S.devices.find(x => x.role === role);
    const missing = !d && Object.values(S.missing_roles).includes(role);
    const cls = d ? (d.video != null ? "ok" : "stale") : missing ? "stale" : "off";
    const what = d ? esc(d.name) + (d.video != null ? "" : ", connects at Start") : missing ? "not plugged in" : "not used";
    return `<span class="rolechip ${cls}"><span class="dot"></span><b>${esc(roleLabel(role))}:</b> ${what}</span>`;
  }).join("");
  const summary = document.getElementById("devices-summary");
  if (summary.dataset.html !== chips) { summary.innerHTML = chips; summary.dataset.html = chips; }
  fill(document.getElementById("hands"), S.hands);
  document.getElementById("arm-hint").textContent = S.settings.arm === "franka" ?
    "The Franka controller and its LEAP hand run on the control computer and are started there by hand; this page shows what to run and waits for them." : "";
  fill(document.getElementById("modes"), S.robot_modes);
  document.querySelectorAll(".seg[data-key]").forEach(seg => {
    const val = String(S.settings[seg.dataset.key]);
    seg.querySelectorAll("button").forEach(b => { b.classList.toggle("on", b.dataset.val === val); b.disabled = locked; });
  });
  // Hand only needs the LEAP hand: the xArm gripper is driven through the arm
  modes.querySelector('[data-val="hand_only"]').disabled = locked || S.settings.hand !== "leap";
  document.getElementById("mode-hint").textContent =
    S.settings.robot === "arm_only" ? "No gloves: only the arm follows the Quest; the " + S.hands[S.settings.hand] + " stays in a fixed pose" + (S.settings.hand === "leap" ? " and is not started." : ".") :
    S.settings.robot === "hand_only" ? "The arm stays off. The hand still needs the Quest (fingers are placed relative to the wrist) and moves while recording." :
    S.settings.robot === "sim" ? "Nothing real moves; the robot only moves in the simulation window and the headset." :
    S.settings.hand !== "leap" ? "Hand only is available with the LEAP hand." : "";
  if (S.settings.arm === "franka") document.getElementById("mode-hint").textContent += (document.getElementById("mode-hint").textContent ? " " : "") +
    "The Franka hand isn't supported by the teleop yet.";
  if (document.activeElement !== dirInput) dirInput.value = S.settings.data_dir;
  dirInput.disabled = document.getElementById("browse").disabled = locked;
  const go = document.getElementById("go");
  go.textContent = S.busy === "starting" ? "Starting…" : S.busy === "stopping" ? "Stopping…" : S.busy === "restarting" ? "Restarting…" :
                   S.busy === "connecting" ? "Connecting…" : S.running ? "Stop" : "Start";
  go.className = "big" + (S.running && !S.busy ? " stop" : "");
  go.disabled = !!S.busy;
  document.getElementById("plan").textContent = locked ? (S.running ? "Stop to change the setup." : "") :
    "Starts: " + S.rows.map(r => r.label).join(" → ");

  let msg = "";
  if (S.leftovers.length) msg += `<div class="banner warn">Still running from another setup: ${esc(S.leftovers.join(", "))}. Press Stop to stop it.</div>`;
  if (S.problems.length) msg += `<div class="card attention"><h2>⚠ ${S.running ? "Needs attention" : "Check before starting"}</h2>` + S.problems.map(p =>
    `<div class="problem">⚠ ${esc(p.text)} ${p.fix ? `<button onclick="post('fix/${p.fix}')">${esc(p.fix_label)}</button>` : ""}</div>`).join("") + `</div>`;
  const info = (S.teleop && S.teleop.info) || {};
  if (S.checks_age > 30) msg += `<div class="banner warn">The launcher's background checks stopped ${Math.round(S.checks_age)} s ago, so this page may be out of date. Close the launcher window and open it again (the robot keeps running).</div>`;
  if (S.busy) msg += `<div class="banner warn">${esc(S.busy_step || "Working…")}</div>`;
  else if (info.recording === "yes") msg += `<div class="banner ok"><b>Recording.</b> Press A in the headset to stop.</div>`;
  else if (S.ready) msg += `<div class="banner ok"><b>Next:</b> ${esc(info.quest || "Start the ARCap app in the headset.")}</div>`;
  else if (S.running && info.stage && info.stage !== "running") msg += `<div class="banner warn">Teleop server: ${esc(info.stage)}…</div>`;
  document.getElementById("message").innerHTML = msg;

  const rows = document.getElementById("rows");
  rows.querySelectorAll("details").forEach(d => d.open ? openLogs.add(d.dataset.key) : openLogs.delete(d.dataset.key));
  const scrolls = {}; rows.querySelectorAll("pre").forEach(p => scrolls[p.dataset.key] = p.scrollTop);
  rows.innerHTML = S.rows.map(r => `
    <div class="row ${r.health}">
      <div class="line"><span class="dot"></span><span class="name">${esc(r.label)}</span>
        <span class="detail">${esc(r.detail)}</span>
        ${S.running && !S.busy && !r.manual ? `<button class="small" onclick="post('${r.key}/restart')">Restart</button>` : ""}</div>
      <details data-key="${r.key}" ${openLogs.has(r.key) ? "open" : ""}><summary>${esc(r.where)} · log</summary><pre data-key="${r.key}"></pre></details>
    </div>`).join("");
  S.rows.forEach(r => { const p = rows.querySelector(`pre[data-key="${r.key}"]`); p.textContent = r.log.join("\\n");
    p.scrollTop = r.key in scrolls ? scrolls[r.key] : p.scrollHeight; });

  const t = S.teleop, live = document.getElementById("live-card");
  live.hidden = !t;
  if (t) document.getElementById("streams").innerHTML = Object.values(t.streams).map(x => `
    <div class="s ${x.state}"><div class="line"><span class="dot"></span><b>${esc(x.label)}</b></div>
      <div>${x.rate.toFixed(0)} Hz · ${fmtAge(x.age)}</div>
      <div class="detail">${Object.entries(x.detail).filter(([k]) => k !== "joints").map(([k, v]) => esc(k) + ": " + esc(v)).join("<br>")}</div></div>`).join("") +
    `<div class="s"><b>Recording</b><div>${esc(t.info.recording ?? "-")} · ${esc(t.info.trajectories ?? 0)} saved</div></div>`;
  keepScroll(document.getElementById("launch"), S.launch_log.join("\\n"));
  renderConfig(false);
  document.getElementById("save-config").disabled = S.running || !!S.busy;
  if (S.running || S.busy) document.getElementById("config-msg").textContent = "Stop everything to change settings.";
}
tick();
setInterval(tick, 1000);
</script>
</body>
</html>
"""


def main():
    load_launcher_config()
    load_settings()
    STEPS["xarm"].location_changed()
    threading.Thread(target=run_checks, daemon=True).start()
    # Bound to localhost only: this page can move the robot, so it is not exposed on the lab network.
    class QuietServer(ThreadingHTTPServer):
        daemon_threads = True

        def handle_error(self, request, client_address):
            # Windows reports a browser hanging up as WinError 10053/10054; that's normal, not an error
            if not isinstance(sys.exc_info()[1], (ConnectionAbortedError, ConnectionResetError, BrokenPipeError)):
                super().handle_error(request, client_address)

    server = QuietServer(("127.0.0.1", CONFIG["port"]), Handler)
    url = f"http://localhost:{CONFIG['port']}"
    print(f"Teleop launcher running on {url}  (Ctrl+C to quit; programs keep running)")
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
