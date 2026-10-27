# ARCap: Collecting High-quality Human Demonstrations for robot Learning with Augmented Reality Feedback (Part II)

-------
## Table of Contents
- [Installation](#installation)
- [Teleop](#teleop)
- [Acknowledgements](#acknowledgements)
- [BibTeX](#bibtex)
- [License](#license)

-------

## Installation

The installation is split in two parts, an installation on a Windows laptop and one on a control machine, the NUC.

### **On the Linux machine (Jetson Orin or NUC)**

#### Xarm
If using an xarm, run the following:
```
mkdir -p ~/hw_ctrl_ws/src
cd ~/hw_ctrl_ws/src
git clone <https://github.com/montrealrobotics/deoxys_control.git> -b xarm
git clone https://github.com/montrealrobotics/LEAP_Hand_API.git
pip3 install zmq xarm-python-sdk pyquaternion numpy pyRobotiqGripper

```

Build the leap hand api ROS2 docker image following the installation instructions in the [leap api](https://github.com/montrealrobotics/LEAP_Hand_API) repo.

<details>
    <summary>If using the Franka arm</summary>

```
mkdir -p ~/dexcap_ws/src
git clone git@github.com:montrealrobotics/deoxys_control.git -b xarm
git clone git@github.com:montrealrobotics/LEAP_Hand_API.git
```

Build the controller code (if using Franka arm); Run this command in directory `deoxys_control/deoxys/` on the NUC.

``` shell
cd ~/dexcap_ws/src/deoxys_control/deoxys
make -j build_franka=1
```

Build the leap hand api ROS2 docker image following the installation instructions in the [leap api](https://github.com/montrealrobotics/LEAP_Hand_API) repo.
</details>
<br>

### **Docker install On the Linux machine**

Alternatively you can do a docker installation (recommended)

Build the docker image:

For xarm:

```
cd ~/hw_ctrl_ws/src/deoxys_control
docker compose build deoxys_xarm
```

Bring up the container:

```
docker compose up deoxys_xarm -d
```

<details>
  <summary>For franka:</summary>


```
cd ~/hw_ctrl_ws/src/deoxys_control
docker compose build deoxys_franka
```

Bring up the container:

```
docker compose up deoxys_franka -d
```
</details>

### On the Windows laptop:

Configure the ethernet interface connected to the NUC/Arm control computer to have an IP in the same subnet, if the NUC is 172.16.0.3, set the ethernet interface IP to 172.16.0.2.

Clone this repo and deoxys control:

```
mkdir -p ~/dexcap_ws/src
git clone git@github.com:montrealrobotics/DexCap.git -b arcap_policy
git clone git@github.com:montrealrobotics/deoxys_control.git
```

To view the GUI while running the code in the docker container, install vcxsrv following [this guide](https://vcxsrv.com/).

After installation, run Xlaunch, select 'Multiple Windows' and display number '0'.
> Next > Start no Client > Next > Ensure that 'Disable access control' is selected > Next > Finish.

The Xserver should now be running.
You will need to generate an Xauth key and copy it to the docker container.

- Open C:\Program Files\VcXrv\x0.hosts
- Add your IP address (172.16.0.2)
- Navigate to C:\Program Files\VcXrv\
- Run .\xauth.exe add [YOURIP]:0 . 00000000000000000000000000000000
- Navigate to C:\users\<YOURUSERNAME>
- Copy .Xauthority file to ~/dexcap_ws/src/DexCap/

In a powershell window, set the DISPLAY variable to the IP of the ethernet interface connected to the NUC:
```
set-variable -name DISPLAY -value 172.16.0.2:0.0
```

**Build the docker image**

There can be an issue with port conflicts, we found that the POSE_CMD_PORT, 12346, can be grabbed by other services, this can be prevented by adding it to the excluded port range.

Open a powershell window with administrator privileges and run:

```
Netsh interface ipv4 add excludedportrange protocol=udp startport=12346 numberofports=1 store=persistent
```

Build the docker image:

```
cd ~/dexcap_ws/src/DexCap

docker-compose build
```

Start the docker container:

```
docker-compose up -d
```


## Teleop

<details>
    <summary>If using the Franka arm</summary>

    Start the franka arm:

    1. Turn on the power
    2. Switch the internet connection
    3. Unlock the robot
    4. Release the user-stop
    5. Activate FCI


</details>
<br>

If using the leap hand, connect the power cable to the Leap hand.

If using the xarm:

1. Turn on the power
2. Connect a ethernet cable to a switch that is also connected to the NUC and windows machine.

Power on the Quest 3 headset

Connect the rokoko gloves to their power banks, start up rokoko studio and start data streaming.

The windows laptop, rokoko gloves and Quest3 headset should all be connected to the same wifi network.

### Quick start: the launcher (xArm + LEAP hand)

On the Windows laptop, double-click `launcher\launch.bat` (or run `python launcher\launcher.py`). It opens
http://localhost:8090, which:

- checks Docker Desktop, VcXsrv, the control computer, the arm control box and the Quest (and can start Docker Desktop and VcXsrv);
- with **Start all**, starts in order and waits for each to come up:
  0. Rokoko: opens Rokoko Studio if needed, waits for the gloves and turns on the livestream through the Studio
     Command API (Settings > Command API, default port 14053 and key 1234; a scene with the actor and the
     custom JSONv3 stream to this laptop must already be set up in Studio),
  1. the LEAP hand on the control computer (the Jetson; `leap-hand-ros2` container,
     `ros2 launch leap_hand launch_leap_redis.py`),
  2. the xArm client (`xarm_client.py`) in `deoxys_xarm_cont`, on this laptop or on the control computer
     (choose "xArm controller runs on"; the teleop is told where with `--arm_host`),
  3. the teleop server in the `dexcap` container (`scripts/teleop_server.py`);
- shows each program's log, with Start / Stop / Restart buttons, and the teleop data streams.

Then open the ARCap app in the headset.

**Cameras and tactile sensors.** On the Cameras tab, click a device's picture to see it live and drag it onto
its role (wrist / left / right camera, left / right tactile sensor), then Save. Roles are remembered by USB port,
so keep each device in the same port. The laptop's built-in webcam is not offered. Docker Desktop only sees USB
devices attached to WSL: the launcher binds each device once (Windows asks for permission) and attaches the
chosen ones at Start (with the Ubuntu WSL distribution running, since Docker's own one can't attach), then passes
their `/dev/video` numbers to the teleop (`--wrist_camera`, `--left_camera`, `--right_camera`, `--tactile_left`,
`--tactile_right`). Tactile images are saved as `robot0_tactile_left` / `robot0_tactile_right`.

Each recording's `traj.hdf5` holds 224x224 images for training. Like DROID, every recording folder also gets a
full-resolution video per camera, named by role (`wrist.mp4`, `left.mp4`, `right.mp4`, `tactile_left.mp4`, ...),
with `<name>_timestamps.csv` giving each frame's capture time. A stereo camera's video keeps the whole
side-by-side pair; tactile sensors are saved at half resolution. The videos are written by each camera's own
reader thread, so they don't slow the teleop loop. (ZED SVO files would need the ZED SDK in the container.)

Choosing **Franka** as the robot arm keeps the old Franka setup: the arm controller (`auto_arm.sh`) and its LEAP
hand run on the NUC (`CTRL_HOST.IP_ETH` in `franka.yaml`) and are started there by hand. The launcher shows the
commands, waits for the NUC, the Franka and the LEAP hand's redis to answer, and then starts the teleop with
`--robot_arm franka`. The laptop's Ethernet must be on the Franka network (172.16.0.2). Addresses, the teleop arguments and
container names are in `CONFIG` at the top of `launcher/launcher.py`. The launcher reaches the control computer (Jetson) with
`ssh real@192.168.42.10`, so the laptop needs an SSH key on the Jetson (`ssh-keygen`, then add the public key to
`~/.ssh/authorized_keys` on the Jetson). The page only listens on localhost, since it can move the robot.

While the teleop server runs it also serves a status page on port 8080 (shown inside the launcher): one card per
data stream (Quest pose in, IK results out, Rokoko gloves, xArm state, LEAP hand redis, robot commands) with its
rate and time since the last packet, so a dropped stream shows up as a red card rather than a silent hang. Add
`- "8080:8080"` to the `dexcap` ports in `docker-compose.yml` to open it directly at http://localhost:8080 (this
needs the container to be recreated).

**Quest / Rokoko ports.** Docker Desktop stops forwarding a published UDP port into the container once the
program listening on it inside the container exits, which is why the Quest stream used to stop after the first
teleop run until the container was restarted. The launcher therefore starts `scripts/udp_relay.py` in the
`dexcap` container: it keeps ports 12346 (Quest) and 14043 (Rokoko) open permanently and passes the packets to
ports 22346 and 24043, where the teleop listens. When the relay is running and you start the teleop by hand, add
`--pose_port 22346 --rokoko_port 24043`; to run without the relay, restart the container first.

The sections below describe the same steps by hand.

### On the Linux machine

Start a docker container for the leap hand following the readme in the leap repo. Use the ros2 version.

In the docker container launch the following to start listening to hand commands:
```
ros2 launch leap_hand launch_leap_redis.py
```

<details>
    <summary>If using Franka arm:</summary>
    Under `deoxys_control/deoxys/`, run the following command to start the real-time control of the arm:

    ``` shell
    ./auto_scripts/auto_arm.sh config/franka.yml
    ```
</details>
<br>


If using the Xarm:

Launch the xarm with the following (the IP should be the IP of the arm control box):
```
cd ~/hw_ctrl_ws/src/deoxys_control

python deoxys/deoxys/robot_interfaces/xarm_interface/xarm_client.py 192.168.42.222 -g
```

#### Docker version

Open a terminal in the docker container and launch the client:

```
docker exec -it deoxys_xarm_cont bash

cd deoxys_control

python deoxys/deoxys/robot_interfaces/xarm_interface/xarm_client.py 192.168.42.222 -g
```

To select the type of gripper, use arg ‘-t’ with either robotiq or xarm

```
python deoxys/deoxys/robot_interfaces/xarm_interface/xarm_client.py 192.168.42.222 -g -t robotiq
```

### On the Windows laptop:

**Start Rokoko gloves**

&emsp; With the Rokoko right glove powered, open Rokoko Studio. Connect to the glove and enable data streaming, in the right panel, under 'Streaming', the bottom entry with a cube icon is for custom streaming, open the settings for this option and set forward IP to your computer's IP, 
&emsp; set data format as `JSONv3`. Click 'Activate'

**Start the teleop script**

&emsp; Open a terminal in the container:

```
docker exec -it dexcap bash

source /home/.venv/bin/activate

cd /workspace/DexCap

uv run --active scripts/teleop_server.py --real_robot True --use_gloves True --robot_arm xarm
```

This should open pybullet and end with a prompt to open the app in the Quest headset.

**Start the app in the VR headset**

&emsp; With the headset powered on, open the arcap app, you will be prompted for the IP of the windows laptop **wifi** IP.

&emsp; Then select ‘xarm’ and gripper type and ‘Connect’.

&emsp; You should see a visual of the simulated robot, move it to a convenient position.

&emsp; Pressing ‘A’ will start recording, this will start commands streaming to the robot.


## Troubleshooting

The ARCap app in the Quest headset should be started after the teleop script has initialised and prompts you to launch the app.

Once you start the app in the Quest headset, data should be streaming to the windows laptop, you can check this is the case by running, the following,
the port is configured in ip_config.py:
```
ncat -u -l <POSE_CMD_PORT>
```

If the teleop script reports that no data is received from Rokoko, check in Rokoko studio that the streaming is configured to broadcast to the correct IP, the IP of the laptop.

-------
## Acknowledgements
- Our policy training is implemented based on [robomimic](https://github.com/ARISE-Initiative/robomimic), [Diffusion Policy](https://github.com/real-stanford/diffusion_policy).
- The robot arm controller is based on [Deoxys](https://github.com/UT-Austin-RPL/deoxys_control).
- The robot LEAP hand controller is based on [LEAP_Hand_API](https://github.com/leap-hand/LEAP_Hand_API).

-------
## BibTeX
```
@article{}
```

-------
## License
Licensed under the [MIT License](LICENSE)
