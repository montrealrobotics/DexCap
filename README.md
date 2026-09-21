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
