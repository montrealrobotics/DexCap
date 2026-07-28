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

### On the NUC:

#### Xarm
If using an xarm also run the following:
```
mkdir -p ~/hw_ctrl_ws/src
cd ~/hw_ctrl_ws/src
git clone <https://github.com/montrealrobotics/deoxys_control.git> -b xarm
git clone https://github.com/montrealrobotics/LEAP_Hand_API.git
pip3 install zmq xarm-python-sdk pyquaternion numpy pyRobotiqGripper

```

Build the leap hand api ROS2 docker image following the installation instructions in the [leap api](https://github.com/montrealrobotics/LEAP_Hand_API) repo.

#### Franka arm

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

#### Docker install
Alternatively you can do a docker installation (recommended)

...

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

Check that all of the ports you wish to use are included in docker_compose.yaml and that udp ports are specified as such.
To stop windows from blocking these ports in the container, before starting the container, run

```
net stop winnat
```

Start the container and then...

```
net start winnat
```

Build the Docker image. (this should make the .Xauthority file available in the docker container at ~/root/)
```
docker compose build dexcap
```

When bringing up the docker container, the DISPLAY environment variable should be set to the same value as the host (172.16.0.2:0.0).
```
docker compose up dexcap
```

## Teleop

Start the franka arm:

1. Turn on the power
2. Switch the internet connection
3. Unlock the robot
4. Release the user-stop
5. Activate FCI

Connect the power cable to the Leap hand.

If using the xarm:

1. Turn on the power
2. Connect a ethernet cable to a switch that is also connected to the NUC and windows machine.

Power on the Quest 3 headset

Connect the rokoko gloves to their power banks, start up rokoko studio and start data streaming.

The windows laptop, rokoko gloves and Quest3 headset should all be connected to the same wifi network.

### On the NUC

Start a docker container for the leap hand following the readme in the leap repo. Use the ros2 version.

In the docker container launch the following to start listening to hand commands:
```
ros2 launch leap_hand launch_leap_redis.py
```

For Franka:
Under `deoxys_control/deoxys/`, run the following command to start the real-time control of the arm:

``` shell
./auto_scripts/auto_arm.sh config/franka.yml
```

For the Xarm:

Launch the xarm with:
```
cd ~/hw_ctrl_ws/src/deoxys_control

python deoxys/deoxys/robot_interfaces/xarm_interface/xarm_client.py 192.168.42.222 -g
```

### On the Laptop
In the docker container, start the teleop server:
```
cd ~/dexcap_ws/src/DexCap/scripts
uv run teleop_server.py
```

Start the ARCap app in the Quest 3 headset, enter the IP of the windows laptop (wifi interface)

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
