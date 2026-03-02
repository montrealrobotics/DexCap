# ARCap: Collecting High-quality Human Demonstrations for robot Learning with Augmented Reality Feedback (Part II)

-------
## Table of Contents
- [Overview](#overview)
- [Installation](#installation)
- [Data Processing](#data-processing)
- [Building Training Dataset](#building-training-dataset)
- [Training Policy](#training-policy)
- [Testing on Robot](#testing-on-robot)
- [Acknowledgements](#acknowledgements)
- [BibTeX](#bibtex)
- [License](#license)

-------
## Overview

This repository is the policy training and testing code for "ARCap: Collecting High-quality Human Demonstrations for Robot Learning with Augmented Reality Feedback" ([Paper](https://arxiv.org/abs/2403.07788), 
[Website](https://stanford-tml.github.io/ARCap)) by Chen et al. at [The Movement Lab](https://tml.stanford.edu/) 
and [Stanford Vision and Learning Lab](http://svl.stanford.edu/).

In this repo, we provide our full implementation code for [Data processing](#data-processing), 
[Building dataset](#building-training-dataset), [Training policy](#training-policy) and testing on hardware [Testing on Robot](#testing-on-robot).

-------
## Installation

The installation is split in two parts, an installation on a Windows laptop and one on a control machine, the NUC.

### On the NUC:

#### Xarm
If using an xarm also run the following:
```
mkdir -p ~/xarm_ws/src
git clone https://github.com/montrealrobotics/LEAP_Hand_API.git
git clone https://github.com/montrealrobotics/xarm_ros2.git -b humble_no_gazebo --recursive
cd ~/xarm_ws/src/
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/xarm_ws/
colcon build --symlink-install
```

Build the leap hand api ROS2 docker image following the installation instructions in the [leap api](https://github.com/montrealrobotics/LEAP_Hand_API) repo.

#### Franka arm

```
mkdir -p ~/dexcap_ws/src
git clone git@github.com:montrealrobotics/deoxys_control.git
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
./auto_scripts/auto_arm.sh config/charmander.yml
```

For the Xarm:

Launch the xarm with:
```
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=<ARM_IP> report_type:=normal
```

Launch node that converts messages to ros service calls for the Xarm:
```
ros2 launch xarm_api zmq_to_ros.launch.py
```

### On the Laptop
In the docker container, start the teleop server:
```
cd ~/dexcap_ws/src/DexCap/STEP3_inference
uv run teleop_server.py
```

Start the ARCap app in the Quest 3 headset, enter the IP of the windows laptop (wifi interface)

-------
## Data Collection
Please refers to [ARCap_part1](https://github.com/Stanford-TML/ARCap_part1/tree/release)

-------
## Building Training Dataset
After collecting and processing the raw data, we can now transfer the data to the workstation and use the following script to generate a `hdf5` dataset file in [robomimic](https://github.com/ARISE-Initiative/robomimic) format for training.
```	
python demo_create_hdf5_multi_arcap.py
```

You can download our processed dataset from [Link](https://huggingface.co/datasets/Ericcsr/ARCap).

-------
## Training Policy
After building the `hdf5` dataset, we can start policy training with the following script and config file:
```
cd DexCap/STEP2_train_policy/robomimic
python scripts/train.py --config training_config/[NAME_OF_CONFIG].json
```
After traing, model checkpoints will be stored in `STEP2_train_policy/trained_models`. The default training config will train a point cloud-based Diffusion Policy, which takes the point cloud observation from the chest camera (transformed to the fixed world frame) as input and generates a sequence (20 steps) of actions for both robot hands and arms (46 dimensions in total). For more details on the algorithm, please check out our study paper.

-------
## Testing on Robot
Before testing on robot, we need to know camera's extrinsic parameter relative to robot base. ARCap provide an easy and intuitive way for hand eye calibtation. First, we need to connect the depth camera to Ubuntu workstation and run:
```
cd STEP3_inference
python calibrate_camera.py
```
Then, put on the headset and start ARCap application, select robot and align the virtual robot base with actual robot base; click `button A` after tuning robot base pose to enable deploy mode. Then put the headset to a support and tuning headset pose so that the manipulation scene is visible. Press `button A` and `button X` to confirm headset pose and finish hand eye calibration.

## Troubleshooting

The ARCap app in the Quest headset should be started after the teleop script has initialised and prompts you to launch the app.

Once you start the app in the Quest headset, data should be streaming to the windows laptop, you can check this is the case by running, the following,
the port is configured in ip_config.py:
```
ncat -u -l <POSE_CMD_PORT>
```

To test trained policy, first install...
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
