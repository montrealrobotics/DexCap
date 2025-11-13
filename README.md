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

```
mkdir -p ~/dexcap_ws/src
git clone git@github.com:montrealrobotics/DexCap.git
git clone git@github.com:montrealrobotics/deoxys_control.git
git clone git@github.com:montrealrobotics/LEAP_Hand_API.git
```

The first step is to install and build environments for the Ubuntu workstation, which could also be a headless server for dataset building, training and testing. Simply follow:
```	
cd ~/dexcap_ws/src/DexCap
conda env create -f environment.yml
conda activate dexcap
```
Then install the deoxys controller

```
cd ~/dexcap_ws/src/deoxys_control/deoxys
./InstallPackage
make -j build_deoxys=1
pip install -e .
cd protobuf/python
pip install -e .
```

## Teleop

Start a docker container for the leap hand following the readme in the leap repo. Use the ros2 version.
In the docker container launch the following to start listening to hand commands:
```
ros2 launch leap_hand launch_leap.py
```

To start the teleop server:
```
cd ~/dexcap_ws/src/DexCap/STEP3_inference
python teleop_server.py
```

Running the robot control code on the NUC:
COMING SOON

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