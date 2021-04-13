# PoseDSAC
This project repository contains code for PoseDSAC, an integrated system for 
pose estimation from RGB images.

## Table of Contents
- [Background](#background)
- [Dependencies](#dependencies)
- [Install](#install)
- [Usage](#usage)
- [Authors](#authors)

## Background
RGB cameras are cheap and ubiquitous sensors that can be found in almost every mobile device. In view of this, recent work in SLAM research has focused on performing pose estimation and localization using only RGB images from cameras. These efforts have been fairly successful at extracting features from images which could be used by a robot to track its displacement and its pose in a room. The feature extractors used in such efforts are often hand-designed and do not work
as well in environments where the sought-after features are scarce. 

The recent advancement in Deep Learning research has led to the development of effective function approximators that could be trained with the right data to approximate almost any function. In view of this, recent work in Visual SLAM has focused on ways to train neural networks to be effective feature extractors and pose estimators. In this work, we train a neural network to predict the pose of a robot given a sequence of images taken as the robot moves around a large area. We then jointly optimize the posterior joint probability of the predicted pose from the neural network and the robot's noisy odometry using a Pose SLAM formulation to get a better estimate of the robot's pose. We perform this optimization iteratively using GTSAM's implementation of iSAM2, an iterative smoothing and mapping system.

This repository contains our implementation of this pose estimation system. Our core design philosophy in this implementation is to strive for speed of inference and estimation. We use ROS melodic as the main communication protocol between modules. 

## Dependencies
Although this project was built and tested using [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu), most of the other versions of ROS could potentially support this project.

This project also uses [Pytorch 1.7](https://pytorch.org/) for neural network training and inference and [GTSAM 4.0](https://github.com/borglab/gtsam) for pose graph optimization.

## Install
To use this repo, first install these dependencies
- [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Pytorch 1.7](https://pytorch.org/)
- [GTSAM 4.0](https://github.com/borglab/gtsam)

To clone this repo as well as the DSAC* submodule use the command 

```
$ git clone --recursive https://github.com/alphonsusadubredu/posenet_gtsam
```

## Usage
To run the entire pose estimation pipeline, first open a new terminal and run the following commands
```
$ cd posenet_gtsam/ros_ws
$ catkin_make
$ source devel/setup.bash
$ rosrun gtsam_ros gtsam_rosnode
```
These series of commands starts up the gtsam solver service in the background.

Next, open a new terminal and run the following commands to perform pose estimation
```
$ cd src/gtsam_ros/scripts
$ python overall_workflow.py
```

## Authors
Alphonsus Adu-Bredu, Tianyi Liu, Yuqing Zhang, Jingyu Song, Noah Del Coro