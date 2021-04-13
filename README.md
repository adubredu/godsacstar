# PoseDSAC
This project repository contains code for PoseDSAC, an integrated system for 
pose estimation from RGB images.

## Table of Contents
- [Background](#background)
- [Install](#install)
- [Usage](#usage)
- [Authors](#authors)

## Background
RGB cameras are cheap and ubiquitous sensors that can be found in almost every mobile device. In view of this, recent work in SLAM research has focused on performing pose estimation and localization using only RGB images from cameras. These efforts have been fairly successful at extracting features from images which could be used by a robot to track its displacement and its pose in a room. The feature extractors used in such efforts are often hand-designed and do not work
as well in environments where the sought-after features are scarce. 

The recent advancement in Deep Learning research has led to the development of effective function approximators that could be trained with the right data to approximate almost any function. In view of this, recent work in Visual SLAM has focused on ways to train neural networks to be effective feature extractors and pose estimators. In this work, we train a neural network to predict the pose of a robot given a sequence of images taken as the robot moves around a large area. We then jointly optimize the posterior joint probability of the predicted pose from the neural network and the robot's noisy odometry using a Pose SLAM formulation to get a better estimate of the robot's pose. We perform this optimization iteratively using GTSAM's implementation of iSAM2, an iterative smoothing and mapping system.

This repository contains our implementation of this pose estimation system. Our core design philosophy in this implementation is to strive for speed of inference and estimation. We use ROS melodic as the main communication protocol between modules. 

## Install
Although this project was built and tested using [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu), most of the other versions of ROS could potentially support this project.

This project also uses [Pytorch](https://pytorch.org/) for neural network training and inference and [GTSAM](https://github.com/borglab/gtsam) for pose graph optimization


## Usage

## Authors