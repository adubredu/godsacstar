# PoseDSAC
This project repository contains code for PoseDSAC, an integrated system for 
pose estimation from RGB images.

## Table of Contents
- [PoseDSAC](#posedsac)
  - [Table of Contents](#table-of-contents)
  - [Background](#background)
  - [Dependencies](#dependencies)
  - [Install](#install)
  - [Dataset Preparation](#dataset-preparation)
    - [NCLT Dataset](#nclt-dataset)
    - [Other Datasets](#other-datasets)
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
- [Pytorch 1.6 or above](https://pytorch.org/), tested on Pytorch 1.6/1.7
- [GTSAM 4.0](https://github.com/borglab/gtsam)
- [OpenCV 3.4.2](https://opencv.org/)
- [scikit-image](https://scikit-image.org/)

To clone this repo as well as the DSAC* submodule use the command 

```
$ git clone --recursive https://github.com/alphonsusadubredu/posenet_gtsam
```

For the DSAC*, you will need to install its the C++ extension by
```
$ cd dsacstar/dsacstar
$ python setup.py install
```
Now you should have the DSAC* installed. Run `import dsacstar` to check!

## Dataset Preparation
Currently the DSAC* supports some popular datasets (e.g., [7Scenes (MSR)](https://www.microsoft.com/en-us/research/project/rgb-d-dataset-7-scenes/), [12Scenes (Stanford)](http://graphics.stanford.edu/projects/reloc/) and [Cambridge Landmarks](http://mi.eng.cam.ac.uk/projects/relocalisation/#dataset)) already. You may feel free to check the submodule of DSAC* to check its usage on aforementioned datasets.
We also add support for [U-M NCLT dataset](http://robots.engin.umich.edu/nclt/), which is a more challenging dataset as its extent is much larger and its feature is hard to detect.

### NCLT Dataset
Firstly, you will need to go to [NCLT website](http://robots.engin.umich.edu/nclt/) to download the sequence you wish. After downloading,  unpack the file and manage the data for each sequence as shown below.
```
dsacstar
├── datasets
│   ├── nclt_source
│   │   │── 2012-01-08
│   │   │   ├── groundtruth_2012-01-08
│   │   │   ├── lb3
|   |   |   |   |── CamX
│   │   │── another_seq
│   │   │   ├── groundtruth_another_seq
│   │   │   ├── lb3
|   |   |   |   |── CamX
├── dsacstar
```
You may also need to change some parameters according to the cam you use and region you want to sample. Please check the `datasets/setup_nclt.py` script and find the `TODO` block!

### Other Datasets
To setup other datasets is quite simple, run the following commands the dataset would be downloaded and unpacked automatically. You may also check [DSAC*](https://github.com/vislearn/dsacstar) check the datails for each dataset.
```
$ cd dsacstar/datasets
$ python setup_7scenes/12scenes/cambridge.py
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