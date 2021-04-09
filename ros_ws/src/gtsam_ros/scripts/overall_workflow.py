#!/usr/bin/env python3
import rospy
import roslib
from geometry_msgs.msg import Pose2D
from gtsam_ros.srv import *
import numpy as np 
from read_odometry import odometry 
import sys
import os
from scipy.spatial.transform import Rotation as R

# dsac imports
sys.path.append('/home/tannerliu/Software/posenet_gtsam/dsacstar')
import torch
import cv2
import dsacstar
from network import Network
from skimage import io
from torchvision import transforms
from matplotlib import pyplot as plt


# odom = odometry('/root/posenet_gtsam/ros_ws/src/gtsam_ros/data/', 0, 0)
odom = odometry('/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/data')
image_dataset = '/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/data/rgb'

def get_odometry_pose(img_timestamp):
    motionCum = [0,0,0]
    motionCovCum = [0,0,0]
    motion, motionCov, timestamp = odom.getOdometry()
    while timestamp <= img_timestamp:
        motion,motionCov,timestamp = odom.getOdometry()
        motionCum = [motion[0]+motionCum[0],
                     motion[1]+motionCum[1],
                     motion[2]+motionCum[2]]
        motionCovCum = [motionCov[0]+motionCovCum[0],
                        motionCov[1]+motionCovCum[1],
                        motionCov[2]+motionCovCum[2]]
    return motionCum, motionCovCum


def nn_init():
    scene = 'nclt'
    weightsDir = '/home/tannerliu/Software/dsacstar/network_output/nclt_trial_v2_e2e.pth'
    # hyperparameters
    hypotheses = 64 # number of hypotheses, i.e. number of RANSAC iterations
    threshold = 10 # inlier threshold in pixels (RGB) or centimeters (RGB-D)
    inlieralpha = 100 # alpha parameter of the soft inlier count; controls the softness of the hypotheses score distribution; lower means softer
    maxpixelerror = 100 # maximum reprojection (RGB, in px) or 3D distance (RGB-D, in cm) error when checking pose consistency towards all measurements; error is clamped to this value for stability
    mode = 1 # test mode: 1 = RGB, 2 = RGB-D
    # dataset parameters
    focal_length = 100
    # load weights
    network = Network(torch.zeros((3)), False)
    network.load_state_dict(torch.load(weightsDir))
    network = network.cuda()
    network.eval()
    #define image processing elements
    image_transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize(int(480)),
        transforms.Grayscale(),
        transforms.ColorJitter(brightness=0.1, contrast=0.1),
        transforms.ToTensor(),
        transforms.Normalize(
            mean=[0.4],
            std=[0.25]
            )
    ])
    return network, image_transform
 
def nn_predict(network, imgTrans, imageDir):
    image = io.imread(imageDir)
    image = imgTrans(image)
    image = image.unsqueeze(0)
    image = image.cuda()
    scene_coordinates = network(image)
    scene_coordinates = scene_coordinates.cpu()
    out_pose = torch.zeros((4, 4))
    hypotheses = 64 # number of hypotheses, i.e. number of RANSAC iterations
    threshold = 10 # inlier threshold in pixels (RGB) or centimeters (RGB-D)
    inlieralpha = 100 # alpha parameter of the soft inlier count; controls the softness of the hypotheses score distribution; lower means softer
    maxpixelerror = 100 # maximum reprojection (RGB, in px) or 3D distance (RGB-D, in cm) error when checking pose consistency towards all measurements; error is clamped to this value for stability
    mode = 1 # test mode: 1 = RGB, 2 = RGB-D
    focal_length = 100
    dsacstar.forward_rgb(
        scene_coordinates, 
        out_pose, 
        hypotheses, 
        threshold,
        focal_length, 
        float(image.size(3) / 2), #principal point assumed in image center
        float(image.size(2) / 2), 
        inlieralpha,
        maxpixelerror,
        network.OUTPUT_SUBSAMPLE)
    out_pose = out_pose.inverse().numpy()
    r = R.from_matrix(out_pose[:3, :3])
    theta = r.as_euler('zxy', degrees=False)[0]
    prediction = [out_pose[0,3], out_pose[1,3], theta]
    return prediction


def optimize_pose_graph(measurement, odom_pose):
    rospy.wait_for_service("isam_input_channel")
    opt_result = [None, None, None]
    try:
        channel = rospy.ServiceProxy("isam_input_channel", ISAMinput)
        send_request = ISAMinputRequest()

        #sending odometry pose
        send_request.motion_mean.x = odom_pose[0][0]
        send_request.motion_mean.y = odom_pose[0][1]
        send_request.motion_mean.theta = odom_pose[0][2]
        send_request.motion_covariance.x = odom_pose[1][0]
        send_request.motion_covariance.y = odom_pose[1][1]
        send_request.motion_covariance.theta = odom_pose[1][2]

        #sending measurement pose
        send_request.measurement_mean.x = measurement[0]
        send_request.measurement_mean.y = measurement[1]
        send_request.measurement_mean.theta = measurement[2]
        send_request.measurement_covariance.x = 0.01
        send_request.measurement_covariance.y = 0.01
        send_request.measurement_covariance.theta = 0.1

        #receiving optimized pose
        response = channel(send_request)

        #reading optimized x,y,th
        opt_result[0] = response.estimate.x
        opt_result[1] = response.estimate.y
        opt_result[2] = response.estimate.theta

    except rospy.ServiceException as e:
        '''
        if this exception is caught, it's likely that you didn't run 
         `rosrun gtsam_ros gtsam_rosnode`  to start the gtsam solver service.
        '''
        print(e)
        sys.exit()

    return opt_result






if __name__ == '__main__':
    result_estimates = []
    network, imgTrans = nn_init()
    for image in os.listdir(image_dataset):
        imageDir = image_dataset + "/" + image
        img_timestamp = int(image[11:-10])
        measurement = nn_predict(network, imgTrans, imageDir)

        odom_pose = get_odometry_pose(img_timestamp)

        result = optimize_pose_graph(measurement, odom_pose)

        result_estimates.append(result)