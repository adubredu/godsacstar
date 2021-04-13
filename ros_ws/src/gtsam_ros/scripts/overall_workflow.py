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

from sensor_msgs.msg import Image
import cv2

# dsac imports
from dsacStar import dsacStar

# plotting
from matplotlib import pyplot as plt


# odom = odometry('/root/posenet_gtsam/ros_ws/src/gtsam_ros/data/', 0, 0)
odom = odometry('/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/data')
image_dataset = '/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/data/rgb'
weightsDir = '/home/tannerliu/Software/posenet_gtsam/dsacstar/network_output/nclt_0413_e2e.pth'
focalLength = 1.556233184415584390e+02

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
        send_request.motion_covariance.x = 10
        send_request.motion_covariance.y = 10
        send_request.motion_covariance.theta = 10

        #sending measurement pose
        send_request.measurement_mean.x = measurement[0]
        send_request.measurement_mean.y = measurement[1]
        send_request.measurement_mean.theta = measurement[2]
        send_request.measurement_covariance.x = 20
        send_request.measurement_covariance.y = 20
        send_request.measurement_covariance.theta = 3.14/4

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
    result_x = []
    result_y = []
    ds = dsacStar(weightsDir, focalLength)

    # # dbg
    # i = 0
    # stop = 250

    for image in sorted(os.listdir(image_dataset)):
        print(image)
        imageDir = image_dataset + "/" + image
        img_timestamp = int(image[11:-10])
        measurement = ds.predict(imageDir)
        # extract SE(2) pose
        r = R.from_matrix(measurement[:3, :3])
        theta = r.as_euler('zyx', degrees=False)[0]
        measurement = [measurement[0,3], measurement[1,3], theta]
        # retrieve current time step odometry
        odom_pose = get_odometry_pose(img_timestamp)
        result = optimize_pose_graph(measurement, odom_pose)
        result_estimates.append(result)
        result_x.append(result[0])
        result_y.append(result[1])
        # i += 1
        # if i == stop:
        #     break

    # saving
    result_x = np.array(result_x)
    np.save("/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/helper/gtsam_x", result_x)
    result_y = np.array(result_y)
    np.save("/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/helper/gtsam_y", result_y)


# def callback(msg):
#     stamp = msg.header.stamp
#     print(stamp)
#     # img = bridge.imgmsg_to_cv2(msg)
#     img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
#     cv2.imshow("duh", img) # TODO: BGR or RGB?
#     cv2.waitKey(0)
#     a = 1



# if __name__ == '__main__':
#     rospy.init_node('pose_estimator')
#     rospy.Subscriber('nclt/lb3', Image, callback)
#     rospy.spin()
