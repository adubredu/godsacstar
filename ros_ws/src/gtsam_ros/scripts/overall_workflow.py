#!/usr/bin/env python
import rospy
import roslib
from geometry_msgs.msg import Pose2D
from gtsam_ros.srv import *
import numpy as np 
from read_odometry import odometry 
import sys

odom = odometry('/root/posenet_gtsam/ros_ws/src/gtsam_ros/data/', 0, 0)

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

 
def neural_network(image):
	prediction = [0,0,0]
	return prediction


def optimize_pose_graph(measurement, odom_pose):
	rospy.wait_for_service("isam_input_channel")
	opt_result = [None, None, None]
	try:
		channel = rospy.ServiceProxy("isam_input_channel", ISAMinput)
		send_request = ISAMinputRequest()

		send_request.motion_mean.x = odom_pose[0][0]
		send_request.motion_mean.y = odom_pose[0][1]
		send_request.motion_mean.theta = odom_pose[0][2]
		send_request.motion_covariance.x = odom_pose[1][0]
		send_request.motion_covariance.y = odom_pose[1][1]
		send_request.motion_covariance.theta = odom_pose[1][2]

		send_request.measurement_mean.x = measurement[0]
		send_request.measurement_mean.y = measurement[1]
		send_request.measurement_mean.theta = measurement[2]
		send_request.measurement_covariance.x = 0.01
		send_request.measurement_covariance.y = 0.01
		send_request.measurement_covariance.theta = 0.1
		response = channel(send_request)

		# print("estimate is: ",response.estimate.x,response.estimate.y,
		# 	response.estimate.theta)
		opt_result[0] = response.estimate.x
		opt_result[1] = response.estimate.y
		opt_result[2] = response.estimate.theta

	except rospy.ServiceException as e:
		print(e)
		sys.exit()

	return opt_result






 if __name__ == '__main__': 
 	result_estimates = []

	for image, img_timestamp in image_dataset:
		measurement = neural_network(image)

		odom_pose = get_odometry_pose(img_timestamp)

		result = optimize_pose_graph(measurement, odom_pose)

		result_estimates.append(result)

		
		


        

