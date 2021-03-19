#!/usr/bin/env python
import rospy
import roslib
from geometry_msgs.msg import Pose2D
from gtsam_ros.srv import *
import numpy as np 


def send_measurement_request():
	rospy.wait_for_service("isam_input_channel")
	try:
		channel = rospy.ServiceProxy("isam_input_channel", ISAMinput)
		send_request = ISAMinputRequest()
		send_request.motion_mean.x = 5.0
		send_request.motion_mean.y = 0.0
		send_request.motion_mean.theta = 0.0
		send_request.motion_covariance.x = 0.01
		send_request.motion_covariance.y = 0.01
		send_request.motion_covariance.theta = 0.1
		send_request.measurement_mean.x = 0.0
		send_request.measurement_mean.y = 0.0
		send_request.measurement_mean.theta = 0.0
		send_request.measurement_covariance.x = 0.01
		send_request.measurement_covariance.y = 0.01
		send_request.measurement_covariance.theta = 0.1
		response = channel(send_request)

		print("estimate is: ",response.estimate.x,response.estimate.y,
			response.estimate.theta)
	except rospy.ServiceException as e:
		print(e)


if __name__ == "__main__":
	send_measurement_request()