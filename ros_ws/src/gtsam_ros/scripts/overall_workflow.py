#!/usr/bin/env python3
import rospy
import roslib
import tf
from geometry_msgs.msg import Pose, PoseArray 
from gtsam_ros.srv import *
import numpy as np 
from read_odometry import odometry 
import sys
import os
from scipy.spatial.transform import Rotation as R

# dsac imports
from dsacStar import dsacStar


# odom = odometry('/root/posenet_gtsam/ros_ws/src/gtsam_ros/data/', 0, 0)
odom = odometry('/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/data')
image_dataset = '/home/tannerliu/Software/posenet_gtsam/ros_ws/src/gtsam_ros/data/rgb'
weightsDir = '/home/tannerliu/Software/posenet_gtsam/dsacstar/network_output/nclt_trial_v2_e2e.pth'
focalLength = 100 # TODO: modify this

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


def publish_entire_trajectory(estimates):
    traj = []
    for pose in estimates:
        p = Pose()
        p.position.x = pose[0]
        p.position.y = pose[1]
        quaternion = tf.transformations.quaternion_from_euler(0,0,pose[2])
        p.orientation.x = quaternion[0]
        p.orientation.y = quaternion[1]
        p.orientation.z = quaternion[2]
        p.orientation.w = quaternion[3]
        traj.append(p)
    array = PoseArray()
    array.poses = traj  
    
    pub = rospy.Publisher('/estimated_poses', PoseArray, queue_size=100)
    pub.publish(array)


def publish_single_pose(pose_estimate):
    p = Pose()
    p.position.x = pose[0]
    p.position.y = pose[1]
    quaternion = tf.transformations.quaternion_from_euler(0,0,pose[2])
    p.orientation.x = quaternion[0]
    p.orientation.y = quaternion[1]
    p.orientation.z = quaternion[2]
    p.orientation.w = quaternion[3] 
    pub = rospy.Publisher('/estimated_pose', Pose, queue_size=100)
    pub.publish(p)#


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
        # send_request.motion_covariance.x = odom_pose[1][0]
        # send_request.motion_covariance.y = odom_pose[1][1]
        # send_request.motion_covariance.theta = odom_pose[1][2]
        send_request.motion_covariance.x = 0.1
        send_request.motion_covariance.y = 0.1
        send_request.motion_covariance.theta = 0.1

        #sending measurement pose
        send_request.measurement_mean.x = measurement[0]
        send_request.measurement_mean.y = measurement[1]
        send_request.measurement_mean.theta = measurement[2]
        send_request.measurement_covariance.x = 0.1
        send_request.measurement_covariance.y = 0.1
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
    rospy.init_node('pose_estimator')

    result_estimates = []
    ds = dsacStar(weightsDir, focalLength)
    for image in sorted(os.listdir(image_dataset)):
        print(image)
        imageDir = image_dataset + "/" + image
        img_timestamp = int(image[11:-10])
        measurement = ds.predict(imageDir)
        # extract SE(2) pose
        r = R.from_matrix(measurement[:3, :3])
        theta = r.as_euler('zxy', degrees=False)[0]
        measurement = [measurement[0,3], measurement[1,3], theta]
        # retrieve current time step odometry
        odom_pose = get_odometry_pose(img_timestamp)
        result = optimize_pose_graph(measurement, odom_pose)
        result_estimates.append(result)

        # publish_single_pose(result)

    publish_entire_trajectory(result_estimates)
