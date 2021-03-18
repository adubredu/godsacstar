#ifndef GTSAM_ROSNODE_H
#define GTSAM_ROSNODE_H

#include "ros/ros.h"
#include "gtsamSolver.h"
#include "gtsam_ros/ISAMinput.h"
#include <geometry_msgs/Pose2D.h>
#include <sstream>

using namespace std;

class GTSAM_node
{
public:
	GTSAM_node(ros::NodeHandle* nodehandle);
	bool receive_input_service(gtsam_ros::ISAMinput::Request &req,
		gtsam_ros::ISAMinput::Response &res);

private:
	ros::NodeHandle nh;
	ros::ServiceServer service_input;
	PoseNetiSam solver;
};

#endif