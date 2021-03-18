#include "gtsam_ros/gtsam_rosnode.h" 



GTSAM_node::GTSAM_node(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
	vector<float> priorMean{0.1,-0.1,0.01};
	vector<float> priorCov{0.01,-0.01, 0.1};
	solver.initialize(priorMean, priorCov);

	service_input = nh.advertiseService("isam_input_channel", 
		&GTSAM_node::receive_input_service, this);
	ROS_INFO("READY TO RUMBLE!"); 
}


bool GTSAM_node::receive_input_service(gtsam_ros::ISAMinput::Request &req,
		gtsam_ros::ISAMinput::Response &res)
{
	geometry_msgs::Pose2D motion_mean = req.motion_mean;
	geometry_msgs::Pose2D motion_covariance = req.motion_covariance;
	geometry_msgs::Pose2D measurement_mean = req.measurement_mean;
	geometry_msgs::Pose2D measurement_covariance = req.measurement_covariance;

	vector<float> odom_mean{(float)motion_mean.x, (float)motion_mean.y, (float)motion_mean.theta};
	vector<float> odom_cov{(float)motion_covariance.x, (float)motion_covariance.y, (float)motion_covariance.theta};
	vector<float> mes_mean{(float)measurement_mean.x, (float)measurement_mean.y, (float)measurement_mean.theta};
	vector<float> mes_cov{(float)measurement_covariance.x, (float)measurement_covariance.y, (float)measurement_covariance.theta};

	solver.step(odom_mean, odom_cov);
	solver.addObs(mes_mean, mes_cov);
	vector<float> estimate = solver.update(1);
 
	res.estimate.x = estimate[0];
	res.estimate.y = estimate[1];
	res.estimate.theta = estimate[2];

	return true;
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "gtsam_rosnode");
	ros::NodeHandle nh;
	GTSAM_node gtsam_node(&nh);
	ros::spin();
	return 0;
}