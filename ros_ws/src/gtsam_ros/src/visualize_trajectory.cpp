#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>

std::vector<double> euler_to_quat(double yaw) {
    // Roll and pitch are 0
    double roll = 0, pitch = 0;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    std::vector<double> quaternion(4);
    quaternion[0] = cr * cp * cy + sr * sp * sy; //w
    quaternion[1] = sr * cp * cy - cr * sp * sy; //x
    quaternion[2] = cr * sp * cy + sr * cp * sy; //y
    quaternion[3] = cr * cp * sy - sr * sp * cy; //z
    return quaternion;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualize_trajectory");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    uint32_t shape = visualization_msgs::Marker::ARROW;
    
    while (ros::ok()) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/segway";
        marker.header.stamp = ros::Time::now();
        marker.ns = "visualize_trajectory";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.3;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();


        // estimated pose from GTSAM
        double est_x = 0;
        double est_y = 0;
        double est_theta = 0;
        
        marker.pose.position.x = est_x;
        marker.pose.position.y = est_y;
        marker.pose.position.z = 0;
        std::vector<double> quat = euler_to_quat(est_theta);
        marker.pose.orientation.w = quat[0];
        marker.pose.orientation.x = quat[1];
        marker.pose.orientation.y = quat[2];
        marker.pose.orientation.z = quat[3];

        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok())
                return 0;
            ROS_WARN_ONCE("Check if RVIZ is opend");
            sleep(1);
        }
        marker_pub.publish(marker);
    }
}