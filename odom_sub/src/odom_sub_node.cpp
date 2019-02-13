#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <XmlRpcException.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){}

int main(int argc, char **argv){
	ros::init(argc, argv, "odom_sub");
	
	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("/zed/odom", 1, odomCallback);
	
	ros::spin();
	
	return 0;
	
}
