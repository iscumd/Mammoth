#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "isc_shared_msgs/drive_mode.h"
#include "yeti_snowplow/location_point.h"
#include "yeti_snowplow/target.h"
#include "yeti_snowplow/waypoint.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <time.h>
#include <vector>
using namespace std;

ros::Publisher realPub;
ros::Publisher reactancePub;

void navigationCallback(const geometry_msgs::Twist::ConstPtr& velocity){
	geometry_msgs::Twist navVelocity = *velocity;

    geometry_msgs::Twist msgReal;
	msgReal.linear.x = 1 - (1/(5 * navVelocity.linear.x + 1));
    msgReal.angular.z = 1 - (1/(5 * navVelocity.angular.z + 1));
	realPub.publish(msgReal);

    geometry_msgs::Twist msgReactance;
	msgReactance.linear.x = 1.0;
    msgReactance.angular.z = 1.0;
	reactancePub.publish(msgReactance);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "navigation_pid_test");

	ros::NodeHandle n;

	realPub = n.advertise<geometry_msgs::Twist>("/localization/velocity", 5);
    reactancePub = n.advertise<geometry_msgs::Twist>("/obstacle_reactance/velocity", 5);

    ros::Subscriber navSub = n.subscribe("/navigation/velocity", 5, navigationCallback);

	// ros::spin();
	// ros::Rate loopRate(100); //Hz
	// while(ros::ok()) {
	// 	ros::spinOnce();
	// 	if(pidEnable){
	// 		pid();
	// 	}
	// 	loopRate.sleep();
	// }
    ros::spin();
	
	return 0;
}
