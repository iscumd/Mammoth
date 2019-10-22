#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

#include <string>
#include <time.h>
#include <vector>
using namespace std;

ros::Publisher navigationVelocityPub;
double speed = 0.0;
double turn = 0.0;

void navigationSpeedCallback(const std_msgs::Float64::ConstPtr& newVal){	
	speed = newVal->data;
}

void navigationTurnCallback(const std_msgs::Float64::ConstPtr& newVal){	
	turn = newVal->data;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "navigation_pid_velocity");

	ros::NodeHandle n;

	navigationVelocityPub = n.advertise<geometry_msgs::Twist>("/navigation/velocity", 5);

	ros::Subscriber speedSub = n.subscribe("/navigation/speed", 5, navigationSpeedCallback);
	ros::Subscriber turnSub = n.subscribe("/navigation/turn", 5, navigationTurnCallback);

	// ros::spin();
	ros::Rate loopRate(100); //Hz
	while(ros::ok()) {
		ros::spinOnce();
		
		geometry_msgs::Twist msg;
		msg.linear.x = speed;
		msg.angular.z = turn;
		navigationVelocityPub.publish(msg);

		loopRate.sleep();
	}
	
	return 0;
}
