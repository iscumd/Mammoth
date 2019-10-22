#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "isc_shared_msgs/drive_mode.h"
#include <geometry_msgs/Twist.h>

//from params
float REVERSE_SPEED;
float ANGULAR_TOLERANCE;
float SPEED_TOLERANCE;
int STALL_TIMER;

ros::Subscriber vel_setpoint_sub;
ros::Subscriber angular_setpoint_sub;
float vel_setpoint;
float angular_setpoint;
ros::Subscriber vel_actual_sub;
ros::Subscriber angular_actual_sub;
float vel_actual;
float angular_actual;
ros::Subscriber drive_mode_sub;
std::string drive_mode;

bool is_stalled;
geometry_msgs::Twist stall_vel;
ros::Publisher stall_vel_pub;
ros::Publisher nav_status_pub;

void velocitySetpointCallback(const std_msgs::Float64::ConstPtr& msg) {
	vel_setpoint = msg->data;
}

void angularSetpointCallback(const std_msgs::Float64::ConstPtr& msg) {
	angular_setpoint = msg->data;
}

void velocityActualCallback(const std_msgs::Float64::ConstPtr& msg) {
	vel_actual = msg->data;
}

void angularActualCallback(const std_msgs::Float64::ConstPtr& msg) {
	angular_actual = msg->data;
}

void driveModeCallback(const isc_shared_msgs::drive_mode::ConstPtr& msg) {
	drive_mode = msg->mode;
}

void detectStall(const ros::TimerEvent& event) {
	if (drive_mode != "auto") {
		is_stalled = false;
		return;
	}
	if (angular_actual < angular_setpoint * ANGULAR_TOLERANCE) {
		if (vel_actual < vel_setpoint * SPEED_TOLERANCE) {
			is_stalled = true;
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "stall_detection");
    ros::NodeHandle n;

	stall_vel.linear.x = REVERSE_SPEED;
	stall_vel.angular.z = 0.0;
	is_stalled = false;

	ros::Timer stall_timer = n.createTimer(ros::Duration(STALL_TIMER), detectStall);

	vel_setpoint_sub = n.subscribe("linear_velocity_setpoint", 1, velocitySetpointCallback);
	angular_setpoint_sub = n.subscribe("angular_velocity_setpoint", 1, angularSetpointCallback);
	vel_actual_sub = n.subscribe("linear_velocity", 1, velocityActualCallback);
	angular_actual_sub = n.subscribe("angular_velocity", 1, angularActualCallback);
	drive_mode_sub = n.subscribe("yeti/drive_mode", 1, driveModeCallback);

    stall_vel_pub = n.advertise<geometry_msgs::Twist>("/stall/velocity", 1000);
    //nav_status_pub = n.advertise<std_msgs::Bool>("/navigation/disable", 1000);

	/*
		REVERSE_SPEED		Speed yeti reverses at when a stall is detected

		ANGULAR_TOLERANCE	Percentage of angular speed which determines the
							acceptable range to activate stall detection
								(actual speed) < tolerance% * (setpoint speed)

		ANGULAR_TOLERANCE	Percentage of linear speed which determines the
							acceptable range to activate stall detection
								(actual speed) < tolerance% * (setpoint speed)

		STALL_TIMER			How often in seconds to check for a stall event
	*/
	n.param("stall_detection_reverse_speed", REVERSE_SPEED, -0.5f);
	n.param("stall_detection_angular_tolerance", ANGULAR_TOLERANCE, 0.01f);
	n.param("stall_detection_speed_tolerance", SPEED_TOLERANCE, 0.01f);
	n.param("stall_detection_stall_timer", STALL_TIMER, 1);

	while(ros::ok()) {
		ros::spinOnce();
		if(is_stalled)
        	stall_vel_pub.publish(stall_vel);
	}
	return 0;
}
