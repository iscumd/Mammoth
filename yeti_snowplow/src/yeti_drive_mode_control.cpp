#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "isc_joy/xinput.h"
#include "isc_shared_msgs/drive_mode.h"
#include "isc_shared_msgs/wheel_speeds.h"
#include "std_msgs/Bool.h"

#include <sstream>
#include <string>

ros::Publisher driveModePub;

bool startButtonDown = false;
bool autoMode = false;
ros::Publisher wheelSpeedPub;
ros::Publisher pidEnablePub;

int speedBoostButton = false;

void updateDriveMode(){
	ROS_INFO("Drive Mode Control: Switching to %s mode.", autoMode ? "AUTO" : "MANUAL");
	if(autoMode){
		isc_shared_msgs::drive_mode msg;
		msg.mode = "auto";
		driveModePub.publish(msg);

                std_msgs::Bool pidEnableMsg;
                pidEnableMsg.data = 1;
                pidEnablePub.publish(pidEnableMsg);
	}
	else {
		isc_shared_msgs::drive_mode msg;
		msg.mode = "manual";
		driveModePub.publish(msg);

                std_msgs::Bool pidEnableMsg;
                pidEnableMsg.data = 0;
                pidEnablePub.publish(pidEnableMsg);
	}
}

void joystickCallback(const isc_joy::xinput::ConstPtr& joy){	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */

	/* We track the button state to ensure you can't 
	accidentally hold down Start and press another 
	button to toggle the mode */
	if(joy->Start){ //The Start button has been pressed
		startButtonDown = true;
	}
	if(startButtonDown && !joy->Start){ //The Start button has been released
		startButtonDown = false;
		autoMode = !autoMode;
		updateDriveMode();
	}
	speedBoostButton = joy->LS;
}

void manualCallback(const geometry_msgs::Twist::ConstPtr& msg){
	if(!autoMode){
		float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;

		if(speedBoostButton){
            leftWheelSpeed = (msg->linear.x - msg->angular.z);
            rightWheelSpeed = (msg->linear.x + msg->angular.z);
        }
		else{
            leftWheelSpeed = (msg->linear.x - msg->angular.z)/2;
            rightWheelSpeed = (msg->linear.x + msg->angular.z)/2;
        }

		isc_shared_msgs::wheel_speeds msg;
		msg.left = leftWheelSpeed;
		msg.right =  rightWheelSpeed;
		wheelSpeedPub.publish(msg);
	}
}

void autoCallback(const geometry_msgs::Twist::ConstPtr& msg){
	if(autoMode){
		float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;

		leftWheelSpeed = (msg->linear.x - msg->angular.z);
		rightWheelSpeed = (msg->linear.x + msg->angular.z);

		isc_shared_msgs::wheel_speeds msg;
		msg.left = leftWheelSpeed;
		msg.right =  rightWheelSpeed;
		wheelSpeedPub.publish(msg);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "yeti_drive_mode_control");

	ros::NodeHandle n;

	driveModePub = n.advertise<isc_shared_msgs::drive_mode>("yeti/drive_mode", 1000, true);
	wheelSpeedPub = n.advertise<isc_shared_msgs::wheel_speeds>("motors/wheel_speeds", 5);
	pidEnablePub = n.advertise<std_msgs::Bool>("/pid_enable", 1, true);

	updateDriveMode();

	ros::Subscriber joystickSub = n.subscribe("/joystick/xinput", 5, joystickCallback);
	ros::Subscriber manualSub = n.subscribe("manual_control", 5, manualCallback);
	ros::Subscriber autoSub = n.subscribe("auto_control", 5, autoCallback);

	ros::spin();
	
	return 0;
}
