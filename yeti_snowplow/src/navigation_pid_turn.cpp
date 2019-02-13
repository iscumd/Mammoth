#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
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

//PID CONTROL
//u(t) = Kp * e(t) + Ki * Integral of e(t) + Kd * derivative of e(t)
double turn;               // between -1 to 1
//Proportional
//P accounts for present values of the error. For example, if the error is large 
//and positive, the control output will also be large and positive.
double kP; // Proportional Term of PID
double pErr; // Current proportional Error
double lastpErr; //Last proportional Error
//DERIVATIVE
//D accounts for possible future trends of the error, based on its current rate of change.
double kD; //Derivative Term of PID
double dErr; // calculated Derrivative Error
//INTEGRAL
//I accounts for past values of the error.For example, if the current 
//output is not sufficiently strong, the integral of the error will 
//accumulate over time, and the controller will respond by applying a stronger action.
double kI; // Integral Term
double iErr; //

ros::Publisher pub;
// geometry_msgs::Twist previousTargetVelocity;
geometry_msgs::Twist currentTargetVelocity;
geometry_msgs::Twist realVelocity;
double lastTime, thisTime;
double maxIntErr;
bool isAutoMode = false;
bool stallDisable = false;

double mathSign(double number){
	//Returns the number's sign
	//Equivalent to .NET's Math.Sign()
	//number>0 = 1
	//number=0 = 0
	//number<0 = -1
	if (number == 0){
		return 0;
	}
	else if (number > 0) {
		return 1;
	}
	else {
		return -1;
	}
}

double adjust_angle(double angle, double circle){
	//circle = 2pi for radians, 360 for degrees
	// Subtract multiples of circle
	angle -= floor(angle / circle) * circle;
	angle -= floor(2 * angle / circle) * circle;

	return angle;
}

void initPID(){
	lastTime = ((double)clock()) / CLOCKS_PER_SEC;
	pErr = iErr = dErr = 0;
}

void driveModeCallback(const isc_shared_msgs::drive_mode::ConstPtr& msg){
	isAutoMode = false;
	if(msg->mode == "auto"){
		isAutoMode = true;
	}
}

void stallDisableCallback(const std_msgs::Bool::ConstPtr& msg){
	stallDisable = msg->data;
}

void obstacleReactanceVelocityCallback(const geometry_msgs::Twist::ConstPtr& velocity){	
	/* This fires every time a new velocity is published */

	if(velocity->linear.x != currentTargetVelocity.linear.x || velocity->angular.z != currentTargetVelocity.angular.z){
		// previousTargetVelocity = currentTargetVelocity;
		currentTargetVelocity = *velocity;
		initPID();
	}
}

void localizationVelocityCallback(const geometry_msgs::Twist::ConstPtr& velocity){
	realVelocity = *velocity;
}

void pid(){
	double heading = realVelocity.angular.z;//location->theta;
	int dir = mathSign(currentTargetVelocity.linear.x);//(int)currentTarget.dir;
	double dt; //Delta time. Holds the difference in time from the last time this function was called to this time.
	double desiredAngle; // the desired heading. The heading which would cause the robot to directly face the target

	if (dir < 0){ //if direction says we should go backward, turn heading around
		heading = heading - M_PI * mathSign(heading);
	}
	//FIND ANGLE TO DESTINATION
	// desired angle is the desired Heading the robot should have at this instance if it were to be facing the target.
	desiredAngle = adjust_angle(currentTargetVelocity.angular.z/*atan2(dx, dy)*/, 2.0*M_PI);

	thisTime = ((double)clock()) / CLOCKS_PER_SEC;
	dt = thisTime - lastTime;

	/* Current Target Heading PID Calculations */
	lastpErr = pErr; //save the last proportional error
	pErr = adjust_angle(heading - desiredAngle, 2.0 * M_PI); //calculate the current propotional error between our current heading and the target heading
	iErr = iErr + pErr * dt; //increase the cumulated error.
	iErr = mathSign(iErr) * fmin(abs(iErr), maxIntErr); //limit the maxmium integral error

	if (dt != 0){ //if the time has changed since the last iteration of guide. (cannot divide by 0).
		dErr = (pErr - lastpErr) / dt; // calculate the derrivative error
	}
	if (cos(pErr) > 0.5){ //if the robot is not facing more than +-60 degrees away from the target
		turn = -(kP * sin(pErr) *2 + kI * iErr + kD * dErr);  //Nattu; calulate how much the robot should turn at this instant.
	}
	else { //if the robot is facing more than 60 degrees away from the target
		turn = -0.5 * mathSign(pErr); //if you need to turn in place, then ignore PID temporarily
	}
	lastTime = thisTime;

	std_msgs::Float64 msg;
	msg.data = turn;
	pub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "navigation_pid_turn");

	ros::NodeHandle n("~");

	n.param<double>("proportional_constant", kP, 0.5);
	n.param<double>("derivative_constant", kD, 0.0);
	n.param<double>("integral_constant", kI, 0.0);
	n.param<double>("integral_max_error", maxIntErr, 0.5);

	pub = n.advertise<std_msgs::Float64>("/navigation/turn", 5);

	initPID();

	ros::Subscriber driveModeSub = n.subscribe("/yeti/drive_mode", 5, driveModeCallback);
	ros::Subscriber stallDisableSub = n.subscribe("/navigation/disable", 5, stallDisableCallback);

	ros::Subscriber reactanceVelocitySub = n.subscribe("/obstacle_reactance/velocity", 5, obstacleReactanceVelocityCallback);
	ros::Subscriber localizationVelocitySub = n.subscribe("/localization/velocity", 5, localizationVelocityCallback);

	// ros::spin();
	ros::Rate loopRate(100); //Hz
	while(ros::ok()) {
		ros::spinOnce();
		if(!isAutoMode && !stallDisable){
			pid();
		}
		loopRate.sleep();
	}
	
	return 0;
}
