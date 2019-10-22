#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "yeti_snowplow/location_point.h"
#include "yeti_snowplow/target.h"
#include "yeti_snowplow/waypoint.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <time.h>
#include <vector>
using namespace std;

struct CVAR{
	double targdist;
    double targbearing;
    double front;
    double right;
    double speed;               // between -1 to 1
    double turn;                // between -1 to 1
    double pErr;
    double lastpErr;
    double kP;
    double dErr;
    double kD;
    double iErr;
    double kI;
    double lookAhead;
};

ros::Publisher turnPub;
ros::ServiceClient waypointClient;
yeti_snowplow::target previousTarget;
yeti_snowplow::target currentTarget;
CVAR cvar;
double lastTime, thisTime;
double maxIntErr = 0.5;
double destinationThresh = 0.5;

double mathSign(double number){
	//Returns the number's sign
	//Equivalent to .NET's Math.Sign()
	//number>0 = 1
	//number=0 = 0
	//number<0 = -1
	if (number == 0){
		return 0;
	}
	else {
		return number / abs(number);
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
	cvar.pErr = cvar.iErr = cvar.dErr = 0;
}

void localizationCallback(const geometry_msgs::Pose2D::ConstPtr& location){	
	/* This fires every time a new position is published */

	double heading = location->theta;
	int dir = (int)currentTarget.dir;
	double dx, dy, s, c, dt;
	double desiredAngle;

	if (dir < 0){
		heading = heading - M_PI * mathSign(heading);
	}

	dx = currentTarget.location.x - location->x;
	dy = currentTarget.location.y - location->y;
	
	//FIND DISTANCE AND ANGLE TO DESTINATION
	cvar.targdist = sqrt(dx * dx + dy * dy);//current distance from the robot to the target
	// desired angle is the desired Heading the robot should have at this instance if it were to be facing the target.
	desiredAngle = adjust_angle(atan2(dx, dy), 2.0*M_PI);

	//USED FOR WAYPOINT NAVIGATION
	// cvar.right = dx * c - dy * s;
	// cvar.front = dy * c + dx * s;
	// c = cos(heading); //find Cosine term of the robots heading
	// s = sin(heading); //find sine term of the robots heading

	cvar.speed = currentTarget.speed;

	thisTime = ((double)clock()) / CLOCKS_PER_SEC;
	dt = thisTime - lastTime;

	cvar.lastpErr = cvar.pErr;
	cvar.pErr = adjust_angle(heading - desiredAngle, 2.0 * M_PI);
	cvar.iErr = cvar.iErr + cvar.pErr * dt;
	cvar.iErr = mathSign(cvar.iErr) * fmin(abs(cvar.iErr), maxIntErr);

	if (dt != 0){
		cvar.dErr = (cvar.pErr - cvar.lastpErr) / dt;
	}
	if (cos(cvar.pErr) > 0.5){ // +-60 degrees
		cvar.kP = 0.5;
		cvar.turn = -(cvar.kP * sin(cvar.pErr) *2 + cvar.kI * cvar.iErr + cvar.kD * cvar.dErr);  // Nattu
	}
	else {
		cvar.turn = -0.5 * mathSign(cvar.pErr); //if you need to turnin place, then ignore PID
	}
	lastTime = thisTime;

	geometry_msgs::Twist msg;
	msg.linear.x = cvar.speed;
	msg.angular.z = cvar.turn;
	turnPub.publish(msg);

	if (cvar.targdist < destinationThresh){ //reached target
		initPID();

		previousTarget = currentTarget;

		yeti_snowplow::waypoint waypointReq;
		waypointReq.request.ID = previousTarget.location.id + 1;
		if (waypointClient.call(waypointReq)){
			currentTarget = waypointReq.response.waypoint;
		}
		else { //we've hit the last waypoint or the service is no longer available
			//TODO
		}
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "navigation_pid");

	ros::NodeHandle n;

	turnPub = n.advertise<geometry_msgs::Twist>("navigationPID", 5);

	waypointClient = n.serviceClient<yeti_snowplow::waypoint>("waypoint");
	yeti_snowplow::waypoint waypointReq;
	// for (int i=0; i < 13; i++){        
	//     waypointReq.request.ID = i;
	//     ROS_INFO("Sent request: %i", waypointReq.request.ID);
	//     while (waypointClient.call(waypointReq) == false){
	//         ROS_ERROR("Failed to call the waypoint service");
	//         ROS_INFO("Sent request: %i", waypointReq.request.ID);
	//     }

	//     ROS_INFO("Received response: x=%f y=%f heading=%f dir=%i PID=%s speed=%f", waypointReq.response.x, waypointReq.response.y, waypointReq.response.heading, waypointReq.response.dir, waypointReq.response.PID?"true":"false", waypointReq.response.speed);
	// }
	waypointReq.request.ID = 0;
	ROS_INFO("Sent request: %i", waypointReq.request.ID);
	while (waypointClient.call(waypointReq) == false){ //the service may not be ready yet, so we'll keep trying until we get a response
		ROS_ERROR("Failed to call the waypoint service; trying again");
		ROS_INFO("Sent request: %i", waypointReq.request.ID);
	}
	currentTarget = waypointReq.response.waypoint;

	initPID();

	ros::Subscriber localizationSub = n.subscribe("localization", 5, localizationCallback); //TODO: get name of localization topic

	ros::spin();
	
	return 0;
}
