#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "isc_shared_msgs/EncoderCounts.h"

#include <math.h>
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>
#include <sstream>
#include <string>

using std::string;
using std::stringstream;
using serial::Serial;
using serial::utils::SerialListener;
using serial::utils::BufferedFilterPtr;

//serial code based on https://github.com/wjwwood/ax2550/
std::string port;
serial::Serial *serialPort;
serial::utils::SerialListener serialListener;
bool roboteqIsConnected = false;
bool hasEncoder = false;
bool flip_inputs = false;
double leftSpeed = 0, rightSpeed = 0;
double gearReduction = 1.0;
bool enableLogging;

void driveModeCallback(const geometry_msgs::Twist::ConstPtr &msg){	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */
	float speedMultiplier = 1000.0; 

	leftSpeed = (msg->linear.x - msg->angular.z) * speedMultiplier;
	rightSpeed = (msg->linear.x + msg->angular.z) * speedMultiplier;
	
	if(enableLogging) ROS_INFO("Roboteq: left wheel=%f right wheel=%f", leftSpeed, rightSpeed);
}

void disconnect(){
	roboteqIsConnected = false;
	if(serialListener.isListening()){
		serialListener.stopListening();
	}
	if(serialPort != NULL) {
		delete serialPort;
		serialPort = NULL;
	}
}

void connect(){
	if(roboteqIsConnected){
		ROS_WARN("Roboteq already connected");
		return;
	}
	if(port.empty()){
		ROS_ERROR("Serial port name is empty.");
		return;
	}

	disconnect();

	serialPort = new Serial();
	serialPort->setPort(port);
 	serialPort->setBaudrate(115200);
  	//serialPort->setParity(serial::parity_even);
  	serialPort->setStopbits(serial::stopbits_one);
  	serialPort->setBytesize(serial::eightbits);
	serial::Timeout to = serial::Timeout::simpleTimeout(10);
	serialPort->setTimeout(to);

	serialPort->open();

	serialListener.setChunkSize(2);
	serialListener.startListening(*serialPort);

	roboteqIsConnected = true;
}

inline string stringFormat(const string &fmt, ...) {
  int size = 100;
  string str;
  va_list ap;
  while (1) {
	str.resize(size);
	va_start(ap, fmt);
	int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
	va_end(ap);
	if (n > -1 && n < size) {
	  str.resize(n);
	  return str;
	}
	if (n > -1) {
	  size = n + 1;
	} else {
	  size *= 2;
	}
  }
  return str;
}

inline bool isPlusOrMinus(const string &token) {
	if (token.find_first_of("+-") != string::npos) {
		return true;
	}
	return false;
}

string sendCommand(string command, string response)
{
	BufferedFilterPtr echoFilter = serialListener.createBufferedFilter(SerialListener::exactly(command));
	if(enableLogging) ROS_INFO("Sending command: %s", command.c_str());
	serialPort->write(command+"\r");
	if (echoFilter->wait(50).empty()) {
		ROS_ERROR("Failed to receive an echo from the Roboteq.");
		return "";
	}
	BufferedFilterPtr responseFilter = serialListener.createBufferedFilter(SerialListener::contains(response));
	string result = responseFilter->wait(100);
	return result;
}


int checkEncoderCount(int channel)
{
	string command_result = sendCommand(stringFormat("?CR %i", channel), "CR=");
	string count;
	//The result will always have first 3 characters as 'CR='.
	//So the count we need starts from the 3rd character until the EOL.
	if (command_result.size() > 0) {
		count = command_result.substr(3);
	} else {
		return 0;
	}

	return std::stoi(count);
}

bool sendSpeed(string command)
{
	string result = sendCommand(command, "+");
	if(result[0] != '+'){
		ROS_ERROR("The Roboteq rejected the command (%s). Got: %s", command.c_str(), result.c_str());
		return false;
	}
	return true;
}

int constrainSpeed(int speed){
	if(abs(speed) > 1000){
		if(speed > 0){
			speed = 1000;
		}
		else{
			speed = -1000;
		}		
	}
	return speed;
}

void move(){
	if(!roboteqIsConnected){
		ROS_WARN("The Roboteq needs to be connected to move.");
		return;
	}

	sendSpeed(stringFormat("!G 1 %d", (flip_inputs ? -1 : 1) * constrainSpeed(rightSpeed)));
	sendSpeed(stringFormat("!G 2 %d", (flip_inputs ? -1 : 1) * constrainSpeed(leftSpeed)));
}

int main(int argc, char **argv){
	ros::init(argc, argv, "roboteq_mdc2460");

	ros::NodeHandle n, n_private("~");

	// Serial port parameter
	n_private.param("serial_port", port, std::string("/dev/ttyUSB0"));
	n_private.param("enable_logging", enableLogging, false);
	n_private.param("gear_reduction", gearReduction, 1.0);
	n_private.param("flip_inputs", flip_inputs, false);
	n_private.param("has_encoders", hasEncoder, false);

	ros::Subscriber driveModeSub = n.subscribe("motor_control", 5, driveModeCallback);
	ros::Publisher pub = n.advertise<isc_shared_msgs::EncoderCounts>("encoder_counts", 1000);
	
	isc_shared_msgs::EncoderCounts count;

	ros::Rate loopRate(100); //Hz
	while(ros::ok()) {
		ROS_INFO("Connecting NextGen Roboteq on port %s", port.c_str());
		try {
			connect();
		} catch(std::exception &e) {
			ROS_ERROR("Failed to connect to the Roboteq: %s", e.what());
			disconnect();
		}

		while(roboteqIsConnected && ros::ok()){
			ros::spinOnce();
			move();
			
			if(hasEncoder){
				count.left_count = checkEncoderCount(1) / gearReduction;
				count.right_count = checkEncoderCount(2) / gearReduction;
				pub.publish(count);
			}

			loopRate.sleep();
		}

		if(!ros::ok()) break;
		ROS_INFO("Will try to reconnect to the Roboteq in 5 seconds.");
		for (int i = 0; i < 100; ++i) {
			ros::Duration(5.0/100.0).sleep();
			if (!ros::ok()) break;
		}
	}

	// ros::spin();
	
	return 0;
}
