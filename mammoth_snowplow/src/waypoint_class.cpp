#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <queue>
#include <iostream>
#include <fstream>

class waypoint_class{
	private:
		waypoint_class();//private null constructor
		int importWaypoints(std::string);
		void publishNextWaypoint();
		void checkPose();
		void addWaypointCallback(const geometry_msgs::Pose);
		
		tf::TransformListener tfListener;
		std::queue<geometry_msgs::Pose> waypoint_queue;//maybe make this a priority queue later 
		ros::NodeHandle nh;
		ros::Publisher pose_pub;
		ros::Subscriber waypoint_sub;
		bool running,reached_waypoint;
		float/*double*/tolerance_x,tolerance_y,tolerance_z,
			tolerance_qx,tolerance_qy,tolerance_qz,tolerance_qw,
			tolerance_roll,tolerance_pitch,tolerance_yaw;
		double current_x,current_y,current_z,current_roll,current_pitch,current_yaw;
		int waypoint_number;
	public:
		waypoint_class(std::string);
		void addWaypoint(geometry_msgs::Pose);
		int status();
		int run();
};

waypoint_class::waypoint_class(std::string filename){
	tolerance_x = 0.4;
	tolerance_y = 0.4;
	tolerance_z = 1;
	tolerance_qx = 0.1;
	tolerance_qy = 0.1;
	tolerance_qz = 0.1;
	tolerance_qw = 0.1;
	tolerance_roll = 0.15;
	tolerance_pitch = 0.15;
	tolerance_yaw = 0.15;
	current_x = current_y = current_z = current_roll = current_pitch = current_yaw = 0;
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
	waypoint_sub = nh.subscribe<geometry_msgs::Pose>("/yeti/new_waypoints",0,&waypoint_class::addWaypointCallback,this);
	bool running = false;
	waypoint_number = 0;
	reached_waypoint = false;
	if(importWaypoints(filename)<0){
		std::cout << "Error not a valid file\n";
		exit(-1);
	}
}

int waypoint_class::importWaypoints(std::string filename){
	geometry_msgs::Pose pose;
	int position;
	std::string line,cell;
	tf::Quaternion quarter;
	double roll, pitch, yaw;

	std::ifstream waypointfile(filename, std::ifstream::in);
	std::getline(waypointfile,line); //remove the word line
	std::cout << line << '\n';
	if(waypointfile.is_open()){
		while(std::getline(waypointfile,line)){
			std::stringstream streamLine(line);
			position = 0;	
			while(std::getline(streamLine,cell,',')){
				switch(position++){
					case 0:
						pose.position.x = std::stof(cell);
						break;
					case 1:
						pose.position.y = std::stof(cell);
						break;
					case 2:
						pose.position.z = std::stof(cell);
						break;
					case 3:
						roll = std::stof(cell);
						break;
					case 4:
						pitch = std::stof(cell);
						break;
					default:
						yaw = std::stof(cell);
						quarter.setRPY(roll,pitch,yaw);
						pose.orientation.x = quarter.getX();
						pose.orientation.y = quarter.getY();
						pose.orientation.z = quarter.getZ();
						pose.orientation.w = quarter.getW();
						waypoint_queue.push(pose);
						break;
				}
			}
		}
		waypointfile.close();
	}
	else{
		std::cout << "Error opening file:" << filename << '\n';
		return -1;
	}
	if(!waypoint_queue.empty()){
		running = true;
		geometry_msgs::Pose p;
		for(int loopcount = waypoint_queue.size();loopcount > 0;loopcount--){
			p = waypoint_queue.front();
			waypoint_queue.pop();
			printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
			waypoint_queue.push(p);
		}
		
	}
	return 1;
}

void waypoint_class::publishNextWaypoint(){
	geometry_msgs::PoseStamped msgs;
	msgs.header.frame_id = "map";
	msgs.header.stamp  = ros::Time(0);
	msgs.header.seq = waypoint_number;	
	msgs.pose.position.x = waypoint_queue.front().position.x;
	msgs.pose.position.y = waypoint_queue.front().position.y;
	msgs.pose.position.z = waypoint_queue.front().position.z;
	msgs.pose.orientation.x = waypoint_queue.front().orientation.x;
	msgs.pose.orientation.y = waypoint_queue.front().orientation.y;
	msgs.pose.orientation.z = waypoint_queue.front().orientation.z;
	msgs.pose.orientation.w = waypoint_queue.front().orientation.w;
	//ROS_INFO(msgs);
	
	tf::Quaternion quarter(msgs.pose.orientation.x,msgs.pose.orientation.y,msgs.pose.orientation.z,msgs.pose.orientation.w);
	current_x = msgs.pose.position.x;
	current_y = msgs.pose.position.y;
	current_z = msgs.pose.position.z;
	tf::Matrix3x3(quarter).getRPY(current_roll,current_pitch,current_yaw );
	pose_pub.publish(msgs);
	waypoint_queue.pop();
}

void waypoint_class::checkPose(){
	double roll,pitch,yaw;
	tf::StampedTransform transform;
	try{
		tfListener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		return;
		}


	transform.getBasis().getRPY(roll,pitch,yaw); //convert quaterian to rpy	
	if(abs(transform.getOrigin().getX() - current_x) < tolerance_x)
		if(abs(transform.getOrigin().getY() - current_y) < tolerance_y)
			if(abs(transform.getOrigin().getZ() - current_z) < tolerance_z)
				if(abs(roll - current_roll) < tolerance_roll)
					if(abs(pitch - current_pitch) < tolerance_pitch)
						if(abs(yaw - current_yaw) < tolerance_yaw)
						{
							reached_waypoint = true;
							std::cout << "reached waypoint True\n";
						}
	if(reached_waypoint){
		std::cout << "Reached Target Waypoint.\n";
		reached_waypoint = false;
		if(waypoint_queue.empty()){
			running = false;
			std::cout << "All Waypoints Completed\n";
		}
		else
			this->publishNextWaypoint();
	}
}

void waypoint_class::addWaypoint(geometry_msgs::Pose msgs){
//////////////////////////
}

void waypoint_class::addWaypointCallback(const geometry_msgs::Pose msgs){
	addWaypoint(msgs);
}

int waypoint_class::status(){
	return (int) running;//chagne later
}

int waypoint_class::run(){
	ros::Rate r(10);//10hz
	ros::Duration(1).sleep();//Delay for Sub/Pub to init
	this->publishNextWaypoint();
	while(ros::ok() && this->running){
		ros::spinOnce();
		this->checkPose();
		r.sleep();
	}
	return 1;//Status of how path was. ie missed nodes and terminated
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Waypoint_node");
	ros::NodeHandle privateNode("~");
  	std::string filename;
	if(privateNode.getParam("filename",filename)){
		waypoint_class client(filename);
		client.run();
	}
	else{
		ROS_ERROR("Failed to get Param \'filename\' for waypoint file. %s",filename.c_str());
		exit(-1);		
	}	
}

