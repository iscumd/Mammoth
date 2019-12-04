#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <deque>
#include <iostream>
#include <fstream>
#include <vector>
//use a deque instead of a queue to insert at the front for adding points

class waypoint_class{
	private:
		waypoint_class();//private null constructor
		int importWaypoints(std::string);
		void publishNextWaypoint();
		void checkPose();
		void addWaypointCallback(const geometry_msgs::Pose);
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
		void printPose(geometry_msgs::Pose);
		void tfToPose(tf::StampedTransform,geometry_msgs::Pose&);

		tf::TransformListener tfListener;
		std::deque<geometry_msgs::Pose> waypoint_queue;//deque for pushing new waypoints to front of designated path
		ros::NodeHandle nh;
		ros::Publisher pose_pub;
		ros::Subscriber waypoint_sub, map_sub;

		bool running,reached_waypoint;

		double tolerance_x,tolerance_y,
			tolerance_qz,tolerance_qw;
		double target_x,target_y,target_qz,target_qw;


		int waypoint_number,rows,cols;
		std::vector<std::vector<bool> > grid;
	public:
		waypoint_class(std::string);
		void addWaypoint(geometry_msgs::Pose);
		int status();
		int run();
};

waypoint_class::waypoint_class(std::string filename){
	tolerance_x = 0.25;//25cm
	tolerance_y = 0.25;

	tolerance_qz = 0.1;//~5 Degree
	tolerance_qw = 0.1;

	target_x = target_y = target_qz = target_qw = 0;

	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
	waypoint_sub = nh.subscribe<geometry_msgs::Pose>("/yeti/new_waypoints",10,&waypoint_class::addWaypointCallback,this);
	map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10, &waypoint_class::mapCallback,this);
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
						waypoint_queue.push_back(pose);
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
	if(!waypoint_queue.empty()){//print all waypoints to terminal
		running = true;
		geometry_msgs::Pose p;
		for(int loopcount = waypoint_queue.size();loopcount > 0;loopcount--){
			p = waypoint_queue.front();
			waypoint_queue.pop_front();
			printPose(p);
			waypoint_queue.push_back(p);
		}
		
	}
	return 1;
}

void waypoint_class::publishNextWaypoint(){
	geometry_msgs::PoseStamped msgs;
	msgs.header.frame_id = "map";
	msgs.header.stamp  = ros::Time(0);
	msgs.header.seq = waypoint_number;	
	msgs.pose.position = waypoint_queue.front().position;
	msgs.pose.orientation = waypoint_queue.front().orientation;
	pose_pub.publish(msgs);

	target_x = msgs.pose.position.x;
	target_y = msgs.pose.position.y;
	target_qz = msgs.pose.orientation.z;
	target_qw = msgs.pose.orientation.w;

	std::cout << "----New Goal Set----\n";
	printPose(waypoint_queue.front());
	//waypoint_queue.pop_front();//Moved to checkpose because allows for adding points without losing the desired final position
}

void waypoint_class::checkPose(){
	tf::StampedTransform transform;
	geometry_msgs::Pose p;

	try{
		tfListener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		return;
		}


	tfToPose(transform,p);//fixes negative w
	std::cout << "\nTransform\n";printPose(p);printPose(waypoint_queue.front());

	if(std::abs( target_x - p.position.x) < tolerance_x)
		if(std::abs( target_y - p.position.y) < tolerance_y)
			if(std::abs( target_qz - p.orientation.z) < tolerance_qz)
				if(std::abs( target_qw - p.orientation.w) < tolerance_qw)
					reached_waypoint = true;

	if(reached_waypoint){
		std::cout << "Reached Target Waypoint.\n";
		reached_waypoint = false;
		waypoint_queue.pop_front(); //pop moved here so that yeti will retain the original goal if a new goal is added from a callback
		if(waypoint_queue.empty()){
			running = false;
			std::cout << "All Waypoints Completed\n";
		}
		else
			this->publishNextWaypoint();
	}
}

void waypoint_class::addWaypoint(geometry_msgs::Pose msgs){
	waypoint_queue.push_front(msgs);
	this->publishNextWaypoint();
	//maybe some additional logic or rearrangement of this function
}

void waypoint_class::addWaypointCallback(const geometry_msgs::Pose msgs){
	addWaypoint(msgs);
}

void waypoint_class::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgs){
/*Manual Recreation/skipping of waypoints if path is determined to have waypoint inside of collision area*/
/*	std::cout << "------------\n" << "Resolution: " << msgs->info.resolution << " \n"
		<< "Width: " << msgs->info.width << " \n"
		<< "Height: " << msgs->info.height << " \n"
		<< "X: " << msgs->info.origin.position.x << " \n"
		<< "Y: " << msgs->info.origin.position.y << " \n";
	static bool init = true;
	std::cout << ros::Time::now() << "\n";

	if(init){//Make Matrix form holding map data
		rows = msgs->info.height;
		cols = msgs->info.width;
		grid.resize(rows);
		for(int i = 0; i< rows; i++)
			grid[i].resize(cols);
		init = false;
	}

	int currCell = 0;
	for(int i = 0; i< rows; i++) {
		for(int j = 0; j < cols; j++) {
			if(msgs->data[currCell++] == 0) 
				grid[i][j] = false;
			else
				grid[i][j] = true; 
		}
	}

	std::cout << ros::Time::now() << "\n";
*/


/*
//run next waypoint
waypoint_queue.pop_front();
this->publishNextWaypoint();
*/


}

void waypoint_class::printPose(geometry_msgs::Pose p){
	std::cout << "X:    Y:    Z:    QX:   QY:   QZ:   QW:   \n";
	printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
}

void waypoint_class::tfToPose(tf::StampedTransform t,geometry_msgs::Pose& p){
	tf::pointTFToMsg(t.getOrigin(),p.position);
	tf::quaternionTFToMsg(t.getRotation(),p.orientation);
	if(p.orientation.w <0 ){//inverse negative w
		p.orientation.w *= -1;
		p.orientation.z *= -1;
	}
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

