#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
/*
NavStack make me feel like
⢾⣾⣷⣾⣽⣻⣿⣇⣿⣿⣧⣿⢸⣿⣿⡆⢸⣹⣿⣆⢥⢛⡿⣿⣿⣿⡇
⡓⣉⠉⠙⠻⢿⣿⣿⣟⣻⠿⣹⡏⣿⣿⣧⢸⣧⣿⣿⣨⡟⣿⣿⣿⣿⡇
⣷⣹⣿⠄⠄⠄⠄⠘⢿⣿⣿⣯⣳⣿⣭⣽⢼⣿⣜⣿⣇⣷⡹⣿⣿⣿⠁
⢻⣷⣿⡄⢈⠿⠇⢸⣿⣿⣿⣿⣿⣿⣟⠛⠲⢯⣿⣒⡾⣼⣷⡹⣿⣿⠄
⢸⣿⣿⣷⣬⣽⣯⣾⣿⣿⣿⣿⣿⣿⣿⣿⡀⠄⢀⠉⠙⠛⠛⠳⠽⠿⢠
⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⢄⣹⡿⠃⠄⠄⣰⠎⡈⣾
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣭⣽⣖⣄⣴⣯⣾⢷⣿
⠸⣿⣿⣿⣿⣿⣿⠯⠊⠙⢻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⣾⣿
⣦⠹⣿⣿⣿⣿⣿⠄⢀⣴⢾⣼⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣾⣿⣿
⣿⣇⢽⣿⣿⣿⡏⣿⣿⣿⣿⣿⡇⣿⣿⣿⣿⡿⣿⣛⣻⠿⣟⣼⣿⣿⣿
⣿⣿⡎⣷⣽⠻⣇⣿⣿⣿⡿⣟⣵⣿⣟⣽⣾⣿⣿⣿⣿⢯⣾⣿⣿⣿⠟
⣿⣿⣿⢹⣿⣿⢮⣚⡛⠒⠛⢛⣋⣶⣿⣿⣿⣿⣿⣟⣱⠿⣿⣿⠟⣡
*/


#include <string>
#include <deque>
#include <iostream>
#include <fstream>
#include <vector>
//use a deque instead of a queue to insert at the front for adding points

typedef struct waypoint {geometry_msgs::Pose pose; double tolerance;} waypoint;

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

		std::deque<waypoint> waypoint_queue;//deque for pushing new waypoints to front of designated path
		ros::NodeHandle nh;
		ros::Publisher pose_pub,rotate_pub;
		ros::Subscriber waypoint_sub, map_sub;

		bool running,reached_waypoint;

		double tolerance_x,tolerance_y,tolerance_qz,tolerance_qw;
		double target_x,target_y,target_qz,target_qw;

		int rows,cols;
		std::vector<std::vector<signed char> > snowMap;
	public:
		tf::TransformListener tfListener;
		waypoint_class(std::string);
		void addWaypoint(geometry_msgs::Pose);
		void run();
};

waypoint_class::waypoint_class(std::string filename){
	ros::NodeHandle privateNode("~");
	if(!privateNode.getParam("tolerance_x",tolerance_x)){
		tolerance_x = 0.2;
		std::cout << "MISSING PARAM:Defaulted tolerance_x to " << tolerance_x << '\n';
	}
	if(!privateNode.getParam("tolerance_y",tolerance_y)){
		tolerance_y = 0.2;
		std::cout << "MISSING PARAM:Defaulted tolerance_y to " << tolerance_y << '\n';
	}
	if(!privateNode.getParam("tolerance_qz",tolerance_qz)){
		tolerance_qz = 0.1;
		std::cout << "MISSING PARAM:Defaulted tolerance_qz to " << tolerance_qz << '\n';
	}
	if(!privateNode.getParam("tolerance_qw",tolerance_qw)){
		tolerance_qw = 0.1;
		std::cout << "MISSING PARAM:Defaulted tolerance_qw to " << tolerance_qw << '\n';
	}
	target_x = target_y = target_qz = target_qw = 0;

	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
	rotate_pub = nh.advertise<std_msgs::Bool>("/yeti/rotate",10);
	waypoint_sub = nh.subscribe<geometry_msgs::Pose>("/yeti/new_waypoints",10,&waypoint_class::addWaypointCallback,this);
	map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10, &waypoint_class::mapCallback,this);

	bool running = false;
	reached_waypoint = false;
	if(importWaypoints(filename)<0){
		std::cout << "Error not a valid file\n";
		exit(-1);
	}
}

int waypoint_class::importWaypoints(std::string filename){
	waypoint tempWaypoint;
	int position;
	std::string line,cell;
	tf::Quaternion quarter;
	double roll, pitch, yaw;

	std::ifstream waypointFile(filename, std::ifstream::in);
	std::getline(waypointFile,line); //remove the word line
	std::cout << line << '\n';
	if(waypointFile.is_open()){
		while(std::getline(waypointFile,line)){
			std::stringstream streamLine(line);
			position = 0;
			while(std::getline(streamLine,cell,',')){
				switch(position++){
					case 0:
						tempWaypoint.pose.position.x = std::stof(cell);
						break;
					case 1:
						tempWaypoint.pose.position.y = std::stof(cell);
						break;
					case 2:
						tempWaypoint.pose.position.z = std::stof(cell);
						break;
					case 3:
						roll = std::stof(cell);
						break;
					case 4:
						pitch = std::stof(cell);
						break;
					case 5:
						yaw = std::stof(cell);
						break;
					default:
						quarter.setRPY(roll,pitch,yaw);//could set roll and pitch to 0 since yeti should never be tilting
						tempWaypoint.pose.orientation.x = quarter.getX();
						tempWaypoint.pose.orientation.y = quarter.getY();
						tempWaypoint.pose.orientation.z = quarter.getZ();
						tempWaypoint.pose.orientation.w = quarter.getW();
						tempWaypoint.tolerance = std::stof(cell);
						waypoint_queue.push_back(tempWaypoint);
						break;
				}
			}
		}
		waypointFile.close();
	}
	else{
		std::cout << "Error opening file:" << filename << '\n';
		return -1;
	}
	if(!waypoint_queue.empty()){//print all waypoints to terminal
		running = true;
		geometry_msgs::Pose p;
		for(int loopcount = waypoint_queue.size();loopcount > 0;loopcount--){
			tempWaypoint = waypoint_queue.front();
			waypoint_queue.pop_front();
			printPose(tempWaypoint.pose);
			waypoint_queue.push_back(tempWaypoint);
		}
	}
	return 1;
}

void waypoint_class::publishNextWaypoint(){
	geometry_msgs::PoseStamped msgs;
	std_msgs::Bool rotate;
	std::string cmdString;
	const char * cmd;
	static int waypoint_number = 0;

	msgs.header.frame_id = "map";
	msgs.header.stamp  = ros::Time(0);
	msgs.header.seq = waypoint_number++;
	msgs.pose.position = waypoint_queue.front().pose.position;
	msgs.pose.orientation = waypoint_queue.front().pose.orientation;
	pose_pub.publish(msgs);

	target_x = msgs.pose.position.x;
	target_y = msgs.pose.position.y;
	target_qz = msgs.pose.orientation.z;
	target_qw = msgs.pose.orientation.w;

	
	rotate.data = false;
	if(waypoint_queue.front().tolerance > 0.4)
		rotate.data = true;
	rotate_pub.publish(rotate);
	cmdString = "rosrun dynamic_reconfigure dyniparam set /move_base/DWAPlannerROS xy_goal_tolerance " + std::to_string(waypoint_queue.front().tolerance);//make sure that new waypoint will be published
	rotate.data = true;
	cmd = cmdString.c_str();
	system(cmd);
	tolerance_x = tolerance_y = waypoint_queue.front().tolerance + 0.1;//tolerance should be higher than planner tolerance


	std::cout << "----New Goal Set----\n";
	printPose(waypoint_queue.front().pose);
	//waypoint_queue.pop_front();//Moved to checkpose because allows for adding points without losing the desired final position
}

void waypoint_class::checkPose(){
	tf::StampedTransform transform;
	geometry_msgs::Pose p;
	static double timer = ros::Time::now().toSec();
	
	//This when to re-eval the path every 5 secs but it was fixed by making the map update
	/*if(timer + 5 < ros::Time::now().toSec()){
		this->publishNextWaypoint();
		timer = ros::Time::now().toSec();
	}*/
	

	try{
		tfListener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("WAYPOINT_PUBLISHER: %s",ex.what());
		ros::Duration(1.0).sleep();
		return;
	}


	tfToPose(transform,p);//fixes negative w
	//std::cout << "\nCurrentTf/GoalTf\n";printPose(p);printPose(waypoint_queue.front().pose);

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
	waypoint tempWaypoint;
	tempWaypoint.pose = msgs;
	tempWaypoint.tolerance = tolerance_x;//default value for tolerance will be default waypoint tolerance
	waypoint_queue.push_front(tempWaypoint);
	this->publishNextWaypoint();
	//maybe some additional logic or rearrangement of this function
}

void waypoint_class::addWaypointCallback(const geometry_msgs::Pose msgs){
	addWaypoint(msgs);
}

void waypoint_class::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgs){
//Manual Recreation/skipping of waypoints if path is determined to have waypoint inside of collision area
/*std::cout << "------------\n" << "Resolution: " << msgs->info.resolution << " \n"
		<< "Width: " << msgs->info.width << " \n"
		<< "Height: " << msgs->info.height << " \n"
		<< "X: " << msgs->info.origin.position.x << " \n"
		<< "Y: " << msgs->info.origin.position.y << " \n";*/
	static bool init = true;
	//std::cout << "Start Time:" << ros::Time::now() << "\n";

	if(init){//Make Matrix form holding map data
		//Hard coded because math is off becuase 
		rows = 300;//15.0 / msgs->info.resolution;
		cols = 80;//4.0 / msgs->info.resolution;
		snowMap.resize(rows);
		for(int i = 0; i< rows; i++)
			snowMap[i].resize(cols);
		init = false;
		std::cout << "Width: " << cols << "Height: " << rows << '\n'; 
	}
	
	//rows is x cols is y
	/*
	Map of the Matrix
	---------------------
	|x,y                |
	|                   |
	|-------------------|
	|      |     |      |
	|      |     |      |
	|      |     |      |
	|      |     |      |
	|      |     |      |
	|      |     |      |
	|-------------------|
	|                   |
	|       Start       |
	|                0,0|
	---------------------
	*/
	//std::cout << "----OCCUPANCY GRID-----\n";
	for(int x = 0; x< rows; x++) {
		for(int y = 0; y < cols; y++) { 
				snowMap[x][y] = (signed char) msgs->data[4000*(y+1960)+(x+1970)];
				//std::cout << (signed int)snowMap[x][y] << " "; 
		}
		//std::cout << "\n";
	}

	//std::cout << "Stop Time:" << ros::Time::now() << "\n";

/*
//run new waypoint
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

void waypoint_class::run(){
	ros::Rate r(10);//10hz
	ros::Duration(10).sleep();//Delay for Sub/Pub to Initalize
	this->publishNextWaypoint();
	while(ros::ok() && this->running){
		ros::spinOnce();
		this->checkPose();
		r.sleep();
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Waypoint_node");
	ros::NodeHandle privateNode("~");
	std::string filename;
	if(privateNode.getParam("filename",filename)){
		waypoint_class client(filename);
		//wait for the Map to be read before sending the waypoints
		while(ros::ok() && !client.tfListener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0)))
			std::cout << "WaypointGen: Waiting for /Map\n";
		client.run();
	}
	else{
		ROS_ERROR("Failed to get Param \'filename\' for waypoint file. %s",filename.c_str());
		exit(-1);
	}
}

