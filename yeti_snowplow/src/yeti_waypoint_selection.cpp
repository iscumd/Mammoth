#include "ros/ros.h"
#include "yeti_snowplow/location_point.h"
#include "yeti_snowplow/target.h"
#include "yeti_snowplow/waypoint.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
using namespace std;

vector<yeti_snowplow::target> targetLocationList;

//because C++ doesn't have built in string splitting http://stackoverflow.com/a/236803
void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


vector<yeti_snowplow::target> ReadFile(string filename){
	string str;
	ifstream file;
	vector<yeti_snowplow::target> navigationPoints = vector<yeti_snowplow::target>();
	int pointCount = 0;

	file.open(filename.c_str());
	while(getline(file, str)){
		ROS_DEBUG("Read file line: %s", str.c_str());
		vector<string> lineFields = split(str, ' '); //x y direction PID speed
		if(lineFields.size() == 5){ //ignore if too short or starts with "// "
			yeti_snowplow::target currentLidarPoint;
			currentLidarPoint.location.x = atof(lineFields[0].c_str());
			currentLidarPoint.location.y = atof(lineFields[1].c_str());
			currentLidarPoint.location.heading = 0.0;
			currentLidarPoint.dir = atoi(lineFields[2].c_str());
			currentLidarPoint.PID = (atoi(lineFields[3].c_str()) > 0);
			currentLidarPoint.speed = atof(lineFields[4].c_str());
			currentLidarPoint.location.id = pointCount;
			pointCount++;

			navigationPoints.push_back(currentLidarPoint);
		}
	}
	file.close();

	return navigationPoints;
}

bool waypoint(yeti_snowplow::waypoint::Request  &req,
              yeti_snowplow::waypoint::Response &res){
	ROS_INFO("Received request: %i", req.ID);
	if (req.ID > -1 && req.ID < targetLocationList.size()){
		res.waypoint = targetLocationList[req.ID];
		ROS_INFO("Sent response: x=%f y=%f heading=%f dir=%i PID=%s speed=%f", res.waypoint.location.x, res.waypoint.location.y, res.waypoint.location.heading, res.waypoint.dir, res.waypoint.PID?"true":"false", res.waypoint.speed);
		return true;
	}
	else {
		return false;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "yeti_waypoint_selection");

	ros::NodeHandle n;
	
	std::string navigationFile;
	if (ros::param::get("navigationFile", navigationFile)){
		ROS_INFO("Using navigationFile %s", navigationFile.c_str());

		targetLocationList = ReadFile(navigationFile);

		for(int i = 0; i < targetLocationList.size(); i++){
			ROS_INFO("Target #%i: %f %f %i %i %f", targetLocationList[i].location.id, targetLocationList[i].location.x, targetLocationList[i].location.y, targetLocationList[i].dir, targetLocationList[i].PID, targetLocationList[i].speed);
		}

		ros::ServiceServer service = n.advertiseService("waypoint", waypoint);

		ros::spin();
	}
	else{
		ROS_FATAL("No navigationFile parameter specified!");
	}
	
	return 0;
}