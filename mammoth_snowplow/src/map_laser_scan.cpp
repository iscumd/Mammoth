#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "map_point_cloud");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("/os1_cloud_node/scan2", 50);

	int num_points = 1024;
	tf::StampedTransform transform;
	double ignore,yaw;
	const double PI = atan(1) * 4.0;
	const double farX = 13.5+0.7, closeX = -1.5-0.7, leftY = 2.0+0.7, rightY = -2.0-0.7;
	double stepInRad = PI * 2.0/((double)num_points);
	double angle,x,y;
	tf::TransformListener tfListener;
	

	sensor_msgs::LaserScan cloud;
	cloud.header.frame_id = "/os1_lidar";
	cloud.angle_min = -3.14f;
	cloud.angle_max = 3.14f;
	cloud.angle_increment = 2*PI/num_points;
	cloud.time_increment = 0;
	cloud.scan_time = 0.001f;
	cloud.range_min = 0.2f;
	cloud.range_max = 24;
	cloud.ranges.resize(num_points);

	ros::Rate r(10);
	while(ros::ok()){   
	try{
			tfListener.lookupTransform("/map", "/os1_lidar", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ros::Duration(0.1).sleep();
			continue;
		}
		tf::Matrix3x3(transform.getRotation()).getRPY(ignore,ignore,yaw);
		x = transform.getOrigin().getX();
		y = transform.getOrigin().getY();

		//generate Boarder of the I Field
		yaw += PI;//correct for scan start location
		for(int i = 0; i < num_points; i++){
			angle = yaw + stepInRad*i;
			angle = std::fmod(angle,2*PI)-PI;
			if(angle > PI/2.0){
				cloud.ranges[i] = std::min(std::abs(((leftY-y)/sin(angle))),std::abs(((x-closeX)/cos(angle))));
			}
			else if(angle >= 0){
				cloud.ranges[i] = std::min(std::abs(((leftY-y)/sin(angle))),std::abs(((farX-x)/cos(angle))));
			}
			else if (angle > PI/(-2.0)){
				cloud.ranges[i] = std::min(std::abs(((y-rightY)/sin(angle))),std::abs(((farX-x)/cos(angle))));
			    
			}
			else{
				
				cloud.ranges[i] = std::min(std::abs(((y-rightY)/sin(angle))),std::abs(((x-closeX)/cos(angle))));
				
			}
			
		}
		
		cloud.header.stamp = ros::Time::now();
		pub.publish(cloud);
		r.sleep();
	}
}
