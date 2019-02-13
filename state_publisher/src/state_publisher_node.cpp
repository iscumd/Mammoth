#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cmath>

ros::Publisher linearPub;
ros::Publisher angularPub;
ros::Publisher anglePub;

geometry_msgs::Twist control_effort;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // std::cout << (double)msg->twist.twist.linear.x << " " << (double)msg->twist.twist.angular.z << std::endl;
  // ROS_INFO("linear: [%f], angular[%f]", (double)msg->twist.twist.linear.x, (double)msg->twist.twist.angular.z);
  std_msgs::Float64 linear, angular;
	linear.data = msg->twist.twist.linear.x;
	double lin_vel = std::sqrt((msg->twist.twist.linear.x*msg->twist.twist.linear.x + msg->twist.twist.linear.y*msg->twist.twist.linear.y));
  angular.data = msg->twist.twist.angular.z;
	linearPub.publish(lin_vel);
  angularPub.publish(angular);
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    std_msgs::Float64 angle;
    angle.data = msg->theta;
    anglePub.publish(angle);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "state_publisher");

  ros::NodeHandle n;

  linearPub = n.advertise<std_msgs::Float64>("linear_velocity", 1);
  angularPub = n.advertise<std_msgs::Float64>("angular_velocity", 1);
  anglePub = n.advertise<std_msgs::Float64>("theta_rot",1);

  ros::Subscriber odomSub = n.subscribe("/lvt/odometry", 1, odomCallback);
  ros::Subscriber poseSub = n.subscribe("yeti/pose",1,poseCallback);

  ros::spin();

  return 0;

};
