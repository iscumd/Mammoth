#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/LinearMath/Matrix3x3.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  tf::TransformListener listener;
  ros::Publisher pose_pub = node.advertise<geometry_msgs::Pose2D>("/yeti/pose", 10);
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link",
                               ros::Time(0), transform);
                               
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    std::cout << transform.getOrigin().x() << ", " << transform.getOrigin().y() << std::endl;
    geometry_msgs::Pose2D pose;
    pose.x = transform.getOrigin().x();
    pose.y = transform.getOrigin().y();
    tf::Matrix3x3 mat(transform.getRotation());
    double r,p,y;
    mat.getRPY(r,p,y);
    pose.theta = y;
    std::cout << r << " " << p << " " << y << std::endl;
    pose_pub.publish(pose);
    rate.sleep();
  }
  return 0;
};
