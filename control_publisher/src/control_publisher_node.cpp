#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

geometry_msgs::Twist control_effort;

void linear_control(const std_msgs::Float64::ConstPtr &linear_msg) {
    control_effort.linear.x = linear_msg->data;
}

void angular_control(const std_msgs::Float64::ConstPtr &angular_msg) {
    control_effort.angular.z = angular_msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_publisher");

    ros::NodeHandle n;

    ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("/auto_control", 1);
    control_effort.linear.x = 0;
    control_effort.angular.z = 0;

    ros::Subscriber linearControlSub = n.subscribe("linear_control_effort", 1, linear_control);
    ros::Subscriber angularControlSub = n.subscribe("angular_velocity_control_effort", 1, angular_control);

    ros::Rate rate(10.0);

    while (n.ok()) {

        control_pub.publish(control_effort);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

};
