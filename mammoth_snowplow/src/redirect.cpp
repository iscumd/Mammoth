#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

bool rotate;
ros::Publisher cmd_out;
ros::Subscriber cmd_in, rotate_callback;

void subCallback(const std_msgs::Bool::ConstPtr msgs){
	rotate = msgs->data;
}

void subVelocity(const geometry_msgs::Twist::ConstPtr msgs){
	geometry_msgs::Twist mod = *msgs;
	//if(rotate)mod.linear.x = 0;
	//mod.angular.z *=2;
	cmd_out.publish(mod);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Velocity Node");
	ros::NodeHandle nh;
	
	rotate = false;
	
	cmd_out = nh.advertise<geometry_msgs::Twist>("/yeti/actuation/control2",10);
	cmd_in = nh.subscribe("/yeti/actuation/control",10,subVelocity);
	rotate_callback = nh.subscribe("/yeti/rotate",1,subCallback);
	//ros::spin();

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Transform transform2;
	transform.setOrigin( tf::Vector3(0.29, 0, 0.76));
	transform2.setOrigin( tf::Vector3(-0.364, -0.02, -0.63));
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	transform2.setRotation(q);
	ros::Rate r(200);
	while(ros::ok()){
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "os1_sensor"));
		br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "camera_pose_frame", "base_link"));
		ros::spinOnce();
		r.sleep();
	}
}

