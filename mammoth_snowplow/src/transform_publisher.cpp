#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "Static Transform Node");
	ros::NodeHandle nh;

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

