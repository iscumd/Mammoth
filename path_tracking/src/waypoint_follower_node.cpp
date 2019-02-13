#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <math.h>

struct WayPoint
{
    float x, y;
};

std::vector<WayPoint> waypoints;
ros::Publisher vel_pub;
ros::Publisher rot_pub;

float ang_diff(float th1, float th2)
{
    return fmod(((th1 - th2) + 3.0f * M_PI), (2.0f * M_PI)) - M_PI;
}

float distance(float x1, float y1, float x2, float y2)
{
    return std::sqrt(std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2));
}

float chop(float x, float xmin, float xmax)
{
    if (x < xmin)
    {
        return xmin;
    }
    else if (x > xmax)
    {
        return xmax;
    }
    else
    {
        return x;
    }
}

void PoseTrackerCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    //ROS_INFO("I heard: [%f]", msg->x);
    float linear_tune = .5;
    float angular_tune = 2;
    static int curr_waypoint = 0;
    float dx = waypoints[curr_waypoint].x - msg->x;
    float dy = waypoints[curr_waypoint].y - msg->y;
    float heading_th = atan2(dy, dx);
    float ang_err = ang_diff(heading_th, msg->theta);
    std::cout << ang_err << std::endl;
    float w = chop(angular_tune * ang_err, -5, 5);
    float v = chop(1.0 / (linear_tune * abs(ang_err) + .000000001), 0.0, 0.5);
    // w = 10;
    // float v = chop(0.1 + distance(waypoints[curr_waypoint].x, waypoints[curr_waypoint].y, msg->x,msg->y), 0.0, 10);
    // geometry_msgs::Twist vel;
    std_msgs::Float64 vel, rot, zero;
    vel.data = v;
    rot.data = heading_th;
    zero.data = 0;
    // vel.linear.x = v;
    // vel.angular.z = w;
    if (distance(waypoints[curr_waypoint].x,  waypoints[curr_waypoint].y, msg->x, msg->y) > .5)
    {
        vel_pub.publish(vel);
        rot_pub.publish(rot);
    }
    else
    {
      vel_pub.publish(zero);
      rot_pub.publish(zero);
        if(curr_waypoint < waypoints.size()-1)
        {
                curr_waypoint++;
        }

    }
}

int main(int argc, char **argv)
{
    waypoints.push_back({6, -0.5});
    waypoints.push_back({6, 0.5});
    // waypoints.push_back({-2, 0});
    waypoints.push_back({0, 0});
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    vel_pub = n.advertise<std_msgs::Float64>("linear_velocity_setpoint", 1);
    rot_pub = n.advertise<std_msgs::Float64>("rotation_setpoint", 1);

    ros::Subscriber pose_sub = n.subscribe("yeti/pose", 1, PoseTrackerCallback);

    ros::spin();

    return 0;
}
