#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>

#include "pure_pursuit.h"

#define DEFAULT_ANGULAR_VEL_TUNE 0.5
#define DEFAULT_LOOKAHEAD_DISTANCE 2
#define DEFAULT_PUBLISH_HEADING_TO_POINT true
#define DEFAULT_PUBLISH_HEADING_ERROR true
#define DEFAULT_PUBLISH_LOOKAHEAD_POINT false

#define PATH_AND_POINT_DIST_FROM_GROUND 0.3

class PurePursuitRos{
public:
    PurePursuitRos() : m_tracker(m_path, m_lookahead_distance), m_path_is_initialized(false)
    {
        n.param<double>("lookahead_distance", m_lookahead_distance, DEFAULT_LOOKAHEAD_DISTANCE);
        n.param<bool>("publish_heading_to_point", m_publish_heading_to_point, DEFAULT_PUBLISH_HEADING_TO_POINT);
        n.param<bool>("publish_heading_error", m_publish_heading_error, DEFAULT_PUBLISH_HEADING_ERROR);
        n.param<bool>("publish_lookahead", m_publish_lookahead_point, DEFAULT_PUBLISH_LOOKAHEAD_POINT);
        m_tracker.reset_lookahead_distance(m_lookahead_distance);
        m_vel_pub = n.advertise<std_msgs::Float64>("linear_velocity_setpoint", 1);
        m_ang_vel_pub = n.advertise<std_msgs::Float64>("angular_velocity_setpoint", 1);
        m_path_sub = n.subscribe("path", 1, &PurePursuitRos::receive_path, this);
        if(m_publish_heading_to_point) {
            //m_head_to_point_pub = n.advertise<std_msgs::Float64>("pure_pursuit/heading_to_point", 1);
            m_head_to_point_pub = n.advertise<std_msgs::Float64>("/rotation_setpoint", 1);
        }
        if(m_publish_heading_error){
            m_head_error_pub = n.advertise<std_msgs::Float64>("pure_pursuit/heading_error", 1);
        }
        if(m_publish_lookahead_point){
            m_lookahead_pub = n.advertise<geometry_msgs::PointStamped>("pure_pursuit/lookahead_point", 1);
        }

    }



    void process()
    {
        if(m_path_is_initialized) {
            Point3D lookahead;
            std_msgs::Float64 msg;
            double heading_to_point, heading_error;
            std::tie (lookahead, heading_to_point, heading_error) = m_tracker.get_target_state(get_pose());
            msg.data = lookahead.z;
            m_vel_pub.publish(msg);

            double ang_vel = chop(DEFAULT_ANGULAR_VEL_TUNE * heading_error, -M_PI_4, M_PI_4);
            msg.data = ang_vel;
            //m_ang_vel_pub.publish(msg);

            if(m_publish_heading_to_point) {
                msg.data = heading_to_point;
                m_head_to_point_pub.publish(msg);
            }
            if(m_publish_heading_error) {
                msg.data = heading_error;
                m_head_error_pub.publish(msg);
            }
            if(m_publish_lookahead_point){
                geometry_msgs::PointStamped lookahead_point;
                lookahead_point.header.frame_id="map";
                lookahead_point.header.stamp = ros::Time::now();
                lookahead_point.point.x = lookahead.x;
                lookahead_point.point.y = lookahead.y;
                lookahead_point.point.z = PATH_AND_POINT_DIST_FROM_GROUND;
                m_lookahead_pub.publish(lookahead_point);
            }

        }
    }

private:

    double chop(double x, double x_min, double x_max){
        if(x < x_min)
        {
            return x_min;
        }
        else if(x > x_max)
        {
            return x_max;
        }
        else{
            return x;
        }
    }

    void receive_path(const nav_msgs::Path::ConstPtr& path)
    {
        m_path = to_path(*path);
        m_tracker.reset_path(m_path);
        if(!m_path_is_initialized) {
            ROS_INFO("Got Path!");
            m_path_is_initialized = true;
        }
    }

    Path to_path(const nav_msgs::Path& path)
    {
        Path converted;
        for(const auto& p: path.poses)
        {
            converted.push_back(to_point3d(p));
        }
        return converted;
    }

    Point3D to_point3d(const geometry_msgs::PoseStamped& ps)
    {
        return Point3D(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
    }

    Point3D get_pose(){
        tf::StampedTransform transform;
        try{
            m_tf_listener.lookupTransform("/map", "/base_link",
                                     ros::Time(0), transform);

        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        Point3D pose;
        pose.x = transform.getOrigin().x();
        pose.y = transform.getOrigin().y();
        tf::Matrix3x3 mat(transform.getRotation());
        double r,p,y;
        mat.getRPY(r,p,y);
        pose.z = y;
        return pose;
    }

    ros::NodeHandle n;
    ros::Publisher m_vel_pub;
    ros::Publisher m_ang_vel_pub;
    ros::Publisher m_head_to_point_pub;
    ros::Publisher m_head_error_pub;
    ros::Publisher m_lookahead_pub;
    ros::Subscriber m_path_sub;
    tf::TransformListener m_tf_listener;
    Path m_path;
    PurePursuit m_tracker;
    double m_lookahead_distance;
    bool m_path_is_initialized;
    bool m_publish_heading_error;
    bool m_publish_lookahead_point;
    bool m_publish_heading_to_point;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");

    PurePursuitRos pure_pursuit;

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        pure_pursuit.process();

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
