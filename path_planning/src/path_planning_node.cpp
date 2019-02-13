//
// Created by aaron on 08/01/19.
//

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <vector>
#include <geometry_msgs/Pose2D.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/src/Geometry/ParametrizedLine.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define SET_SPEED 0.5
#define OBSTACLE_BUFFER 1.2
#define SAMPLE_STEP_SIZE 0.1
#define GOAL_DISTANCE_ACCEPTENCE 0.8

#define LEFT 1
#define RIGHT 0

using geometry_msgs::PoseStamped;

struct Point2D
{
    double x;
    double y;
    Point2D(){}
    Point2D(double x, double y)
    {
        this->x = x;
        this->y = y;
    }
};

PoseStamped point(Point2D p, double speed) {
    PoseStamped p1;
    p1.header.stamp = ros::Time::now();
    p1.header.frame_id = "map";
    p1.pose.position.x = p.x;
    p1.pose.position.y = p.y;
    p1.pose.position.z = speed;
    return p1;
}

typedef std::pair<Point2D, Point2D> Segment2D;

std::pair<Point2D, double> project_to_line_segment( Point2D p, Segment2D seg )
{
    /* This implementation is a slightly modified version of the function from :
     * http://forums.codeguru.com/showthread.php?194400-Distance-between-point-and-line-segment
     */
    double cx            = p.x;
    double cy            = p.y;
    double ax            = seg.first.x;
    double ay            = seg.first.y;
    double bx            = seg.second.x;
    double by            = seg.second.y;
    double r_numerator   = ( cx - ax ) * ( bx - ax ) + ( cy - ay ) * ( by - ay );
    double r_denomenator = ( bx - ax ) * ( bx - ax ) + ( by - ay ) * ( by - ay );
    double r             = r_numerator / r_denomenator;

    double px = ax + r * ( bx - ax );
    double py = ay + r * ( by - ay );

    double s = ( ( ay - cy ) * ( bx - ax ) - ( ax - cx ) * ( by - ay ) ) / r_denomenator;

    double distanceLine    = fabs( s ) * sqrt( r_denomenator );
    double distanceSegment = -1;
    //
    // (xx,yy) is the point on the lineSegment closest to (cx,cy)
    //
    double xx = px;
    double yy = py;

    if ( ( r >= 0 ) && ( r <= 1 ) )
    {
        distanceSegment = distanceLine;
    }
    else
    {

        double dist1 = ( cx - ax ) * ( cx - ax ) + ( cy - ay ) * ( cy - ay );
        double dist2 = ( cx - bx ) * ( cx - bx ) + ( cy - by ) * ( cy - by );
        if ( dist1 < dist2 )
        {
            xx              = ax;
            yy              = ay;
            distanceSegment = sqrt( dist1 );
        }
        else
        {
            xx              = bx;
            yy              = by;
            distanceSegment = sqrt( dist2 );
        }
    }

    return std::make_pair( Point2D( xx, yy ), distanceSegment );
}

double
distance( const Point2D& point1,
          const Point2D& point2 )
{
    return std::sqrt( std::pow( ( point2.x - point1.x ), 2 )
                      + std::pow( ( point2.y - point1.y ), 2 ) );
}

double
distance( const geometry_msgs::Pose2D& point1,
          const Point2D& point2 )
{
    return std::sqrt( std::pow( ( point2.x - point1.x ), 2 )
                      + std::pow( ( point2.y - point1.y ), 2 ) );
}

class PathPlanner{
public:
    PathPlanner(Point2D starting_position, std::vector<Point2D> igoals): pose_initialized(false), curr_goal(0), goals(igoals), direction_preference(std::make_pair<int,int>(RIGHT,LEFT))
    {
        path_pub = n.advertise<nav_msgs::Path>("path", 100);
        pose_sub = n.subscribe("/yeti/pose",1, &PathPlanner::receive_pose, this);
        obstacle_sub = n.subscribe("obstacle_detection/obstacles",1,&PathPlanner::receive_obstacles, this);
        n.param<double>("set_speed", set_speed, SET_SPEED);
        n.param<double>("obstacle_buffer", obstacle_buffer, OBSTACLE_BUFFER);
        n.param<double>("goal_acceptance_distance",goal_acceptance_distance, GOAL_DISTANCE_ACCEPTENCE);

    }

    nav_msgs::Path get_path(const geometry_msgs::Pose2D& state, const Point2D& goal_point, std::vector<Point2D> obstacles)
    {
        //sample straight line from state to goal
        //if sample point is within obstacle buffer
            //move point left or right (orthogonol to path until outside of buffer
                //left or right based on distance from other obstacles or course boundaries
        //if state is at goal point
            //increment goal point
        Point2D sample_point(state.x,state.y);
        nav_msgs::Path path;
        Eigen::Vector2d state_vec( state.x, state.y );
        Eigen::Vector2d goal_vec( goal_point.x, goal_point.y);

        Eigen::ParametrizedLine<double, 2> line = line.Through( state_vec, goal_vec);
        double curr_dist = 0;
        while(distance(sample_point, goal_point) > goal_acceptance_distance)
        {
            auto p = line.pointAt(curr_dist);
            sample_point.x = p.data()[0];
            sample_point.y = p.data()[1];
            for(ulong i = 0; i < detected_obstacles.size(); ++i){
                if(distance(sample_point, detected_obstacles.at(i)) < obstacle_buffer)
                {
                    bool left = false, right = false;
                    double dist_left, dist_right;
                    dist_left = project_to_line_segment(detected_obstacles.at(i),std::make_pair<Point2D,Point2D>(Point2D(-3,2), Point2D(12,2))).second;
                    dist_right = project_to_line_segment(detected_obstacles.at(i),std::make_pair<Point2D,Point2D>(Point2D(-3,-2), Point2D(12,-2))).second;
                    if(dist_left > obstacle_buffer && dist_right > obstacle_buffer)
                    {
                        auto left_proj = project_to_line_segment(Point2D(state.x,state.y),std::make_pair<Point2D,Point2D>(Point2D(-3,2), Point2D(12,2)));
                        double robot_dist_left = left_proj.second;
                        auto right_proj = project_to_line_segment(Point2D(state.x,state.y),std::make_pair<Point2D,Point2D>(Point2D(-3,-2), Point2D(12,-2)));
                        double robot_dist_right = right_proj.second;
                        if(curr_goal < 1)
                        {
                            if(direction_preference.first == LEFT) {
                                sample_point.y =
                                        (std::fabs(detected_obstacles.at(i).y) + std::fabs(left_proj.first.y)) / 2;
                            } else
                            {
                                sample_point.y = -(std::fabs(detected_obstacles.at(i).y) + std::fabs(right_proj.first.y)) / 2;
                            }
                        }
                        else{
                            if(direction_preference.second == LEFT) {
                                sample_point.y =
                                        (std::fabs(detected_obstacles.at(i).y) + std::fabs(left_proj.first.y)) / 2;
                            } else
                            {
                                sample_point.y = -(std::fabs(detected_obstacles.at(i).y) + std::fabs(right_proj.first.y)) / 2;
                            }
                        }
                    }
                    else if(dist_left <= dist_right)
                    {
                        sample_point.y = sample_point.y - obstacle_buffer/2;
                    } else{
                        sample_point.y = sample_point.y + obstacle_buffer/2;
                    }
                }
            }
            path.poses.push_back(point(sample_point,set_speed));
            curr_dist += SAMPLE_STEP_SIZE;
        }
        return path;
    }

    void process()
    {
        if(distance(robot_pose,goals.at(curr_goal)) < goal_acceptance_distance)
        {
            curr_goal++;
        }
        if(curr_goal == goals.size())
        {
            while(true)
            {
                nav_msgs::Path stop;
                geometry_msgs::PoseStamped zero;
                zero.header.frame_id = "map";
                zero.header.stamp = ros::Time::now();
                zero.pose.position.x = robot_pose.x;
                zero.pose.position.y = robot_pose.y;
                zero.pose.position.z = 0;
                geometry_msgs::PoseStamped zero2;
                zero2.header.frame_id = "map";
                zero2.header.stamp = ros::Time::now();
                zero2.pose.position.x = robot_pose.x + 1;
                zero2.pose.position.y = robot_pose.y + 1;
                zero2.pose.position.z = 0;
                stop.poses.push_back(zero);
                stop.poses.push_back(zero);
                publish_path(stop);
            }
        }
        nav_msgs::Path path = get_path(robot_pose, goals.at(curr_goal), detected_obstacles);
        publish_path(path);
    }

private:

    void receive_pose(const geometry_msgs::Pose2D::ConstPtr& pose)
    {
        robot_pose = *pose;
        if(!pose_initialized)
        {
            pose_initialized = true;
        }

    }

    void receive_obstacles(const visualization_msgs::MarkerArray::ConstPtr& obstacles)
    {
        std::vector<Point2D> obstacle_vec;

        for(const auto& o : obstacles->markers)
        {
            obstacle_vec.emplace_back(Point2D(o.pose.position.x, o.pose.position.y));
        }

        detected_obstacles = obstacle_vec;
    }

    void publish_path(nav_msgs::Path& path)
    {
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        path_pub.publish(path);
    }

    bool pose_initialized;
    unsigned long curr_goal;
    double set_speed;
    double obstacle_buffer;
    double goal_acceptance_distance;
    std::pair<int,int> direction_preference;
    std::vector<Point2D> goals;
    std::vector<Point2D> detected_obstacles;
    geometry_msgs::Pose2D robot_pose;
    ros::NodeHandle n;
    ros::Publisher path_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber obstacle_sub;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "path_planning_node");

    std::vector<Point2D> goals =  {{11,0}/*,{11,1.01}*/,{-1,0}};

    PathPlanner planner(Point2D(0,0), goals);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        planner.process();
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
