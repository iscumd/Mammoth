#include <ros/ros.h>
//#include <chrono>
#include <boost/chrono.hpp>
#include <iostream>
#include <deque>
#include <queue>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include "boost/date_time/posix_time/posix_time.hpp"
using namespace boost::posix_time;
using namespace std;
//https://stackoverflow.com/a/4974588 but use boost

typedef boost::chrono::high_resolution_clock Clock;
typedef boost::chrono::milliseconds Milliseconds;

class StallDetection
{
    private:
    ros::NodeHandle n;
	ros::Publisher stallVelocityPub;
    ros::Publisher navigationPIDStatusPub;
	ros::Subscriber robotCurrentVelocitySub;
    ros::Subscriber expectedVelocitySub;
    ros::Subscriber actualVelocitySub;
    ros::Subscriber eStopStatusSub;
    int queueSize;
    deque<geometry_msgs::Pose2D> robotVelocityHistory;
    bool disableNavigationPID;
    bool eStopStatus;
    geometry_msgs::Twist actualVelocity;
    geometry_msgs::Twist expectedVelocity;
    ptime stuckTime;
    double maxSpeed;
    bool stallDetected;
    double overrideSpeed;
    double reverseDurationInMilli;
    //Initialize tolerance thresholds and time saving for Slip detection
    double noMovementTolerance;// 10 cm
    double noRotationTolerance; //5 degrees 
    //DateTime lastStuckTime; //save last stuck time
    //TimeSpan reverseDuration = new TimeSpan(0, 0, 0, 0, 800); // 0.5 Seconds
    double reverseSpeed;
    //Private Functions
    double findMinElement(string returnType)
    {
        if(returnType == "x")
        {
            std::deque<geometry_msgs::Pose2D>::iterator it = std::min_element(robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( geometry_msgs::Pose2D &a, geometry_msgs::Pose2D &b)
                             {
                                 return a.x < b.x;
                             } ); 
                return (*it).x;
        }
        else if (returnType == "y")
        {
            std::deque<geometry_msgs::Pose2D>::iterator it = std::min_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( geometry_msgs::Pose2D &a, geometry_msgs::Pose2D &b)
                             {
                                 return a.y < b.y;
                             } );
                return (*it).y;
        }
        else if(returnType == "theta")
        {
            std::deque<geometry_msgs::Pose2D>::iterator it = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             [](geometry_msgs::Pose2D &a, geometry_msgs::Pose2D &b )
                             {
                                 return a.theta < b.theta;
                             } );
                return (*it).theta;
        }
        else
            return 0.0;

    }
    double findMaxElement(string returnType)
    {
        if(returnType == "x")
        {
            std::deque<geometry_msgs::Pose2D>::iterator it = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             [](geometry_msgs::Pose2D &a, geometry_msgs::Pose2D &b )
                             {
                                 return a.x < b.x;
                             } ); 
                return (*it).x;
        }
        else if (returnType == "y")
        {
            std::deque<geometry_msgs::Pose2D>::iterator it = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             [](geometry_msgs::Pose2D &a, geometry_msgs::Pose2D &b )
                             {
                                 return a.y < b.y;
                             } );
                return (*it).y;
        }
        else if(returnType == "theta")
        {
            std::deque<geometry_msgs::Pose2D>::iterator it = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             [](geometry_msgs::Pose2D &a, geometry_msgs::Pose2D &b )
                             {
                                 return a.theta < b.theta;
                             } );
                return (*it).theta;
        }
        else
            return 0.0;
    }

    void robotPositionCallback(const geometry_msgs::Pose2D::ConstPtr& robotLocation)
    {
        addVelocityToHistory(*robotLocation);
    }
    void eStopStatusCallback(const std_msgs::Bool::ConstPtr& gpio)
    {
        //eStopStatusCallback = gpio->eStop;
    }
    void expectedRobotVelocityCallback(const geometry_msgs::Twist::ConstPtr& velocity)
    {
        expectedVelocity = *velocity;
    }
    void actualRobotVelocityCallback(const geometry_msgs::Twist::ConstPtr& velocity)
    {
        actualVelocity = *velocity;
    }
    public:
    StallDetection()
    {
        robotCurrentVelocitySub = n.subscribe("/localization/robot_location", 100, &StallDetection::robotPositionCallback, this);
        actualVelocitySub = n.subscribe("/localization/velocity", 100, &StallDetection::expectedRobotVelocityCallback, this);
        expectedVelocitySub = n.subscribe("/obstacle_reaction/velocity", 100, &StallDetection::expectedRobotVelocityCallback, this);
        eStopStatusSub = n.subscribe("gpio/inputs", 1, &StallDetection::eStopStatusCallback, this);
        n.param("robot_velocity_queue_size", queueSize, 100);
        n.param("slip_detection_noMovementTolerance",noMovementTolerance, 0.1);// 10 cm
        n.param("slip_detection_noRotationTolerance" ,noRotationTolerance, 5.0); //5 degrees
        n.param("slip_detection_reverseSpeed", reverseSpeed, -0.5);
        n.param("stall_detection_maxSpeed", maxSpeed, 0.7);
        n.param("stall_detection_reverse_duration", reverseDurationInMilli, 0.7);

        stallDetected = false;
        robotVelocityHistory.resize(queueSize);
    } 
    bool getStallStatus()
    {
        return stallDetected;
    }
    double getReverseDurationInMilli()
    {
        return reverseDurationInMilli;
    }
    double getExpectedVelocity()
    {
        return expectedVelocity.linear.x;
    }
    double getMaxSpeed()
    {
        return maxSpeed;
    }
    double getReverseSpeed()
    {
        return reverseSpeed;
    }
    ptime getLastStuckTime()
    {
        return stuckTime;
    }
    void addVelocityToHistory(geometry_msgs::Pose2D robotCurrentVelocity)
    {
        if(robotVelocityHistory.size() > queueSize)
        {
            robotVelocityHistory.pop_front();
        }
        robotVelocityHistory.push_back(robotCurrentVelocity);

        if(robotVelocityHistory.size() >= queueSize)
        {
            activateStallDetection();
        }

    }
    void activateStallDetection()
    {
        double minX = findMinElement("x");
        double maxX = findMaxElement("x");
        double minY = findMinElement("y");
        double maxY = findMaxElement("y");
        double minHeading = findMinElement("theta");
        double maxHeading = findMaxElement("theta");

        if ((pow(maxX - minX, 2) + pow(maxY - minY, 2) < noMovementTolerance) 
            && maxHeading - minHeading < noRotationTolerance && eStopStatus && 
            (expectedVelocity.linear.x != actualVelocity.linear.x) || (expectedVelocity.angular.z != actualVelocity.angular.z))
                {
                    stallDetected = true;
                    ptime t(second_clock::local_time());//save the time which slipping was detected 
                    stuckTime = t;
                    robotVelocityHistory.clear();// clear the QUeue so that stall detection cannot occur again until the queue is full
                }
        else
            stallDetected = false;
    }

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stall_detection");
    ros::NodeHandle n;
    StallDetection stallDetection;
    double reverseDurationInMilli = stallDetection.getReverseDurationInMilli();
    time_duration td = milliseconds(reverseDurationInMilli);
    ros::Publisher stallVelocityPub = n.advertise<geometry_msgs::Twist>("/stall/velocity", 1000);
    ros::Publisher navigationPIDStatusPub = n.advertise<std_msgs::Bool>("/navigation/disable", 1000); 
	geometry_msgs::Twist stallVelocity;
    std_msgs::Bool navigationOff;
    navigationOff.data = stallDetection.getStallStatus();
	while(ros::ok())
	{
		ros::spinOnce();
        ptime lastStuckTime (stallDetection.getLastStuckTime() + td);
        ptime now (second_clock::local_time());
        if(stallDetection.getStallStatus())
            if(lastStuckTime > now)
            {
                //Send Reverse Speed
                double reverseSpeed = stallDetection.getMaxSpeed() * stallDetection.getReverseSpeed();
                stallVelocity.linear.x = reverseSpeed;
                stallVelocity.angular.z = 0.0;
                stallVelocityPub.publish(stallVelocity);
            }
            // else
            // {
            //     //Send Normal Speed
            //     double normalSpeed = stallDetection.getExpectedVelocity();
            //     stallVelocity.linear.x = normalSpeed;
            //     stallVelocity.angular.z = 0.0;
            //     stallVelocityPub.publish(stallVelocity);
            // }
            navigationPIDStatusPub.publish(navigationOff);
        }
	
	
	return 0;
}
