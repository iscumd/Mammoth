#ifndef BUFFER_H
#define BUFFER_H

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include <yeti_snowplow/obstacles.h>
#include <yeti_snowplow/obstacle.h>
#include <yeti_snowplow/location_point.h>
#include <yeti_snowplow/lidar_point.h>
#include <yeti_snowplow/turn.h>
#include <yeti_snowplow/target.h>


#define M_PI           3.14159265358979323846  /* pi */

using namespace std;

class Buffer
{
private:
    const double combinedBufferWidth = 0.55; //0.425; // 425;
    const double combinedBufferLength = 2.5; //2000.0; // the distance in which obstacle avoidance begins if an obstacle is within this distance. \
    //Also defines the required distance away any obstacle must be for the robot to consider turning toward that direction

    const double combinedHalfRobotWidth = 0.35; //350.0;// this is the width of the robot/2. Should algin with the outsides of the wheel.
    int badCount = 0;// temporary counter to hold the number of LiDAR points in the way of a turn angle. used in CombinedVectorSCan

    //Sampling Thresholds
    const double SAMPLING_DISTANCE = 0.1; //100.0;
    const int BADCOUNTTHRESH = 5;// maximum number of LiDAR points associated with a turn angle to be considered not sage (IE if bad count > BADCOUNTTHRESH, then the robot will hit an obstacle. 
    const double ANGLE_SAMPLES = 18;// the number of angles to try to avoid an obstacle
    const double SAMPLING_ANGLE = M_PI /ANGLE_SAMPLES;// figures out the angle between each sample angle
    double lidarDataAngularResolution;
public:
    const double DOOM = (-135.0) * 2.0 * M_PI / 360.0;//this is bad....
    std::vector<yeti_snowplow::location_point> combinedBufPoints;//stores the list of LiDAR points which are within the buffer of the robot.
    
    double adjust_angle(double angle, double circle)//this function limits the angle between (-pi & pi) or (-180 & 180)
    {   //circle = 2pi for radians, 360 for degrees
        //Subtract multiples of circle
        angle -= floor(angle / circle) * circle;
        angle -= floor(2 * angle / circle) * circle;
        return angle;
    }
    
    double distance(yeti_snowplow::location_point point1, yeti_snowplow::location_point point2)
    {
        return sqrt(pow((point2.x - point1.x), 2)- pow((point2.y - point1.y), 2));
    }
    //this function checks a turn angle (provided by wheel scans) to see if there is anything in the way of the desired turn angle.
    //It returns true if the robot can turn that direction without hittinemmg an obstacle, and returns false if there is something in
    //the way of the entered turn angle.
    bool combinedVectorScan(yeti_snowplow::location_point source, yeti_snowplow::location_point destination)
    {
        double target_angle = adjust_angle(atan2(destination.x - source.x, destination.y - source.y), 2.0* M_PI  ); //limit angle between (-pi & pi) or (-180 & 180)
        double target_dist = distance(destination, source); //find distance between corner of wheel to desired target. this equals Buffer length
        double dist;
        int i = 0;
        int j = 0;//set up two iterators
        double tempDist = 0.0;

        yeti_snowplow::location_point sample_point;// create temporary point which is updated each iteration
           
        do// find out if there is anything in the way of the 
        {
            sample_point.x = (i * SAMPLING_DISTANCE) * sin(target_angle);//create temporary XY point along the desired target angle at varying distances
            sample_point.y = (i * SAMPLING_DISTANCE) * cos(target_angle);//create temporary XY point along the desired target angle at varying distances

            for(yeti_snowplow::location_point cpoint : combinedBufPoints)//look through each point point in the buffer LiDAR points to see if any are by the temporary point
            {
                if (cpoint.y < 0) //if it's behind the robot, skip it
                {
                    continue;
                }
                dist = distance(sample_point, cpoint); //figure out distance between the temporary point and this point in the buffer LiDAR points
                if (dist < combinedBufferWidth) // if the dcurrent LiDAR point is close to the temporary point
                {
                    badCount++;//increment the number of LiDAR points in the way of this target angle
                    if (badCount > BADCOUNTTHRESH) //if there's too many lidar points in the way then this is not a valid turn angle
                    {
                        return false;//return false to indicate that this turn angle is not good.
                    }//end BAD COUNT THRESH
                }//END dist<combinedBufferWidth
            }//end For each LiDAR point
            i++;
        } while (SAMPLING_DISTANCE * i < target_dist); // while the sampling distance along the target angle is less than the target distance

        return true;//return true when it is possible for the robot to take this turn angle without hitting obstacles
    }//end CombinedVectorScan

    bool combinedCheckAngle(double targetAngle)
    {
        yeti_snowplow::location_point leftWheel;
        yeti_snowplow::location_point rightWheel;
        yeti_snowplow::location_point target;

        leftWheel.y = 0;
        rightWheel.y = 0;

        leftWheel.x = -combinedHalfRobotWidth;
        rightWheel.x = combinedHalfRobotWidth;

        target.x = combinedBufferLength * sin(targetAngle);
        target.y = combinedBufferLength * cos(targetAngle);

        return (combinedVectorScan(leftWheel, target) && combinedVectorScan(rightWheel, target));
    }

    double combinedRightWheelScan(yeti_snowplow::location_point target)
    {
        yeti_snowplow::location_point source;
        double targetAngle = adjust_angle(atan2(target.x, target.y), 2.0*M_PI);

        double samplePhi;
        int index = 0;

        source.x = combinedHalfRobotWidth;
        source.y = 0;

        do{
            yeti_snowplow::location_point samplePoint;
            samplePoint.x = combinedBufferLength * sin(targetAngle - SAMPLING_ANGLE * index);
            samplePoint.y = combinedBufferLength * cos(targetAngle - SAMPLING_ANGLE * index);
            samplePhi = adjust_angle(atan2(samplePoint.x, samplePoint.y), 2.0 * M_PI);

            if(combinedVectorScan(source, samplePoint))
            {
                if(combinedCheckAngle(samplePhi))
                {
                    if(index == 0)
                        return 0;
                    
                    return samplePhi;
                }
            }
            index++;
        }while(index < ANGLE_SAMPLES);

        return DOOM;
    }

    double combinedLeftWheelScan(yeti_snowplow::location_point target)
    {
        yeti_snowplow::location_point source;
        double targetAngle = adjust_angle(atan2(target.x, target.y), 2.0*M_PI);

        double samplePhi;
        int index = 0;

        source.x = -combinedHalfRobotWidth;
        source.y = 0.0;

        do{
            yeti_snowplow::location_point samplePoint;
            samplePoint.x = combinedBufferLength * sin(targetAngle + SAMPLING_ANGLE * index);
            samplePoint.y = combinedBufferLength * cos(targetAngle + SAMPLING_ANGLE * index);
            samplePhi = adjust_angle(atan2(samplePoint.x, samplePoint.y), 2.0 * M_PI);

            if(combinedVectorScan(source, samplePoint))
            {
                if(combinedCheckAngle(samplePhi))
                {
                    if(index == 0)
                        return 0;
                    
                    return samplePhi;
                }
            }
            index++;
        }while(index < ANGLE_SAMPLES);

        return DOOM;
    }

    vector<yeti_snowplow::obstacle> combinedUpdatePoints(const vector<float> lidarPoints, double  lidarAngle)
    {
        combinedBufPoints.clear();
        badCount = 0; 
        double middleAngle = (135 * (M_PI / 180.0)); //radians
        for(int i =0; i < lidarPoints.size(); i++)
        {
            double thisX = lidarPoints[i] * cos(middleAngle - i * lidarAngle);
            double thisY = lidarPoints[i] * sin(middleAngle - i * lidarAngle);
    
            if(sqrt(pow(thisX, 2) + pow(thisY, 2)) < combinedBufferLength)
            {
                if(thisY > 0)
                {
                    yeti_snowplow::location_point locationPoint;
                    locationPoint.x = thisX;
                    locationPoint.y = thisY;
                    combinedBufPoints.push_back(locationPoint);
                }
            }
        }
    }
};
#endif