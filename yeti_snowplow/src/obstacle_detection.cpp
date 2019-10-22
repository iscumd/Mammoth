#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <yeti_snowplow/obstacle.h>
#include <yeti_snowplow/obstacles.h>

#define M_PI           3.14159265358979323846  /* pi */
using namespace std;

class ObstacleDetection
{
private:
    double maxRadius;
    const double MM2M  = 0.001;
    const int M2MM = 1000;
    double nonSeparationThresh;
    int highThresh;
    int lowThresh;
    const int forgiveCount = 3;
    const double movingObstacleSize = 0.45;
    double movingObstacleThresh;
    int linkedCount;
    double sumOfPoints;
    double obsSizeNum;
    bool isAlreadyLinking = false;
    bool isThereAnObstacle = false;
    double sumOfHeadings = 0.0;
    double obstacleStopThreshold;
    double minObstacleXPosition;
    double maxObstacleXPosition;
    double minObstacleYPosition;
    double maxObstacleYPosition;
    geometry_msgs::Pose2D robotLocation;
    vector<float> lidarData;
    double lidarDataAngularResolution = 0.25; //radians
    vector<yeti_snowplow::lidar_point> lmsData;
    vector<yeti_snowplow::obstacle> obstacles;

    ros::NodeHandle n;
    ros::Subscriber localizationSub;
    ros::Subscriber scanSub;
    ros::Publisher obstaclePub;//ROS obstacle publisher

public:
    ObstacleDetection()
    {
        scanSub = n.subscribe("/scan", 1, &ObstacleDetection::scanCallback, this);
        obstaclePub = n.advertise<yeti_snowplow::obstacles>("/obstacle_detection/obstacles", 100);
        localizationSub = n.subscribe("/localization/robot_location", 100, &ObstacleDetection::localizationCallback, this);

        n.param("obstacle_detection_maxradius", maxRadius, 10.0);
		n.param("obstacle_detection_nonSeperationThresh", nonSeparationThresh, 200.0);
		n.param("obstacle_detection_highThresh", highThresh, 50);
        n.param("obstacle_detection_lowThresh", lowThresh, 4);
        n.param("obstacle_detection_movingObstacleThreshold", movingObstacleThresh, 0.07);
        n.param("obstacle_min_x_position", minObstacleXPosition, -1.750);
        n.param("obstacle_max_x_position", maxObstacleXPosition, 4.75);
        n.param("obstacle_min_y_position", minObstacleYPosition, -2.75);
        n.param("obstacle_max_y_position", maxObstacleYPosition, 11.75);
    }

    vector<float> getLidarData(){
        return lidarData;
    }

    void clearState()
    {
        sumOfPoints = 0;
        obsSizeNum = 0;
        linkedCount = 0;
        isThereAnObstacle = false;
        isAlreadyLinking = false;
        isThereAnObstacle = false;
    }

    void clearObstacles()
    {
        obstacles.clear();
    }

    void linkPoint(double currPointDist,double currentTheta, double twoPointsDist)
    {
        linkedCount += 1;
        sumOfPoints += currPointDist;
        obsSizeNum += twoPointsDist;
        sumOfHeadings += currentTheta;
    }

    double distanceCalculator(yeti_snowplow::lidar_point lidarPoint1, yeti_snowplow::lidar_point lidarPoint2)
    {
        return sqrt(pow((lidarPoint2.x - lidarPoint1.x), 2)- pow((lidarPoint2.y - lidarPoint1.y), 2));
    }

    double distanceFromRobot(double lidarX, double lidarY)
    {
        return sqrt(pow((lidarX - robotLocation.x), 2)- pow((lidarY - robotLocation.y), 2));
    }
    void convertPointCloudToClass()
    {
        lmsData.clear();
        double middleAngle = (135 * (M_PI / 180.0)); //radians
        for(int i = 0; i < lidarData.size(); i++)
        {
            yeti_snowplow::lidar_point lidar_point;
            lidar_point.x = lidarData[i] * cos((i /4.0) - 45.0 * (M_PI / 180.0));
            lidar_point.y = lidarData[i] * sin((i /4.0) - 45.0 * (M_PI / 180.0));
            lidar_point.theta = atan2(lidar_point.y, lidar_point.x);
            lidar_point.distanceFromRobot = distanceFromRobot(lidar_point.x, lidar_point.y);
            lmsData.push_back(lidar_point);
        }
	    // ROS_INFO("convert point cloud done");
    }
    void addAndAnalyzeObstacle(yeti_snowplow::obstacle obstacle)
    {
        double index = (obstacle.objEndIndex - linkedCount) / 2;
        double mag = sumOfPoints / linkedCount;
        //double avgTheta = sumOfHeadings / linkedCount;
        double obstacleAngle = ((135 - index * lidarDataAngularResolution) * (M_PI / 180.0)) + robotLocation.theta;
        obstacle.x = robotLocation.x + mag * cos(obstacleAngle);
        obstacle.y = robotLocation.y + mag * sin(obstacleAngle);

        if (mag > maxRadius || linkedCount > highThresh || linkedCount < lowThresh)
        {
            //Obstacle is not needed
        }
        else if(obstacle.x == 0.0 && obstacle.y == 0.0){
            //the object is at infinity
        }
        //figure out if the object is within the plowing field or not
        else if (obstacle.x > maxObstacleXPosition || obstacle.x < minObstacleXPosition || obstacle.y > maxObstacleYPosition || obstacle.y < minObstacleYPosition)//check if obstacle is outside of Triple I field
        //else if (obstacle.x > 1.75 || obstacle.x < -1.750 || obstacle.y > 11.75 || obstacle.y < -2.75)//check if obstacle is outside of Single I field
        {  
            //outside the field; ignore
        }
        else
        {
            obstacle.startPoint = lmsData[obstacle.objStartIndex];
            obstacle.endPoint = lmsData[obstacle.objEndIndex];
            obstacle.heading = obstacleAngle;
            obstacle.distance = mag;
            obstacle.obsRoughSize = obsSizeNum;
            obstacle.obsLineSize = distanceCalculator(obstacle.startPoint, obstacle.endPoint); 

            if(mag < 5)
            {
                if(abs(obstacle.obsLineSize - movingObstacleSize) < movingObstacleThresh)
                {
                    obstacle.isAMovingObstacle = true;
                    // ROS_INFO("Moving Obstacle detected");
                }
                else
                {
                    obstacle.isAMovingObstacle = false;
                    // ROS_INFO("Static Obstacle detected");
                }
            }

            obstacles.push_back(obstacle);
        }
        clearState();
    }

    void findObstacles()
    {
	    // ROS_INFO("find obstacles start");
        //Detect Obstacle
        clearObstacles();
        clearState();

        int j = 0;// forgive count variable
        yeti_snowplow::obstacle *obstacle = new yeti_snowplow::obstacle;
        int minIndex = 360; //need to analyze full FOV for new localization?
        if(lmsData.size() < (minIndex + 1)){ //don't try to run if there isn't enough data in lmsData
            return;
        }
        for (int i = minIndex; i < lmsData.size() - minIndex + 1; i++) //need to analyze full FOV for new localization?
        {
	        // ROS_INFO("find obstacle for loop start");
            yeti_snowplow::lidar_point currentPoint = lmsData[i];
            // ROS_INFO("got current point");
            bool isPointLinked = false;
            if(currentPoint.distanceFromRobot < maxRadius && currentPoint.distanceFromRobot != 0.0)
            {
                // ROS_INFO("point less than max radius");
                for(j = 1; j <= forgiveCount; j++ )
                {
                    // ROS_INFO("still less than forgive count");
                    yeti_snowplow::lidar_point nextPoint = lmsData[i + j];
                    double pointsDistance = distanceCalculator(currentPoint, nextPoint);
                    if(pointsDistance < nonSeparationThresh * j * MM2M)
                    {
                        linkPoint(currentPoint.distanceFromRobot, currentPoint.theta, pointsDistance);
                        // ROS_INFO("linking point");
                        isPointLinked = true;
                        if(!isAlreadyLinking)
                        { 
                            obstacle->objStartIndex = i;
                            // ROS_INFO("assign start index");
                            isAlreadyLinking = true;
                        }
                        break;
                    }
                }
            }
            if(isPointLinked == false)
            {
                if(isAlreadyLinking)
                {
                    obstacle->objEndIndex = i;
                    addAndAnalyzeObstacle(*obstacle);
                    obstacle = new yeti_snowplow::obstacle;
                    // ROS_INFO("added new obstacle");
                }
                isAlreadyLinking = false;
                clearState();
            }
            else
            {
                i = i + j - 1;
                // ROS_INFO("shift index");
                if(i > lmsData.size() - (minIndex + 1))
                {
                    obstacle->objEndIndex = i;
                    addAndAnalyzeObstacle(*obstacle);
                    clearState();
                }
            }
            
        }

        return;
    }
    void localizationCallback(const geometry_msgs::Pose2D::ConstPtr& robotPosition) 
    {
        robotLocation.x = robotPosition->x;
		robotLocation.y = robotPosition->y;
		robotLocation.theta = robotPosition->theta;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scannedData)
    {
        /* This fires every time a new obstacle scan is published */
        //location contains an array of points, which contains an x and a y relative to the robot

        lidarData = scannedData->ranges;
        lidarDataAngularResolution = scannedData->angle_increment; //radians
	    // ROS_INFO("scan callback done");
    }
    
    void publishObstacles()
    {
        yeti_snowplow::obstacles msg;
        msg.obstacles = obstacles;
        obstaclePub.publish(msg);
	    // ROS_INFO("publish");
    }

};
// input: robot position relative to field
// input: array of lidar scan data
// output: array of obstacles (x,y,r(size), moving(boolean))

int main(int argc, char **argv){
	ros::init(argc, argv, "obstacle_detection_node");

    ObstacleDetection obstacleDetection;
	// ROS_INFO("init class");
	ros::Rate loopRate(25); //Hz
    while(ros::ok())
    {
	    // ROS_INFO("while loop");
        ros::spinOnce();
        if(obstacleDetection.getLidarData().size() > 0){ //make sure the array is filled before running
            obstacleDetection.convertPointCloudToClass();
            obstacleDetection.findObstacles();
            //Publish all obstacles
            obstacleDetection.publishObstacles();
        }
		loopRate.sleep();
    }
	ros::spin();
	
	return 0;
}
