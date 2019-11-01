#! /usr/bin/env python

import rospy
import std_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import pandas as pd #pandas is used to read in csv files


class mammoth_waypoint_manager(object):
    def __init__(self, filename):
        self.target_pose = [0,0,0,0,0,0,0] # x,y,z,qx,qy,qz,qw
        self.waypoint_index = 0
        self.pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.import_file(filename)
        
        
    def import_file(self,filename):
        #import the file here
        #do quaternion_from_euler here
        print("Importing File")
        imported = pd.read_csv(filename) #this statement reads in the csv file
        #note that python will look in the current working directory for this file
        #print(dataframe) #this should print the dataframe from the csv file 
        #if you want to skip the first row of the csv file the code is
        #dataframe = pd.read_csv("filename.csv", skiprows = 1)
        converted = []
        
        for import_counter in range(imported.shape[0]):
        
            quaternion = quaternion_from_euler(imported.iloc[import_counter].Roll,imported.iloc[import_counter].Pitch,imported.iloc[import_counter].Yaw)
            converted.append([imported.iloc[import_counter].X,imported.iloc[import_counter].Y,imported.iloc[import_counter].Z,quaternion[0],quaternion[1],quaternion[2],quaternion[3]])

        self.waypoints = pd.DataFrame(converted,columns=["X","Y","Z","QX","QY","QZ","QW"])
        print self.waypoints
        
        

    def check_pose(self):
        #compare pose to target, if its within a certain tolerance move to next
        if self.target_pose == self.latest_pose:
            self.waypoint_index = self.waypoint_index + 1

    
    
    
    def callback_pose(self, pose):
        self.latest_pose = pose
        self.check_pose()
        
    
    def run(self):
        rospy.Subscriber("pose", geometry_msgs.msg.PoseStamped, self.callback_pose)
        rospy.spin()
        
    
        
    
        
    def send_it(self):
        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.target_pose[0]
        goal.pose.position.y = self.target_pose[1]
        goal.pose.posiiton.z = self.target_pose[2]
        goal.pose.orientation.x = self.target_pose[3]
        goal.pose.orientation.y = self.target_pose[4]
        goal.pose.orientation.z = self.target_pose[5]
        goal.pose.orientation.w = self.target_pose[6]
        self.pub.publish(goal)
        
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('mammoth_waypoint_manager')
        #waypoint_file = rospy.get_param("waypoint_list_filename")
        waypoint_file = "test.csv"
        mammoth = mammoth_waypoint_manager(waypoint_file)
        mammoth.run()
        
    
    except rospy.ROSInterruptException:
        pass
