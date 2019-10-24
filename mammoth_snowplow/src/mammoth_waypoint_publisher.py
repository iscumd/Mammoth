#! /usr/bin/env python

import rospy
import std_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler


class mammoth_waypoint_manager(object, filename):
    def __init__(self):
        self.target_pose = [0,0,0,0,0,0,0] # x,y,z,qx,qy,qz,qw
        self.pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.import()
        
        
    def import(self):
        #import the file here
        #do quaternion_from_euler here
    
    def check_pose(self):
        #compare pose to target, if its within a certain tolerance move to next
        
    def 
    
    
    
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
        waypoint_file = rospy.get_param("waypoint_list_filename")
        mammoth = mammoth_waypoint_manager(waypoint_file)
        mammoth.run()
        
    
    except rospy.ROSInterruptException:
        pass
