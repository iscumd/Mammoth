#! /usr/bin/env python

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf
import pandas as pd #pandas is used to read in csv files


class mammoth_waypoint_manager(object):
    def __init__(self, filename):
        self.listener = tf.TransformListener()
        self.waypoint_index = 0
        self.tolerance_x = 0.1
        self.tolerance_y = 0.1
        self.tolerance_z = 0.1
        self.tolerance_qx = 0.1
        self.tolerance_qy = 0.1
        self.tolerance_qz = 0.1
        self.tolerance_qw = 0.1
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

            quaternion = tf.transformations.quaternion_from_euler(imported.iloc[import_counter].Roll,imported.iloc[import_counter].Pitch,imported.iloc[import_counter].Yaw)
            converted.append([imported.iloc[import_counter].X,imported.iloc[import_counter].Y,imported.iloc[import_counter].Z,quaternion[0],quaternion[1],quaternion[2],quaternion[3]])

        self.waypoints = pd.DataFrame(converted,columns=["X","Y","Z","QX","QY","QZ","QW"])
        print self.waypoints



    def check_pose(self):
        #compare pose to target, if its within a certain tolerance move to next
        try:
            (position, orientation) = self.listener.lookupTransform('/map','/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            exit()

        good_job = 0
        if abs(position[0] - self.waypoint.iloc[waypoint_index].X) > self.tolerance_x:
            if abs(position[1] - self.waypoint.iloc[waypoint_index].Y) > self.tolerance_y:
                if abs(position[2] - self.waypoint.iloc[waypoint_index].Z) > self.tolerance_z:
                    if abs(orientation[3] - self.waypoint.iloc[waypoint_index].QX) > self.tolerance_qx:
                        if abs(orientation[4] - self.waypoint.iloc[waypoint_index].QY) > self.tolerance_qy:
                            if abs(orientation[5] - self.waypoint.iloc[waypoint_index].QZ) > self.tolerance_qz:
                                if abs(orientation[6] - self.waypoint.iloc[waypoint_index].QW) > self.tolerance_qw:
                                    good_job = 1
        if good_job:
            self.waypoint_index = self.waypoint_index + 1
            self.send_it()






    def run(self):
        #rospy.Subscriber("pose", geometry_msgs.msg.PoseStamped, self.callback_pose)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.check_pose()
            rate.sleep()






    def send_it(self):
        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.waypoints.iloc[self.waypoint_index].X #self.target_pose[0]
        goal.pose.position.y = self.waypoints.iloc[self.waypoint_index].Y
        goal.pose.posiiton.z = self.waypoints.iloc[self.waypoint_index].Z
        goal.pose.orientation.x = self.waypoints.iloc[self.waypoint_index].QX
        goal.pose.orientation.y = self.waypoints.iloc[self.waypoint_index].QY
        goal.pose.orientation.z = self.waypoints.iloc[self.waypoint_index].QZ
        goal.pose.orientation.w = self.waypoints.iloc[self.waypoint_index].QW
        self.pub.publish(goal)



if __name__ == '__main__':
    try:
        rospy.init_node('mammoth_waypoint_manager')
        #waypoint_file = rospy.get_param("waypoint_list_filename")
        waypoint_file = "../config/test.csv"
        mammoth = mammoth_waypoint_manager(waypoint_file)
        mammoth.run()


    except rospy.ROSInterruptException:
        pass
