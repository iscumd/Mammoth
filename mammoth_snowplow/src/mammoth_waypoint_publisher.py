#! /usr/bin/env python

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf
import pandas as pd #pandas is used to read in csv files
import os#added for file path


class mammoth_waypoint_manager(object):
    def __init__(self, filename):
        self.listener = tf.TransformListener()
        self.waypoint_index = 0
        self.tolerance_x = 0.4
        self.tolerance_y = 0.4
        self.tolerance_z = 0.3
        self.tolerance_qx = 0.3
        self.tolerance_qy = 0.3
        self.tolerance_qz = 0.3
        self.tolerance_qw = 0.3
        self.pub = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=10)
        self.import_file(filename)


    def import_file(self,filename):
        #import the file here
        #do quaternion_from_euler here
        print("Importing File")
        dir = os.path.dirname(__file__)		#get abs path because relative is from launch location
        filename = os.path.join(dir, filename)
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
	while(True):        
	    try:
            	(position, orientation) = self.listener.lookupTransform('/map','/base_link', rospy.Time(0))
                
	    except Exception as e :#Will always error until it stablizes
                #print(e)
                continue
            #except (tf.ExtrapolationException):
                #print(e)
                #continue
                #exit()
            #except (tf.ConnectivityException):
                #print("Something went wrong with Connectivity")
                #continue
	        #exit()
            #except (tf.LookupException):
                #print("Something went wrong with Lookup")
                #exit()
            break
	#print(position)
	#print(orientation)
	#print(self.waypoints.iloc[self.waypoint_index])
        good_job = 0
        #print(self.waypoint_index)
        #print(position)
	#print(orientation)
	
	print(self.waypoints.iloc[self.waypoint_index])
        if abs(position[0] - self.waypoints.iloc[self.waypoint_index].X) < self.tolerance_x:
	   # print("tolerance_x")
	    if abs(position[1] - self.waypoints.iloc[self.waypoint_index].Y) < self.tolerance_y:
		#print("tolerance_y")
                if abs(position[2] - self.waypoints.iloc[self.waypoint_index].Z) < self.tolerance_z:
		   # print("tolerance_z")
                    if abs(orientation[0] - self.waypoints.iloc[self.waypoint_index].QX) < self.tolerance_qx:
			#print("tolerance_qx")
                        if abs(orientation[1] - self.waypoints.iloc[self.waypoint_index].QY) < self.tolerance_qy:
			   # print("tolerance_qy")
                            if abs(orientation[2] - self.waypoints.iloc[self.waypoint_index].QZ) < self.tolerance_qz:
				#print("tolerance_qz")
                                if abs(orientation[3] - self.waypoints.iloc[self.waypoint_index].QW) < self.tolerance_qw:
                                    good_job = 1
		#else:
		   # print(position[2])
		    #print(self.waypoints.iloc[self.waypoint_index].Z)
        if good_job:
            print("goodJob")
	    self.waypoint_index = self.waypoint_index + 1
            self.send_it()#send next way point. This should probably have some error checking. IE out of array bounds
	    #print(self.waypoint_index)
            #print(position)
	    #print(orientation)
	    #print(self.waypoints.iloc[self.waypoint_index])
	#else:
	 #   rospy.sleep(2)
	  #  self.send_it()





    def run(self):
        #rospy.Subscriber("pose", geometry_msgs.msg.PoseStamped, self.callback_pose)
        rate = rospy.Rate(10.0)
	#print("Running")
        #print(self.waypoints.iloc[self.waypoint_index])
	rospy.sleep(1)       
	self.send_it() #send first waypoint?
        while not rospy.is_shutdown():
            self.check_pose()
            rate.sleep()
        print("Ros not running")






    def send_it(self):       
	goal = geometry_msgs.msg.PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
	goal.header.seq= self.waypoint_index;
        goal.pose.position.x = self.waypoints.iloc[self.waypoint_index].X #self.target_pose[0]
        goal.pose.position.y = self.waypoints.iloc[self.waypoint_index].Y
        goal.pose.position.z = self.waypoints.iloc[self.waypoint_index].Z
        goal.pose.orientation.x = self.waypoints.iloc[self.waypoint_index].QX
        goal.pose.orientation.y = self.waypoints.iloc[self.waypoint_index].QY
        goal.pose.orientation.z = self.waypoints.iloc[self.waypoint_index].QZ
        goal.pose.orientation.w = self.waypoints.iloc[self.waypoint_index].QW
	rospy.loginfo(goal)        
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
