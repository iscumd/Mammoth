#! /usr/bin/env python

import rospy
import std_msgs.msg
import move_base_msgs.msg
import os
import pygame

class auditory_feedback():
    def __init__(self):
        pygame.init()
        pygame.mixer.init()
        self.dir = os.path.dirname(os.path.abspath(__file__))
        self.songname0 = self.dir + '/../audio/planner_failed.ogg'
        self.songname1 = self.dir + '/../audio/new_waypoint.ogg'
        self.songname2 = self.dir + '/../audio/completed.ogg'
        self.songname3 = self.dir + '/../audio/waypoint_kicker.ogg'

    def callback(self,data):
        if(data.data == 1):
            pygame.mixer.music.load(self.songname1)
        elif(data.data == 2):
            pygame.mixer.music.load(self.songname2)
        elif(data.data == 3):
            pygame.mixer.music.load(self.songname3)
        else:
            return
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)


    def callbackError(self,data):
        if(data.status.status == 4):
            pygame.mixer.music.load(self.songname0)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)

    def run(self):
        rospy.init_node('Auditory_Feedback', anonymous=True)
        rospy.Subscriber("/yeti/auditory_feedback", std_msgs.msg.Int8, self.callback)
        rospy.Subscriber("/move_base/result", move_base_msgs.msg.MoveBaseActionResult, self.callbackError)

        rospy.spin()
        
if __name__ == '__main__':
    try:
        auditory = auditory_feedback()
        auditory.run()
    except rospy.ROSInterruptException:
        pass
