#! /usr/bin/env python

import rospy
import std_msgs.msg
import os
import pygame

class meme():
    def __init__(self):
        pygame.init()
        pygame.mixer.init()
        self.songname = os.path.dirname(os.path.abspath(__file__))
        self.songname += '/meme.ogg'
        pygame.mixer.music.load(self.songname)
        
    def callback(self,data):
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
        
    def run(self):
        rospy.init_node('memes', anonymous=True)
        rospy.Subscriber("/yeti/rotate", std_msgs.msg.Bool, self.callback)
        rospy.spin()
        
if __name__ == '__main__':
    try:
        memes = meme()
        memes.run()
    except rospy.ROSInterruptException:
        pass
