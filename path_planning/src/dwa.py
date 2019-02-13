#!/usr/bin/python
import rospy
from geometry_msgs import Pose2D
from std_msgs import Float64

def main():
    rospy.init_node('dynamic_window_planning')
    pub = rospy.Publisher('/linear_velocity_setpoint', Float64 ,queue_size = 1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass