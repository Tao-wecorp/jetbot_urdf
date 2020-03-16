#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time

rospy.init_node('jetbot')
rate = rospy.Rate(30)
pub_vel_left_1 = rospy.Publisher('/robot/joint1_velocity_controller/command', Float64, queue_size=5)
pub_vel_right_1 = rospy.Publisher('/robot/joint2_velocity_controller/command', Float64, queue_size=5)

while not rospy.is_shutdown():
    pub_vel_left_1.publish(30)
    pub_vel_right_1.publish(0)
rate.sleep()