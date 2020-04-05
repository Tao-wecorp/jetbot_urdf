#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
from math import atan2, pi
import numpy as np
import random
import time
import itertools

import rospkg
rospack = rospkg.RosPack()
import os
import sys
helpers_folder = os.path.join(rospack.get_path("jetbot_dqn"), "scripts/helpers")
sys.path.append(helpers_folder)

from openpose import OpenPose
openpose = OpenPose()
x_fpv, y_fpv = [320, 480]

from qlearning import QLearning
q = QLearning()

class Pose(object):
    def __init__(self):
        rospy.init_node('pose_node', anonymous=True)

        self.img_sub = rospy.Subscriber("/robot/camera/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        self.states_sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.states_callback)
        # self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.frame is not None:
                # start_time = time.time()
                frame = deepcopy(self.frame)

                x_hip, y_hip = openpose.detect(frame)[11]
                yaw_angle = q.yaw([x_hip, y_hip])
                # print(yaw_angle)

                frame = cv2.circle(frame, (int(x_hip), int(y_hip)), 3, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                frame = cv2.circle(frame, (int(x_fpv), int(y_fpv)), 5, (255, 0, 255), thickness=-1, lineType=cv2.FILLED)
                cv2.imshow("", frame)
                cv2.waitKey(1)

                # print("%s seconds" % (time.time() - start_time))
            rate.sleep()
    
    def camera_callback(self,data):
        try:
            cv_img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_img

    def states_callback(self,data):
        state_msg = ModelState()
        state_msg.model_name = 'robot'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
        state_msg.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 5*pi/180))
        state_msg.twist.linear.x = 0 
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0
        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0
        state_msg.reference_frame = 'world'
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            robot_ori = data.pose[2].orientation.w

            # states_msg = ModelStates()
            # states_msg = data
            # states_msg.pose[2].orientation.w = 0.8

            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(state_msg)

            # self.set_state(state_msg)
        except rospy.ServiceException, e:
            print(e)


def main():
    try:
        Pose()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()