#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import cv2

from math import *
import numpy as np
import time

from helpers.openpose import OpenPose
openpose = OpenPose()
x_fpv, y_fpv = [320, 480]

# from helpers.qlearning import QLearning
# q = QLearning()
pose = Pose()

class Yaw(object):
    def __init__(self):
        rospy.init_node('yaw_node', anonymous=True)

        self.sub_img = rospy.Subscriber("/robot/camera/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None
        self.robot_position = None

        self.pub_vel_left = rospy.Publisher('/robot/joint1_velocity_controller/command', Float64, queue_size=5)
        self.pub_vel_right = rospy.Publisher('/robot/joint2_velocity_controller/command', Float64, queue_size=5)
        self.sub_states = rospy.Subscriber('/gazebo/model_states', ModelStates, self.states_callback)    
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_simulation()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.frame is not None:
                start_time = time.time()
                frame = deepcopy(self.frame)
                
                # points = openpose.detect(frame)
                # x_hip, y_hip = points[11]
                # yaw_angle = q.yaw([x_hip, y_hip])

                self.pub_vel_left.publish(15)
                self.pub_vel_right.publish(0)
                # time.sleep(10)
                # self.pub_vel_left.publish(0)
                # self.pub_vel_right.publish(0)

                # for i in range(len(points)):
                #     if points[i] is not None:
                #         frame = cv2.circle(frame, (int(points[i][0]), int(points[i][1])), 3, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                # # frame = cv2.circle(frame, (int(x_hip), int(y_hip)), 3, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                # frame = cv2.circle(frame, (int(x_fpv), int(y_fpv)), 10, (255, 0, 255), thickness=-1, lineType=cv2.FILLED)
                # cv2.imshow("", frame)
                # cv2.waitKey(1)

                # print("%s seconds" % (time.time() - start_time))
                time.sleep(round((time.time() - start_time), 1))
            rate.sleep()
    
    def camera_callback(self,data):
        try:
            cv_img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_img

    def states_callback(self,data):
        self.robot_orientation = data.pose[2].orientation
        euler = euler_from_quaternion((self.robot_orientation.x, self.robot_orientation.y, 
                                        self.robot_orientation.z, self.robot_orientation.w))
        yaw = degrees(euler[2])
        print("%f yaw" % yaw)


def main():
    try:
        Yaw()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
