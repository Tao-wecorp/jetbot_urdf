#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cvlib as cv
from cvlib.object_detection import draw_bbox
import math
from math import atan2
import numpy as np
import random
import time
import itertools

from copy import deepcopy

import rospkg
rospack = rospkg.RosPack()
import os
import sys
helpers_folder = os.path.join(rospack.get_path("jetbot_dqn"), "scripts/helpers")
sys.path.append(helpers_folder)

from openpose import OpenPose
openpose = OpenPose()
x_fpv, y_fpv = [320, 480]

class Pose(object):
    def __init__(self):
        rospy.init_node('pose_node', anonymous=True)

        self.image_sub = rospy.Subscriber("/robot/camera/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.frame is not None:
                start_time = time.time()
                frame = deepcopy(self.frame)

                x_hip, y_hip = openpose.detect(frame)[11]
                frame = cv2.circle(frame, (int(x_hip), int(y_hip)), 3, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)

                frame = cv2.circle(frame, (int(x_fpv), int(y_fpv)), 5, (255, 0, 255), thickness=-1, lineType=cv2.FILLED)
                cv2.imshow("", frame)
                cv2.waitKey(1)

                print("%s seconds" % (time.time() - start_time))
            rate.sleep()
    
    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_image


def main():
    try:
        Pose()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()