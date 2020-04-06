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

from helpers.openpose import OpenPose
openpose = OpenPose()
x_fpv, y_fpv = [320, 480]

from helpers.qlearning import QLearning
q = QLearning()