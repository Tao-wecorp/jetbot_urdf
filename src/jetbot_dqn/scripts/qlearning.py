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
import tensorflow as tf
import tensorflow.contrib.slim as 

# State
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
pose = Pose()

#rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: robot, pose: { position: { x: 1, y: 0, z: 2 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'
state = ModelState()   
state.model_name = "robot"
state.pose = pose
state.reference_frame = world

# Action
stop = [0, 0]
forward = [40, 40]
left = [40, -40]
right = [-40, 40]
bacward = [-40, -40]
actions = [stop, forward, left, right, bacward]