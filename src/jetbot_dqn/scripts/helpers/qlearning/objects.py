#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from std_msgs.msg import Float64

import numpy as np
from math import *
import time

# left and right velocity from 1 to 10
ACTIONMAT = np.array([0, 1, -1])
reward = 0.0


class QLearning():
    def __init__(self):
        self.state = 0.0  # [angle]
        # self.position = [0.0, 0.0]  # [person_x, person_y]
        self.yaw = 0.0 # [angle]
        self.goal = 0.0  # [yaw_angle]
        self.full_angel = 0.0  # [max_angle]
        self.reward = 0.0  # point

        self.pub_vel_left = rospy.Publisher('/robot/joint1_velocity_controller/command', Float64, queue_size=5)
        self.pub_vel_right = rospy.Publisher('/robot/joint2_velocity_controller/command', Float64, queue_size=5)
        self.sub_state = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)

    def setState(self, state):
        self.state = state
    
    def setGoal(self, position):
        self.goal = degrees(atan(float(position[0]-320)/(480-position[1])))
        self.full_angel =  degrees(atan(float(320)/(480-position[1])))
       
    def calcReward(self):
        self.reward  = (1-abs(float(self.goal-self.state)/self.goal))*100
        # print self.reward
        return self.reward
    
    def step(self, action, learning_rate):
        # self.pub_vel_left.publish(action[0])
        # self.pub_vel_right.publish(action[1])
        new_state = self.state + (action * learning_rate)

        new_state = max(new_state, self.full_angel*(-1))
        new_state = min(new_state, self.full_angel)

        self.setState(new_state)
        print ("state: "), state
        reward = self.calcReward()
        print ("reward: "), reward
        return self.state, reward

    def state_callback(self,data):
        self.robot_orientation = data.pose[2].orientation
        euler = euler_from_quaternion((self.robot_orientation.x, self.robot_orientation.y, 
                                        self.robot_orientation.z, self.robot_orientation.w))
        self.yaw = degrees(euler[2])
        return self.yaw


state = 0.0
pre_state = state
learningRate = 1
q = QLearning()
q.setState(state)
position = [200, 240]
goal = degrees(atan(float(position[0]-320)/(480-position[1])))
print goal
q.setGoal(position)

count = 0 
reward= 0.0

to_goal = goal - state
pre_to_goal = to_goal
curve = []

while reward < 98:
    index = np.random.randint(0,ACTIONMAT.shape[0])
    action = ACTIONMAT[index]
    print "action: ", action
    state,reward = q.step(action,learningRate)
    to_goal = goal - state
    if abs(to_goal) > abs(pre_to_goal):
        # if the new reward is worse than the old reward, throw this state away
        #print("old state",oldState,state,d2g,oldd2g)
        state = pre_state
    q.setState(state)
    count +=1
    pre_to_goal = to_goal
    pre_state = state
print count