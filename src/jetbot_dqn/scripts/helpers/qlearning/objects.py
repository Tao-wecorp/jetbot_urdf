#! /usr/bin/env python

import numpy as np
from math import *
import sys
import time
import matplotlib.pyplot as mp


class QLearning():
    def __init__(self):
        self.state = 0.0  # [angle]
        self.yaw = 0.0 # [angle]
        self.goal = 0.0  # [yaw_angle]
        self.full_angel = 0.0  # [max_angle]
        self.reward = 0.0  # point

    def setState(self, state):
        self.state = state
    
    def setGoal(self, goal):
        self.goal = goal
        # self.full_angel =  degrees(atan(float(320)/(480-position[1])))
       
    def calcReward(self):
        self.reward  = (1-abs(float(self.goal-self.state)/(self.goal+sys.float_info.epsilon)))*100
        return self.reward
    
    def step(self, action, learning_rate):
        new_state = self.state + (action * learning_rate)

        new_state = max(new_state, -90)
        new_state = min(new_state, 90)

        self.setState(new_state)
        reward = self.calcReward()
        return self.state, reward

def pos_to_ang(position):
    angle = degrees(atan(float(position[0]-320)/(480-position[1])))
    return angle


q = QLearning()
# set sate
state = 0.0
pre_state = state
q.setState(state)
# set goal
ACTIONMAT = np.array([0, 1, -1])
position = [450, 240]
goal = pos_to_ang(position)
q.setGoal(goal)
to_goal = goal - state
pre_to_goal = to_goal
# init
count = 0 
reward= 0.0
learningRate = 1
actions = []
states = []
curve_reward = []
curve_learning_rate = []


while reward < 96:
    goal = goal + 0.2
    q.setGoal(goal)
    index = np.random.randint(0,ACTIONMAT.shape[0])
    action = ACTIONMAT[index]
    actions.append(action)
    state, reward = q.step(action,learningRate)
    states.append(state)
    learningRate = (100-reward)/3 # adaptive learning rate
    to_goal = goal - state

    if abs(to_goal) > abs(pre_to_goal):
        state = pre_state
    q.setState(state)
    count +=1
    pre_to_goal = to_goal
    pre_state = state
    curve_reward.append(reward)
    curve_learning_rate.append(learningRate)
    
print "Epoch ", count, " Goal: ", goal, " State: ", state
print states
mp.plot(curve_reward)
mp.plot(curve_learning_rate)
mp.show()