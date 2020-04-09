#! /usr/bin/env python

import numpy as np
from math import *
import time

ACTIONMAT = np.array([-1, 0, 1])
reward = 0.0

# Action
stop = [0, 0]
left = [40, -40]
right = [-40, 40]
actions = [stop, left, right]


class QLearning():
    def __init__(self):
        self.state = 0.0  # [angle]
        self.position = [0.0, 0.0]  # [x, y]
        self.goal = 0.0  # [angle]
        self.full_angel = 0.0  # [angle]
        self.reward = 0.0  # point

    def setState(self, state):
        self.state = state
    
    def setGoal(self, position):
        self.goal = degrees(atan(float(position[0]-320)/(480-position[1])))
        self.full_angel =  degrees(atan(float(320)/(480-position[1])))
       
    def calcReward(self):
        self.reward  = (1-abs(float(new_goal-self.state))/full_angel)*100
    
    def step(self, act, learning_rate):
        new_state = self.state + (act * learning_rate)

        new_state = max(new_state, self.full_angel)
        new_state = min(new_state, self.full_angel*(-1))

        self.setState(new_state)
        reward = self.calcReward()
        return self.state, reward

    def yaw(self, position):
        new_goal = degrees(atan(float(320-position[0])/(480-position[1])))
        yaw = new_goal + self.goal
        self.goal = yaw
        return yaw