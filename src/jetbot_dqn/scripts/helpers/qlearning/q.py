#!/usr/bin/env python3

import numpy as np
from math import *
import matplotlib.pyplot as mp

ACTIONMAT = np.array([-1, 0, 1])
reward = 0.0

class RobotArm():
    def __init__(self):
        self.state = 0.0  # [angle]
        self.position = [0.0, 0.0]  # [x, y]
        self.goal = 0.0  # [angle]
        self.angle2goal = 0.0  # [angle]
        self.reward = 0.0  # point

    def setState(self, state):
        self.state = state

    def setGoal(self, position):
        goal = degrees(atan(position[0]/position[1]))
        self.goal = goal

    def calcReward(self):
        angle2goal = self.goal-self.state
        self.angle2goal = angle2goal
        reward = (1 - self.angle2goal/self.goal) * 100
        self.reward = reward
        return reward

    def step(self, act, learning_rate):
        new_state = self.state + (act * learning_rate)

        new_state = max(new_state, 0)
        new_state = min(new_state, 90)

        self.setState(new_state)
        reward = self.calcReward()

        return self.state, reward


def stateEqual(state1, state2):
    state1 = state2
    return state1 == state2


def action_sample(mode, state, Qmatrix):
    if mode == "random":
        index = np.random.randint(0, ACTIONMAT.shape[0])
        action = ACTIONMAT[index]
        return index
    if mode == "Q":
        # Qmatrix contains a list of states [0...goal], actions [1/0/-1] and rewards [0...100]

        myStatesQ =[]

        for datum in Qmatrix:
            if stateEqual(datum[0], state):
                myStatesQ.append(datum)
        if len(myStatesQ) == 0:
            action=action_sample('random', state, Qmatrix)
        else:
            maxState=[0, 0, -9999.0]
            for thisStateQ in myStatesQ:
                if thisStateQ[2] > maxState[2]:
                    maxStat = thisStateQ
            if maxState[2] == -9999.0:
                action = action_sample('random', state, Qmatrix)
            else:
                action = maxState[1]
        index = action
    return index


def maxQ(Q, state):
    maxQvalue = 0
    for thisQ in Q:
        thisState=thisQ[0]
        action=thisQ[1]
        qvalue = thisQ[2]
        if stateEqual(state, thisState):
            maxQvalue = max(qvalue, maxQvalue)
    return maxQvalue


def setQ(Q, state, action, value):
    index = 0
    found = False

    for datum in Q:
        try:
            if stateEqual(state, datum[0]) and action == datum[1]:
                Q[index] = [state, action, value]
                found = True
                break
        except:
            print ("except setQ", action, datum)
        index += 1
    if not found:
        Q.append([state, action, value])


state = 0
pre_state = state
learningRate = 5.0
robotArm = RobotArm()
robotArm.setState(state)
goal_position = [10, 10]
robotArm.setGoal(goal_position)
knt = 0  # counter
reward = 0.0
angle2goal = degrees(atan(goal_position[0]/goal_position[1]))
pre_angle2goal = angle2goal
curve = []

# Q learning phase
Q = []
stateReset = 0
state = stateReset
robotArm.setState(state)
gamma = 0.9
G = 0


for epoch in range(1, 100):
    done = False
    G, reward, knt = 0, 0, 0
    state = stateReset
    robotArm.setState(state)
    while not done:
        action = action_sample("Q", state, Q)
        motorAction = ACTIONMAT[action]
        state2, reward = robotArm.step(motorAction, learningRate)
        newQ = reward + gamma * maxQ(Q, state2)
        setQ(Q, state, action, newQ)
        G += reward
        knt += 1
        if knt > 1000 or reward > 99:
            done = True
        state = state2
        robotArm.setState(state)
    if epoch % 2 == 0:
        print("Epoch ", epoch, "TotalReward:", G, " counter:", knt, "Q Len ", len(Q))
