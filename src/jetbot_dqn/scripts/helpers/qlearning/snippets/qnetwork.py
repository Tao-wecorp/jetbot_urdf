import numpy as np
from math import *
import matplotlib.pyplot as mp


class QLearning():
    def __init__(self):
        self.state = 0.0  # [angle]
        self.goal = 0.0  # [angle]
        self.reward = 0.0  # point

    def setState(self, state):
        self.state = state

    def setGoal(self, goal):
        self.goal = goal

    def calcReward(self):
        self.reward = (1 - (self.goal-self.state)/self.goal) * 100.0

    def step(self, act, learning_rate):
        new_state = self.state + (act * learning_rate)

        new_state = max(new_state, 0)
        new_state = min(new_state, 90)

        self.setState(new_state)
        self.calcReward()
        return self.state, self.reward


def stateEqual(state1, state2):
    state1 = state2
    return state1 == state2


def action_sample(mode, state, Qmatrix):
    if mode == "random":
        modes.append(mode)
        index = np.random.randint(0, ACTIONMAT.shape[0])
        action = ACTIONMAT[index]
        return index
    if mode == "Q":
        modes.append(mode)
        # Qmatrix contains a list of states [0...goal], actions [1/0/-1] and rewards [0...100]

        StatesQ = []
        for Q in Qmatrix:
            if stateEqual(Q[0], state):
                StatesQ.append(Q)

        if len(StatesQ) == 0:
            action = action_sample('random', state, Qmatrix)
        else:
            maxState = [0, 0, 0]
            for thisStateQ in StatesQ:
                if thisStateQ[2] > maxState[2]:
                    maxState = thisStateQ
            if maxState[2] == 0:
                action = action_sample('random', state, Qmatrix)
            else:
                action = maxState[1]
        index = action
    return index


def maxQ(Q, state):
    maxQvalue = 0
    for thisQ in Q:
        thisState = thisQ[0]
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


q = QLearning()
# set state
state = 0.0
pre_state = state
q.setState(state)
# set goal
position = [10, 10]
goal = degrees(atan(position[0]/position[1]))
q.setGoal(goal)
# init
ACTIONMAT = np.array([-1, 0, 1])
count = 0
learningRate = 1.0
reward = 0.0 
curve = []
modes = []

# Q learning phase
Q = []
gamma = 0.9
G = 0.0


for epoch in range(1, 5):
    done = False
    G, reward, count = 0, 0, 0
    q.setState(0.0)
    
    while not done:
        goal = goal + 0.2
        q.setGoal(goal)
        index = action_sample("Q", state, Q)
        action = ACTIONMAT[index]
        state2, reward = q.step(action, learningRate)
        newQ = reward + gamma * maxQ(Q, state2)
        setQ(Q, state, index, newQ)
        G += reward
        count += 1
        if count > 10 or reward > 99:
            done = True
        state = state2
        q.setState(state)
        learningRate = (100 - reward) / 3

print modes
print state