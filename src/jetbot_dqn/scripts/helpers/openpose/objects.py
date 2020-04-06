#! /usr/bin/env python

import cv2
import os
import rospy
import rospkg
import time

rospack = rospkg.RosPack()
openpose_folder = os.path.join(rospack.get_path("jetbot_dqn"), "scripts/helpers/openpose/models/")
net = cv2.dnn.readNetFromTensorflow(openpose_folder + "graph_opt.pb")
nPoints = 18
threshold = 0.1
inputSize = 300

class OpenPose():
    def detect(self, cv_image):
        frameWidth = cv_image.shape[1]
        frameHeight = cv_image.shape[0]

        net.setInput(cv2.dnn.blobFromImage(cv_image, 1.0, (inputSize, inputSize), (127.5, 127.5, 127.5), swapRB=True, crop=False))
        output = net.forward()
        output = output[:, :nPoints, :, :]

        H = output.shape[2]
        W = output.shape[3]

        points = []
        for i in range(nPoints):
            probMap = output[0, i, :, :]
            minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)
            x = (frameWidth * point[0]) / W
            y = (frameHeight * point[1]) / H

            if (prob > threshold):
                points.append((int(x), int(y)))
            else :
                points.append(None)
        return points