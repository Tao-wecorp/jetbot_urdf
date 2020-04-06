#!/bin/bash

sudo apt-get install python-pip 
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-joint-state-publisher-gui ros-melodic-teleop-twist-keyboard -y

pip2 install imageio==2.6.1
pip2 install tensorflow-gpu==1.14.0 keras==2.3.1
pip2 install cvlib --no-deps
pip2 install requests progressbar imutils  opencv-python

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/melodic/lib" >> ~/.bashrc
source ~/.bashrc