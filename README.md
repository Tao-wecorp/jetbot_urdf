# Jetbot Gazebo model in ROS

## To-do
1. Add Jetbot SDF file to gazebo library;
2. Launch Jetbot URDF in ROS;
3. Launch worlds in ROS.

## Env
    sudo apt-get install python-pip ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-joint-state-publisher-gui
    sudo apt-get install ros-melodic-teleop-twist-keyboard
    pip2 install imageio==2.6.1
    pip2 install tensorflow-gpu==1.14.0 keras==2.3.1
    pip2 install cvlib --no-deps
    pip2 install requests progressbar imutils  opencv-python

## Run
1. Replace all "home/kimbring2/catkin_ws/src/jetbot/jetbot_gazebo" with "home/robotics/Workspace/jetbot_cadrl/src/jetbot_gazebo/jetbot/jetbot_gazebo";
2. chmod +x all python files under scripts before rosrun.
