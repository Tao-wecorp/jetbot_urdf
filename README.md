# Jetbot Gazebo model in ROS

## To-do
1. Add Jetbot SDF file to gazebo library;
2. Launch Jetbot URDF in ROS;
3. Launch worlds in ROS.

## Env
1. Run ./env.sh

## Run
1. Replace all "home/kimbring2/catkin_ws/src/jetbot/jetbot_gazebo" with "home/robotics/Workspace/jetbot_cadrl/src/jetbot_gazebo/jetbot/jetbot_gazebo";
2. Run chmod +x all python files under scripts before rosrun.
3. Run roslaunch jetbot_description jetbot_rviz.launch
4. Run roslaunch jetbot_gazebo main.launch
5. Run rosrun rqt_gui rqt_gui
6. Run rosrun jetbot_gazebo twist.py