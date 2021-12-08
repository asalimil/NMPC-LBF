# NMPC-LBF

This repository includes the python codes for implementation of NMPC-LBF method.

Please install the following packages and softwares in python:

$ pip install casadi

$ pip install scipy

$ pip install tensorflow

$ pip install keras


You also need to install turtlebot3 simulation packages which are available in this link:

https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/


First, you need to create a ROS package in your workspace (~/catkin_ws) and make it:

$ catkin_create_package nmpc_lbf rospy

$ cd ~/catkin_ws

$ catkin_make 


To run the simulation in an empty world:

1) $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

2) $ roslaunch nmpc_lbf nmpc_lbf_launch.launch

In order to run the simulation in an environment with the presence of obstacles, you can uncomment the lines 2-5 in launch file.




