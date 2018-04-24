# About
The aim of the project is to design and implement an  pathfinding algorithm to be used with the turtlebot 3 -robot. The implemented algorithm is set to be ran on a simulated environment, which is ran on a computer running ubuntu and the ROS operating system. it is a student project belonging to the Software Factory Project course of Oulu University.

# Instalation 
In order to run this software you need to install the following softwares
- Ubuntu 16.04
- ROS (Kinetic distribution)
- Gazebo simulator 
- Turtlebot 3 packages on ROS Kinetic

# How to use
1. create a package in catkin workspace 
2. clone all files from the FinalRelease folder into "script" folder of package you created in catkin workspace
3. Use the this command to ran the controller.py node. 
   Set Turtlebot 3 model to burber and start Gazebo 
  -export TURTELBOT3_MODEL=burger
  -roslaunch turtlebot3_gazebo turtlebot3_world.launch
   navigate into script folder and run the following nodes
  -python get_loc.py
  -python pathplanner.py
  -rosrun [your package name] controller.py
