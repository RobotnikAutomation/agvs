agvs
====

ROS package for the robot AGVS, intended for indoor transportation tasks

List of packages

### agvs_complete

It contains multiple launch files to perform different tasks, from creating a map with gmapping to launching amcl.

### agvs_description

Robot description. Urdf and mesh files.

### agvs_pad            

Component to move the robot by using a ps3 pad.

### planner_msgs

Messages and actions for planning the autonomous movement of the robot.

### agvs_control

Config files used for Gazebo motor controllers.

### agvs_gazebo 

Launch files and worlds to run Gazebo.

### agvs_robot_control

Robot controller that interacts with Gazebo motor controllers.

### purepursuit_planner

Planner to follow a list of waypoints implementing the PurePursuit algorithm.
