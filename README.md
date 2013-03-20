MoveIt! IKFast Converter
==========
* Authors:	Dave Coleman <davetcoleman.com>
  		Based heavily on the arm_kinematic_tools package by Jeremy Zoss, SwRI
		and the arm_navigation plugin generator by David Butterworth, KAIST
* Date: 3/19/2013
* Version: 3.0.0

Create an IKFast Plugin for MoveIt!
---------

Generates a IKFast kinematics plugin for MoveIt using OpenRave generated cpp files
 
You should have already created a MoveIt! package for your robot, by using the Setup Assistant and following this Tutorial.

Tested on ROS Groovy with Catkin from OpenRave 0.8 using a 7dof manipulator

Create IKFast Solition CPP File
---------

This is required first before creating the plugin. See tutorial http://www.ros.org/wiki/Industrial/Tutorials/Create_a_Fast_IK_Solution


Create Plugin
---------

Plugin Package Location:

	catkin_create_pkg myrobot_moveit_plugins	

Create the plugin source code:

       rosrun moveit_ikfast_converter create_ikfast_moveit_plugin.py <yourobot_name> <planning_group_name> <moveit_plugin_pkg> <ikfast_output_path>

Or without ROS:

       python /path/to/create_ikfast_moveit_plugin.py <yourobot_name> <planning_group_name> <moveit_plugin_pkg> <ikfast_output_path>

**Parameters**
yourobot_name - name of robot as in your URDF
planning_group_name - name of the planning group you would like to use this solver for, as referenced in your SRDF and kinematics.yaml
moveit_plugin_pkg - name of the new package you just created - e.g. myrobot_moveit_plugins
ikfast_output_path - file path to the location of your generated IKFast output.cpp file

This will generate a new source file ROBOTNAME_GROUPNAME_ikfast_moveit_plugin.cpp in the src/ directory, and modify various configuration files.

Build the plugin library:

      cd YOUR_ROS_WORKSPACE
      catkin_make

This will build the new plugin library lib/lib<robot_name>_moveit_arm_kinematics.so.


Usage
---------

The IKFast plugin should function identically to the default KDL IK Solver, but with greatly increased performance. You can switch between the KDL and IKFast solvers using the kinematics_solver parameter in the robot's kinematics.yaml file:

$ roscd my_robot_moveit_config
$ <edit> config/kinematics.yaml

manipulator:
  kinematics_solver: my_robot_manipulator_kinematics/IKFastKinematicsPlugin
      -OR-
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin


Test the Plugin
---------

Use the MoveIt Rviz Motion Planning Plugin and use the interactive markers to see if correct IK Solutions are found
