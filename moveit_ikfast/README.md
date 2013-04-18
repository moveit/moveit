MoveIt! IKFast Converter
==========
* Author: Dave Coleman, CU Boulder; Jeremy Zoss, SwRI; David Butterworth, KAIST
* Date: 3/19/2013
* Version: 3.0.0

Generates a IKFast kinematics plugin for MoveIt using OpenRave generated cpp files. 
 
You should have already created a MoveIt! package for your robot, by using the Setup Assistant and following this [tutorial](http://moveit.ros.org/wiki/index.php/Groovy/MoveIt!_Setup_Assistant).

Tested on ROS Groovy with Catkin from OpenRave 0.8 using a 6dof manipulator. Attempting to make it work with 7dof.

Create Collada File For Use With OpenRave
---------

First you will need robot description file that is in [Collada or OpenRave](http://openrave.org/docs/latest_stable/collada_robot_extensions/) robot format. 

If your robot is not in this format we recommend you create a ROS [URDF](http://www.ros.org/wiki/urdf/Tutorials/Create%20your%20own%20urdf%20file) file, then convert it to a Collada .dae file using the following command:

      rosrun collada_urdf urdf_to_collada YOURROBOT.urdf YOURROBOT.dae

Often floating point issues arrise in converting a URDF file to Collada file, so a script has been created to round all the numbers down to x decimal places in your .dae file. From experience we recommend 5 decimal places:

      rosrun moveit_ikfast_converter round_collada_numbers.py <input_dae> <output_dae> <decimal places>

For example

      rosrun moveit_ikfast_converter round_collada_numbers.py YOURROBOT.dae YOURROBOT.rounded.dae 5

To see the links in your newly generated Collada file (assuming you have openrave installed):

      /usr/bin/openrave-robot.py YOURROBOT.dae --info links

To test your newly generated Collada file in OpenRave (assuming you have openrave installed):

      openrave YOURROBOT.dae


Create IKFast Solution CPP File
---------
Once you have a rounded Collada file see [ROS Industrial Tutorial](http://www.ros.org/wiki/Industrial/Tutorials/Create_a_Fast_IK_Solution)

Create Plugin
---------

Plugin Package Location:

       catkin_create_pkg myrobot_moveit_plugins	

Build your workspace so the new package is detected (can be 'roscd')

      cd YOUR_ROS_WORKSPACE
      catkin_make

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

      roscd my_robot_moveit_config
      <edit> config/kinematics.yaml

Edit these parts:

     manipulator:
       kinematics_solver: my_robot_manipulator_kinematics/IKFastKinematicsPlugin
        -OR-
       kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin

Test the Plugin
---------

Use the [MoveIt Rviz Motion Planning Plugin](http://moveit.ros.org/wiki/index.php/Groovy/PR2/Rviz_Plugin/Quick_Start) and use the interactive markers to see if correct IK Solutions are found
