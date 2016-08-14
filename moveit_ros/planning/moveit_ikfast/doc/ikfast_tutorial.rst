Creating a custom IKFast Plugin
===============================
In this section, we will walk through configuring an IKFast plugin for MoveIt!

What is IKFast?
^^^^^^^^^^^^^^^
*From Wikipedia:*
IKFast, the Robot Kinematics Compiler, is a powerful inverse kinematics solver provided within Rosen Diankov's OpenRAVE motion planning software. Unlike most inverse kinematics solvers, IKFast can analytically solve the kinematics equations of any complex kinematics chain, and generate language-specific files (like C++) for later use. The end result is extremely stable solutions that can run as fast as 5 microseconds on recent processors

MoveIt! IKFast
^^^^^^^^^^^^^^
MoveIt! IKFast is a tool that generates a IKFast kinematics plugin for MoveIt using OpenRave generated cpp files. 
This tutorial will step you through setting up your robot to utilize the power of IKFast. MoveIt! IKFast is tested on ROS Groovy with Catkin using OpenRave 0.8 with a 6dof and 7dof robot arm manipulator. 
While it works in theory, currently the IKFast plugin generator tool does not work with >7 degree of freedom arms.

Pre-requisites
^^^^^^^^^^^^^^
You should have already created a MoveIt! configuration package for your robot, by using the Setup Assistant.

MoveIt! IKFast Installation
^^^^^^^^^^^^^^^^^^^^^^^^^^^
Install the MoveIt! IKFast package either from debs or from source.

**Binary Install** ::

 sudo apt-get install ros-indigo-moveit-ikfast

**Source**

Inside your catkin workspace ::

 git clone https://github.com/ros-planning/moveit_ikfast.git

OpenRAVE Installation
^^^^^^^^^^^^^^^^^^^^^
OpenRave is available for Ubuntu using these commands::

 sudo add-apt-repository ppa:openrave/release
 sudo apt-get update
 sudo apt-get install openrave0.8-dp-ikfast

More detailed and updated instructions are available at `OpenRave.org <http://openrave.org/docs/latest_stable/install/#install>`_.

**Note:** You may need to use this hack reported by MoveIt! users:

Edit /usr/lib/python2.7/dist-packages/openravepy/__init__.py to add the following line just after the copyright::

 __openravepy_version__ = "0.8"

**Note:** Use ''openrave0.8'' instead of ''openrave'' in the following tutorial if installed from packages

*Please report your results with this on the moveit-users mailing list.*


Create Collada File For Use With OpenRave
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First you will need robot description file that is in `Collada or OpenRave <http://openrave.org/docs/latest_stable/collada_robot_extensions/>`_ robot format.

If your robot is not in this format we recommend you create a ROS `URDF <http://www.ros.org/wiki/urdf/Tutorials/Create%20your%20own%20urdf%20file>`_ file, then convert it to a Collada .dae file using the following command::

 rosrun collada_urdf urdf_to_collada <myrobot_name>.urdf <myrobot_name>.dae

where <myrobot_name> is the name of your robot.

Often floating point issues arrise in converting a URDF file to Collada file, so a script has been created to round all the numbers down to x decimal places in your .dae file. Its probably best if you skip this step initially and see if IKFast can generate a solution with your default values, but if the generator takes longer than, say, an hour, try the following::

 rosrun moveit_ikfast round_collada_numbers.py <input_dae> <output_dae> <decimal places>

From experience we recommend 5 decimal places, but if the OpenRave ikfast generator takes to long to find a solution, lowering the number of decimal places should help. For example::

 rosrun moveit_ikfast round_collada_numbers.py <myrobot_name>.dae <myrobot_name>.rounded.dae 5

To see the links in your newly generated Collada file::

 /usr/bin/openrave-robot.py <myrobot_name>.dae --info links

This is useful if you have a 7-dof arm and you need to fill in a --freeindex parameter, discussed later.

To test your newly generated Collada file in OpenRave::

 openrave <myrobot_name>.dae

Create IKFast Solution CPP File
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Once you have a numerically rounded Collada file its time to generate the C++ .h header file that contains the analytical IK solution for your robot.

Select IK Type
--------------
You need to choose which sort of IK you want. See `this page <http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types>`_ for more info.  The most common IK type is *transform6d*.

Choose Planning Group
---------------------
If your robot has more than one arm or "planning group" that you want to generate an IKFast solution for, choose one to generate first. The following instructions will assume you have chosen one <planning_group_name> that you will create a plugin for. Once you have verified that the plugin works, repeat the following instructions for any other planning groups you have. For example, you might have 2 planning groups::

 <planning_group_name> = "left_arm"
 <planning_group_name> = "right_arm"

Identify Link Numbers
---------------------

You also need the link index numbers for the *base_link* and *end_link* between which the IK will be calculated. You can count the number of links by viewing a list of links in your model::

 openrave-robot.py <myrobot_name>.dae --info links

**NOTE**: use ''openrave0.8-robot.py'' if installed from packages

A typical 6-DOF manipulator should have 6 arm links + a dummy base_link as required by ROS specifications.  If no extra links are present in the model, this gives: *baselink=0* and *eelink=6*.  Often, an additional tool_link will be provided to position the grasp/tool frame, giving *eelink=7*.

The manipulator below also has another dummy mounting_link, giving *baselink=1* and *eelink=8*.

=============  ======  =======
name           index   parents
=============  ======  =======
base_link      0
mounting_link  1       base_link
link1_rotate   2       mounting_link
link2          3       link1_rotate
link3          4       link2
link4          5       link3
link5          6       link4
link6_wrist    7       link5
tool_link      8       link6_wrist
=============  ======  =======

Generate IK Solver
^^^^^^^^^^^^^^^^^^

To generate the IK solution between the manipulator's base and tool frames for a 6 dof arm, use the following command format::

 python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=<myrobot_name>.dae --iktype=transform6d --baselink=1 --eelink=8 --savefile=<ikfast_output_path>

where <ikfast_output_path> is recommended to be a path that points to a file named ikfast61_<planning_group_name>.cpp.

For a 7 dof arm, you will need to specify a free link::

 python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=<myrobot_name>.dae --iktype=transform6d --baselink=1 --eelink=8 --freeindex=4 --savefile=<ikfast_output_path>

The speed and success of this process will depend on the complexity of your robot. A typical 6 DOF manipulator with 3 intersecting axis at the base or wrist will take only a few minutes to generate the IK.

You should consult the OpenRAVE mailing list and ROS Answers for information about 5 and 7 DOF manipulators.

Create Plugin
^^^^^^^^^^^^^

Create the package that will contain the IK plugin. We recommend you name the package <myrobot_name>_ikfast_<planning_group_name>_plugin. From here on out we'll refer to your IKFast package as simply <moveit_ik_plugin_pkg>::

 cd ~/catkin_ws/src
 catkin_create_pkg <moveit_ik_plugin_pkg>

Build your workspace so the new package is detected (can be 'roscd')::

 cd ~/catkin_ws
 catkin_make

Create the plugin source code::

 rosrun moveit_ikfast create_ikfast_moveit_plugin.py <myrobot_name> <planning_group_name> <moveit_ik_plugin_pkg> <ikfast_output_path>

Or without ROS::

 python /path/to/create_ikfast_moveit_plugin.py <myrobot_name> <planning_group_name> <moveit_ik_plugin_pkg> <ikfast_output_path>

Parameters
^^^^^^^^^^
 * *myrobot_name* - name of robot as in your URDF
 * *planning_group_name* - name of the planning group you would like to use this solver for, as referenced in your SRDF and kinematics.yaml
 * *moveit_ik_plugin_pkg* - name of the new package you just created
 * *ikfast_output_path* - file path to the location of your generated IKFast output.cpp file

This will generate a new source file <myrobot_name>_<planning_group_name>_ikfast_moveit_plugin.cpp in the src/ directory, and modify various configuration files.

Build your workspace again to create the ik plugin::

 cd ~/catkin_ws
 catkin_make

This will build the new plugin library lib/lib<myrobot_name>_<planning_group_name>_moveit_ikfast_moveit_plugin.so that can be used with MoveIt!

Usage
^^^^^
The IKFast plugin should function identically to the default KDL IK Solver, but with greatly increased performance. The MoveIt configuration file is automatically edited by the moveit_ikfast script but you can switch between the KDL and IKFast solvers using the *kinematics_solver* parameter in the robot's kinematics.yaml file ::

 rosed <myrobot_name>_moveit_config/config/kinematics.yaml

Edit these parts::

 <planning_group_name>:
   kinematics_solver: <moveit_ik_plugin_pkg>/IKFastKinematicsPlugin
 -OR-
   kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin

Test the Plugin
^^^^^^^^^^^^^^^

Use the MoveIt Rviz Motion Planning Plugin and use the interactive markers to see if correct IK Solutions are found.

Updating the Plugin
^^^^^^^^^^^^^^^^^^^

If any future changes occur with MoveIt! or IKFast, you might need to re-generate this plugin using our scripts. To allow you to easily do this, a bash script is automatically created in the root of your IKFast package, named *update_ikfast_plugin.sh*. This does the same thing you did manually earlier, but uses the IKFast solution header file that is copied into the ROS package.

Links
=====

 * Back to :moveit_website:`Moveit! Tutorials <tutorials>`


