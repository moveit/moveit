Creating Custom MoveIt! Constraint Samplers
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

Overview
========

Some planning problems require more complex or custom constraint 
samplers for more difficult planning problems. This document explains
how to creat a custom motion planning constraint sampler for use 
with MoveIt!.

Pre-requisites
==============

Creating a constraint sampler
---------------

* Create a ROBOT_moveit_plugins package and within that a subfolder for your YOURROBOT_constraint_sampler plugin. 
  Modify the template provided by hrp2jsk_moveit_plugins/hrp2jsk_moveit_constraint_sampler_plugin
* In your ROBOT_moveit_config/launch/move_group.launch file, within the <node name="move_group">, add
  the parameter:
  
    <param name="constraint_samplers" value="YOURROBOT_moveit_constraint_sampler/YOURROBOTConstraintSamplerAllocator"/>

* Now when you launch move_group, it should default to your new constraint sampler.
