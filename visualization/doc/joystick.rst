Joystick Control of Rviz Interactive Markers
@@@@@@@@@@@@@@@@@@@@@@@@@@@@

Run
==================
Before running moveit stuff, please run ``joy_node`` like::
  rosrun joy joy_node _dev:=/dev/input/js0

First, enable "Allow External Comm." checkbox at ``Planning`` tab.

Second, please execute following command::

   rosrun moveit_ros_visualization moveit_joy.py

The script ``moveit_joy.py`` can read three types of joy sticks:

1. XBox360 Controller via USB
2. PS3 Controller via USB
3. PS3 Contrlller via Bluetooth (Please use ps3joy package <http://wiki.ros.org/ps3joy>)

Joystick Cheat Sheet
==================
::

   left analog stick: +-x/y
   L2/R2: +-z
   L1/R1: +-yaw
   left/right: +-roll
   up/down: +-pitch
   triangle/cross: change planning group
   select/start: change end effector
   square: plan
   circle: execute


Please add "Pose" to rviz Displays and subscribe ``/joy_pose`` in order to see the output from joystick.
