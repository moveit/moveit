Joystick Control of Rviz Interactive Markers
@@@@@@@@@@@@@@@@@@@@@@@@@@@@

Run
==================
First, enable "Allow External Comm." checkbox at ``Planning`` tab.


Second, please execute following command::

   roslaunch moveit_ros_visualization joy_sample.launch

If your joystick is connected to ``/dev/input/foo``, please use ``DEV`` argument::

   roslaunch moveit_ros_visualization joy_sample.launch DEV:=/dev/input/foo


The script ``moveit_joy.py`` can read three types of joy stick:

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
