Joystick Control of Rviz Interactive Markers
@@@@@@@@@@@@@@@@@@@@@@@@@@@@

Run
==================

Startup regular MoveIt! planning node with Rviz (for example demo.launch)

In the Motion Planning plugin of Rviz, enable "Allow External Comm." checkbox in the ``Planning`` tab.

Now launch the joystick control launch file specific to your robot. If you are missing this file, first re-run the MoveIt! Setup Assistant using the latest version of the Setup Assistant:

  roslaunch YOURROBOT_moveit_config joystick_control.launch

The script defaults to using ``/dev/input/js0`` for your game controller port. To customize, you can also use, for example:
  roslaunch YOURROBOT_moveit_config joystick_control.launch dev:=/dev/input/js1

This script can read three types of joy sticks:

1. XBox360 Controller via USB
2. PS3 Controller via USB
3. PS3 Contrlller via Bluetooth (Please use ps3joy package <http://wiki.ros.org/ps3joy>)

Joystick Command Mappings
==================

Command               | PS3 Controller     | Xbox Controller    |
--------------------- | ------------------ | ------------------ |
+-x/y                 | left analog stick  | left analog stick  |
+-z                   | L2/R2              | LT/RT              |
+-yaw                 | L1/R1              | LB/RB              |
+-roll                | left/right         | left/right         |
+-pitch               | up/down            | up/down            |
change planning group | select/start       | Y/A                |
change end effector   | triangle/cross     | back/start         |
plan                  | square             | X                  |
execute               | circle             | B                  |

Debugging
==================
Add "Pose" to rviz Displays and subscribe to ``/joy_pose`` in order to see the output from joystick.

Note that only planning groups that have IK solvers for all their End Effector parent groups will work.
