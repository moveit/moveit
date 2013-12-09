^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_manipulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* corrected maintainers email
* PickPlace: Added comments, renamed variables to be more specific
* use ROS_ERROR instead of logError

0.5.8 (2013-10-11)
------------------
* fix `#331 <https://github.com/ros-planning/moveit_ros/issues/331>`_.
* try to identify the eef and group based on the attached object name

0.5.7 (2013-10-01)
------------------
* use the fact we know an eef must be defined for the place action to simplify code
* abort place if eef cannot be determined, fixes `#325 <https://github.com/ros-planning/moveit_ros/issues/325>`_.
* fix segfault in approach translate

0.5.6 (2013-09-26)
------------------
* dep on manipulation_msgs needs to be added here

0.5.5 (2013-09-23)
------------------
* use new messages for pick & place
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes
* adding manipulation tab, fixed bugs in planning scene interface

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* bugfixes
