^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_manipulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.19 (2014-06-23)
-------------------
* fixes `#461 <https://github.com/ros-planning/moveit_ros/issues/461>` and a potential segfault
* Now check if there is a time for the gripper trajectory.
  If there is a time, use that one on the gripper trajectory, if not, keep
  with the previous strategy of using a default time.
* Contributors: Michael Ferguson, Sammy Pfeiffer

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* fix merge
* refactor how we use params for pick&place
* set the pose frame so we don't get a crash in approach&translate
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* ApproachAndTranslateStage dynamic reconfigure bug fixed.
  The bug shows up in test code, where it becomes apparent that creating a ros::NodeHandle
  in a static initializer makes it very difficult to call ros::init() before creating
  the first NodeHandle.
* Contributors: Dave Hershberger

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------
* Fixed internal comment.
* Contributors: Dave Hershberger

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
