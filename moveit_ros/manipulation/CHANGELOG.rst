^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_manipulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.14 (2018-10-20)
-------------------
* [fix] race conditions when updating PlanningScene `#350 <https://github.com/ros-planning/moveit/issues/350>`_
* Contributors: Michael Görner, Robert Haschke

0.7.13 (2017-12-25)
-------------------

0.7.12 (2017-08-06)
-------------------

0.7.11 (2017-06-21)
-------------------

0.7.10 (2017-06-07)
-------------------

0.7.9 (2017-04-03)
------------------
* [fix][moveit_ros_manipulation] Set planning frame correctly in evaluation of reachable and valid pose filter (`#476 <https://github.com/ros-planning/moveit/issues/476>`_)
* Contributors: Yannick Jonetzko

0.7.8 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* Contributors: Dmitry Rozhkov

0.7.7 (2017-02-06)
------------------
* [maintenance] clang-format upgraded to 3.8 (`#404 <https://github.com/ros-planning/moveit/issues/404>`_)
* Contributors: Dave Coleman

0.7.6 (2016-12-30)
------------------

0.7.5 (2016-12-25)
------------------

0.7.4 (2016-12-22)
------------------

0.7.3 (2016-12-20)
------------------

0.7.2 (2016-06-20)
------------------

0.7.1 (2016-04-11)
------------------

0.7.0 (2016-01-30)
------------------
* Removed trailing whitespace from entire repository
* Contributors: Dave Coleman

0.6.5 (2015-01-24)
------------------
* update maintainers
* Contributors: Michael Ferguson

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* trivial fixes for warnings
* use named logging for manipulation components
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Fix bug in place-planning where attached object was not considered in plan.
* Contributors: Dave Hershberger, Sachin Chitta

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
