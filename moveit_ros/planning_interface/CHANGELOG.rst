^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_planning_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.18 (2020-01-24)
-------------------

0.9.17 (2019-07-09)
-------------------
* Add missing run dependency to eigenpy
* Contributors: Robert Haschke

0.9.16 (2019-06-29)
-------------------
* [maintanance] Resolve catkin lint issues (`#1137 <https://github.com/ros-planning/moveit/issues/1137>`_)
* [maintanance] Improve clang-format (`#1214 <https://github.com/ros-planning/moveit/issues/1214>`_)
* [feature]     Add getMoveGroupClient() to move_group_interface (`#1215 <https://github.com/ros-planning/moveit/issues/1215>`_)
* Contributors: Ludovic Delval, Martin Günther, Robert Haschke, Ryosuke Tajima

0.9.15 (2018-10-29)
-------------------
* [improvement] Exploit the fact that our transforms are isometries (instead of general affine transformations). `#1091 <https://github.com/ros-planning/moveit/issues/1091>`_
* Contributors: Robert Haschke

0.9.14 (2018-10-24)
-------------------

0.9.13 (2018-10-24)
-------------------
* [capability] Added plan_only flags to pick and place (`#862 <https://github.com/ros-planning/moveit/issues/862>`_)
* [maintenance] Python3 support (`#1103 <https://github.com/ros-planning/moveit/issues/1103>`_, `#1054 <https://github.com/ros-planning/moveit/issues/1054>`_)
* [fix] optional namespace args (`#929 <https://github.com/ros-planning/moveit/issues/929>`_)
* Contributors: David Watkins, Michael Görner, Mohmmad Ayman, Robert Haschke, mike lautman

0.9.12 (2018-05-29)
-------------------
* [maintenance] switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* [capability] namespace to moveit_commander (`#835 <https://github.com/ros-planning/moveit/issues/835>`_)
* Constrained Cartesian planning using moveit commander (`#805 <https://github.com/ros-planning/moveit/issues/805>`_)
* Simplify adding CollisionObjects with colors (`#810 <https://github.com/ros-planning/moveit/issues/810>`_)
* support TrajectoryConstraints in MoveGroupInterface + MoveitCommander (`#793 <https://github.com/ros-planning/moveit/issues/793>`_)
* Add API to get planner_id (`#788 <https://github.com/ros-planning/moveit/issues/788>`_)
* Allow wait time to be specified for getCurrentState() (`#685 <https://github.com/ros-planning/moveit/issues/685>`_)
* Contributors: 2scholz, Akiyoshi Ochiai, Bence Magyar, Dave Coleman, Ian McMahon, Robert Haschke, Will Baker, Xiaojian Ma, srsidd

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] MoveGroupInterface: Fixed computeCartesianPath to use selected end-effector. (`#580 <https://github.com/ros-planning/moveit/issues/580>`_)
* [capability][kinetic onward] Adapt pick pipeline to function without object (`#599 <https://github.com/ros-planning/moveit/issues/599>`_)
* [improve] Disabled copy constructors and added a move constructor to MoveGroupInterface (`#664 <https://github.com/ros-planning/moveit/issues/664>`_)
* Contributors: 2scholz, Dennis Hartmann, Jonathan Meyer, Simon Schmeisser

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [improve] MoveGroupInterface: add public interface to construct the MotionPlanRequest (`#461 <https://github.com/ros-planning/moveit/issues/461>`_)
* Contributors: Michael Goerner

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* [enhancement][MoveGroup] Add getLinkNames function (`#440 <https://github.com/ros-planning/moveit/issues/440>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] move_group.cpp: seg fault bug (`#426 <https://github.com/ros-planning/moveit/issues/426>`_)
* [fix] mgi: show correct include path in doxygen (`#419 <https://github.com/ros-planning/moveit/issues/419>`_)
* [fix] fix race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [fix] issue `#373 <https://github.com/ros-planning/moveit/issues/373>`_ for Kinetic (`#377 <https://github.com/ros-planning/moveit/issues/377>`_) (`#385 <https://github.com/ros-planning/moveit/issues/385>`_)
* [capability] PSI: add apply* functions that use ApplyPlanningScene.srv (`#381 <https://github.com/ros-planning/moveit/issues/381>`_)
* [maintenance] Fix test file issues (`#415 <https://github.com/ros-planning/moveit/pull/415>`_, `#412 <https://github.com/ros-planning/moveit/issues/412>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Bastian Gaspers, Dave Coleman, Isaac I.Y. Saito, Jorge Santos Simon, Michael Goerner, Robert Haschke

0.9.3 (2016-11-16)
------------------

0.6.6 (2016-06-08)
------------------
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* merge indigo-devel changes (PR `#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* planning_interface::MoveGroup::get/setPlannerParams
* new method MoveGroup::getDefaultPlannerId(const std::string &group)
  ... to retrieve default planner config from param server
  moved corresponding code from rviz plugin to MoveGroup interface
  to facilitate re-use
* fixing conflicts, renaming variable
* Merge pull request `#589 <https://github.com/ros-planning/moveit_ros/issues/589>`_ from MichaelStevens/set_num_planning_attempts
  adding set_num_planning_attempts to python interface
* comments addressed
* Added python wrapper for setMaxVelocityScalingFactor
* saves robot name to db from moveit. also robot name accessible through robot interface python wrapper
* adding set_num_planning_attempts to python interface
* Merge pull request `#571 <https://github.com/ros-planning/moveit_ros/issues/571>`_ from ymollard/indigo-devel
  Added python wrapper for MoveGroup.asyncExecute()
* Added python wrapper for MoveGroup.asyncExecute()
* Add retime_trajectory to moveit python wrapper
* add getHandle to move_group_interface
* Updated documentation on move() to inform the user that an asynchronus spinner is required. Commonly new users don't do this and move() blocks permanently
* Contributors: Dave Coleman, Dave Hershberger, Isaac I.Y. Saito, Kei Okada, Michael Stevens, Robert Haschke, Sachin Chitta, Scott, Yoan Mollard, dg, ferherranz

0.6.5 (2015-01-24)
------------------
* update maintainers
* Add time factor support for iterative_time_parametrization
* Contributors: Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* include correct ``boost::*_ptr`` class for boost 1.57.
* Contributors: v4hn

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Add missing variants of place (PlaceLocation, place anywhere) for python interface
* Python wrapper for getEndEffectorTips()
* Contributors: Dave Coleman, Sachin Chitta, corot

0.5.19 (2014-06-23)
-------------------
* Add check for planning scene monitor connection, with 5 sec delay
* Contributors: Dave Coleman

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* added move_group python interface bindings to move group interface
  function:
  void setPathConstraints(const moveit_msgs::Constraint &constraint)
  in order to be able to set path constraints from python scripts
  directly and no need to use the DB.
* Use member NodeHandle in action clients.
  Currently services and topics are already using the member NodeHandle instance,
  but not the action clients.
  This is relevant for two reasons:
  - Consistency in the resulting ROS API namespace (everything in the same namespace).
  - Consistency in the spinning policy. All services, topics and actions will be spinned
  by the same NodeHandle, and whatever custom (or not) spinners and callback queues it
  has associated.
* adding error code returns to relevant functions
* Contributors: Adolfo Rodriguez Tsouroukdissian, Emili Boronat, Ioan A Sucan, Sachin Chitta

0.5.16 (2014-02-27)
-------------------
* adding node handle to options in move_group_interface
* adding get for active joints
* Contributors: Sachin Chitta

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* add API for setting the number of motion plans to be evaluated via the MoveGroupInterface
* move_group_interface: improve documentation
* Contributors: Acorn Pooley, Ioan Sucan

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------
* Fixed bug in computeCartesianPathPython.
* Adding collision object interface to planning_scene interface.
* Contributors: Acorn Pooley, Sachin Chitta

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* Fixed doxygen function-grouping.
* Added planning feedback to gui, refactored states tab

0.5.8 (2013-10-11)
------------------
* add function to start state monitor in move_group_interface::MoveGroup

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------
* update planning options

0.5.5 (2013-09-23)
------------------
* add support for setting joint targets from approximate IK
* specifies python version 2.7 for linking (fixes `#302 <https://github.com/ros-planning/moveit_ros/issues/302>`_)
* use new messages for pick & place
* expand functionality of MoveGroupInterface
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make pick more general
* use message serialization for python bindings
* remove CollisionMap, expose topic names in PlanningSceneMonitor, implement detach / attach operations as requested by `#280 <https://github.com/ros-planning/moveit_ros/issues/280>`_
* make headers and author definitions aligned the same way; white space fixes

0.5.2 (2013-07-15)
------------------
* move msgs to common_msgs

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* some refactoring
