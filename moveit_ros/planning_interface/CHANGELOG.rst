^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_planning_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.8 (2017-03-08)
------------------
* [enhancement][MoveGroup] Add getLinkNames function (`#440 <https://github.com/ros-planning/moveit/issues/440>`_)
* [doc][moveit_commander] added description for set_start_state (`#447 <https://github.com/ros-planning/moveit/issues/447>`_)
* Contributors: Dmitry Rozhkov, Isaac I.Y. Saito, henhenhen

0.7.7 (2017-02-06)
------------------
* move_group.cpp: seg fault bug fix (`#426 <https://github.com/ros-planning/moveit/issues/426>`_)
  Fixed a bug with casting CallbackQueueInterface * to ros::CallbackQueue * without a check.
  https://github.com/ros-planning/moveit/issues/425
* install planning_interface python test
  This test is reference in the tutorials and should also work with installed workspaces
  http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/tests.html#integration-test
* [moveit_ros] Missing test_depend. (`#412 <https://github.com/ros-planning/moveit/issues/412>`_)
* clang-format
* PSI: make all logs _NAMED
* PSI: point out synchronicity behavior of apply* and add/removeObjects interfaces
* remove obsolete mutable declaration
  This was only required for the proposal to change the behavior of the add/remove functions.
  Now that I added the apply* interfaces, this is not relevant anymore.
* PSI: add apply* functions that use ApplyPlanningScene.srv
  to update move_group's PlanningScene
* Use ApplyPlanningSceneService in planning_scene_interface
  Added helper function to call ApplyPlanningScene service
  with fallback to asynchronous processing via "planning_scene"
  topic.
* clang-format upgraded to 3.8 (`#404 <https://github.com/ros-planning/moveit/issues/404>`_)
* Contributors: Andreas Köpf, Bastian Gaspers, Dave Coleman, Isaac I.Y. Saito, v4hn

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
* Issue `#630 <https://github.com/ros-planning/moveit_ros/issues/630>`_: remove color information from addCollisionObjects method
* attachment to the commit: fab50d487d86aa4011fb05e41e694b837eca92df
  For more information see the specified commit.
* planning_interface: Set is_diff true for empty start state
  Executing a motion without setting the start state of the robot like
  this:
  ```
  group_arm.setNamedTarget("start_grab_pose");
  success = group_arm.move();
  ```
  throws the error: Execution of motions should always start at the robot's
  current state. Ignoring the state supplied as start state in the motion
  planning request.
  The problem is, when considered_start_state\_ is null, every data field of the start_state
  in the submitted MotionPlanRequest is 0 or false. But we need is_diff to be
  true, because otherwise move_group will not consider its current state as
  actual start state without complaining.
* Implement issue `#630 <https://github.com/ros-planning/moveit_ros/issues/630>`_
* Contributors: Yannick Jonetzko, corot

0.7.1 (2016-04-11)
------------------
* [feat] Adding acceleration scaling factor
* [fix] conflict issues
* [doc] [move_group.cpp] Print the name of the move group action server that failed to be connected (`#640 <https://github.com/ros-planning/moveit_ros/issues/640>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito, hemes

0.7.0 (2016-01-30)
------------------
* Removed trailing whitespace from entire repository
* new method MoveGroup::getDefaultPlannerId(const std::string &group)
  ... to retrieve default planner config from param server
  moved corresponding code from rviz plugin to MoveGroup interface
  to facilitate re-use
* adding set_num_planning_attempts to python interface
* Added python wrapper for setMaxVelocityScalingFactor
* saves robot name to db from moveit. also robot name accessible through robot interface python wrapper
* adding set_num_planning_attempts to python interface
* Added python wrapper for MoveGroup.asyncExecute()
* Add retime_trajectory to moveit python wrapper
* add getHandle to move_group_interface
* Updated documentation on move() to inform the user that an asynchronus spinner is required. Commonly new users don't do this and move() blocks permanently
* Contributors: Dave Coleman, Dave Hershberger, Kei Okada, Michael Stevens, Robert Haschke, Sachin Chitta, Scott, Yoan Mollard, dg, ferherranz

0.6.5 (2015-01-24)
------------------
* update maintainers
* Add time factor support for iterative_time_parametrization
* Contributors: Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* include correct boost::*_ptr class for boost 1.57.
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
