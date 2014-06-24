^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_planning_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
