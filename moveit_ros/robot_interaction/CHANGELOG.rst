^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_robot_interaction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.19 (2014-06-23)
-------------------
* Fix [-Wreorder] warning.
* Allow planning groups to have more than one tip
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* fix test
  This was testing functionality that got removed.  Removed that part of the
  test.
* robot_interaction: add comments
  Comment cryptic public function behavior.
* robot_interaction: fix formatting
  remove tabs and whitespace at the end of lines.
* robot_interaction: fix comment formatting
  Limit lines to 120 chars max (80 preferred in headers).
* robot_interaction: fix setStateFromIK prototypes
  use references instead of pointers.
* robot_interaction: fix header problems
  fix getRobotModel() bug
  make internal functions private.
* remove extraneous code
* add missing headers
* robot_interaction: Fix issues raised by Ioan
* robot_interaction: use LockedRobotState
  Fix a number of thread safety violations.
* robot_interaction: add LockedRobotState and tests
* robot_interaction: use KinematicOptionsMap
  Fixes threading issues.
  Separate the handling of kinematics options into a separate object which
  enforces thread safe access.
* robot_interaction: add KinematicOptions
  KinematicOptions contains the parameters needed to call RobotState::setFromIK.
  KinematicOptionsMap contains a map of string->KinematicOptions a default KinematicOptions.
  These are useful in RobotInteraction with the group name as the key.
* pull RobotInteraction structures out of class
  The Generic, EndEffector, and Joint structures complicate the core of
  RobotInteraction.  Pull them out to simplify the code.  This will also
  help with future plans to make the core of RobotInteraction more
  generic and flexible.
* fix include guards to match moveit conventions
* robot_interaction: include interaction_handler.h from robot_interaction.h
  This is for backwards compatibility with code that only includes
  robot_interaction.h
* robot_interaction: split handler into own file
* robot_interaction: split InteractionHandler into its own file
* robot_interaction: make lock-protected members private
  Since the lock is needed to access these and the lock is private it makes no
  sense for them to be protected.
* robot_interaction: add locking comments
* robot_interaction: simplify code
* robot_interaction: fix comments
* Contributors: Acorn Pooley

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------

0.5.12 (2014-01-03)
-------------------
* Fixed trailing underscores in CHANGELOGs.
* Contributors: Dave Hershberger

0.5.11 (2014-01-03)
-------------------

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* adds KDL link directories to robot_interaction/CMakeLists.txt (fixes `#376 <https://github.com/ros-planning/moveit_ros/issues/376>`_)
* fixed computation of dimension\_.
* fixes for mimic joints and redundant joints

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes
* fix `#283 <https://github.com/ros-planning/moveit_ros/issues/283>`_

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* fix `#275 <https://github.com/ros-planning/moveit_ros/issues/275>`_
* white space fixes (tabs are now spaces)
* adding options struct to kinematics base

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* bugfixes
* robot_interaction: include sphere markers by default
* use improved MOVE_ROTATE_3D marker
