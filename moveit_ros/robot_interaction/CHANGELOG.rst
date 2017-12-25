^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_robot_interaction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [package.xml] Add a release-maintainer. Cleanup `#649 <https://github.com/ros-planning/moveit/pull/649>`_

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [fix] `catkin_make -DCMAKE_ENABLE_TESTING=0` failure (`#478 <https://github.com/ros-planning/moveit/issues/478>`_)
* Contributors: Michael Goerner

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_ 
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.6.6 (2016-06-08)
------------------
* use getModelFrame() as reference frame for markers
* publish markers relative to robot's root frame
  In addition to `#669 <https://github.com/ros-planning/moveit_ros/issues/669>`_, interactive markers need to be place relative to the
  robot's root frame. If nothing is specified (as before), rviz' fixed frame
  is used, leading to offsets when both frames are not identical.
* merge indigo-devel changes (PR `#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* further adapted marker size computation
  - drop largest extension dimension (-> use cross-section size of elongated link)
  - for an end-effector group, consider the sizes of individual links
  instead of the overall size of all links (which becomes huge very fast)
  - enlarge marker size by factor of 1.5 when there is only a single eef marker
* reworked computeLinkMarkerSize()
  compute size such that the marker sphere will cover
  - a spherical link geometry -> AABB.maxCoeff
  - a cubical link geometry -> AABB.norm
  -> use average of both values
  Virtual links (without any shape) will have a size of AABB of zero dims.
  In this case use the dimensions of the closest parent link instead.
* improved computation of interactive marker size
  - use parent_link if group == parent_group
  - scale smaller than 5cm is clipped to 5cm instead of using default
  - clarified size computation, using diameter of AABB
* fixing error caused by BOOST_STATIC_ASSERT
* Fixed compile error caused by BOOST_STATIC_ASSERT in kinematic_options.cpp
  Added kinematics::DiscretizationMethods::DiscretizationMethod to QO_FIELDS in kinematic_options.cpp.
  At pull request `#581 <https://github.com/ros-planning/moveit_ros/issues/581>`_, type of discretization_method was set to int. Changed it to proper type.
* reinstated changes related to the updates in the  moveit_core::KinematicsBase interface
* Revert "  Kinematics Base changes in moveit_core"
* adds the 'returns_approximate_solution' entry so that it is compatible with the changes in kinematics::KinematicsBase class in the moveit_core repo
* Contributors: Daichi Yoshikawa, Dave Coleman, Robert Haschke, Sachin Chitta, jrgnicho

0.6.5 (2015-01-24)
------------------
* update maintainers
* Contributors: Michael Ferguson

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Fix coding style according to the moveit style
* update joystick documentation according to the latest implementation
* add checkbox to toggle if moveit rviz plugin subscribes
  the topics to be used for communication to the external ros nodes.
  update moveit_joy.py to parse srdf to know planning_groups and the
  names of the end effectors and support multi-endeffector planning groups.
* adding PoseStamped topic to move the interactive marker from other ros nodes
  such as joystick programs.
* Contributors: Ryohei Ueda, Sachin Chitta

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
