^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_experimental
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

<<<<<<< HEAD
0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
=======
0.7.13 (2017-12-25)
>>>>>>> upstream/indigo-devel
-------------------
* [fix] remove explicit fcl depends `#632 <https://github.com/ros-planning/moveit/pull/632>`_
* Contributors: v4hn

<<<<<<< HEAD
0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_ 
* Contributors: Dave Coleman

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

0.9.0 (2016-10-19)
------------------
* Replace broken Eigen3 with correctly spelled EIGEN3 (`#254 <https://github.com/ros-planning/moveit/issues/254>`_)
  * Fix Eigen3 dependency throughout packages
  * Eigen 3.2 does not provide EIGEN3_INCLUDE_DIRS, only EIGEN3_INCLUDE_DIR
* fix exported plugin xml for collision_distance_field (`#280 <https://github.com/ros-planning/moveit/issues/280>`_)
  Otherwise the xml can not be found on an installed workspace
* Cleanup readme (`#258 <https://github.com/ros-planning/moveit/issues/258>`_)
* Convert collision_distance_field to std::shared_ptr.
* Use shared_ptr typedefs in collision_distance_field and chomp.
* Use srdf::ModelPtr typedefs.
* Switch to std::make_shared.
* [moveit_experimental] Fix incorrect dependency on FCL in kinetic
  [moveit_experimental] Fix Eigen3 warning
* Remove deprecated package shape_tools
  Fix OcTree boost::shared_ptr
  Remove deprecated CMake dependency
  Fix distanceRobot() API with verbose flags
* Fix CHOMP planner and CollisionDistanceField (`#155 <https://github.com/ros-planning/moveit/issues/155>`_)
  * Copy collision_distance_field package
  * Resurrect chomp
  * remove some old Makefiles and manifests
  * Correct various errors
  * Code formatting, author, description, version, etc
  * Add definitions for c++11. Nested templates problem.
  * Add name to planner plugin.
  * Change getJointModels to getActiveJointModels.
  * Call robot_state::RobotState::update in setRobotStateFromPoint.
  * Create README.md
  * Improve package.xml, CMake config and other changes suggested by jrgnicho.
  * Remove some commented code, add scaling factors to computeTimeStampes
  * Add install targets in moveit_experimental and chomp
  * Add install target for headers in chomp pkgs.
  * Remove unnecessary debugging ROS_INFO.
  * Port collision_distance_field test to indigo.
  * Remove one assertion that makes collision_distance_field test to fail.
* Use urdf::*SharedPtr instead of boost::shared_ptr
  urdfdom_headers uses C++ std::shared_ptr. As it exports it as custom
  *SharedPtr type, we can use them to stay compatible.
  Note that there is no std:shared_ptr<const urdf::ModelInterface>
  typedef, so I replaced it with urdf::ModelInterfaceSharedPtr (loosing a
  const).
  Also, there is no conversion from boost::shared_ptr<urdf::Model> to
  std:shared_ptr<urdf::ModelInterface>, so I used a preprocessor
  directive.
* fetch moveit_resources path at compile time
  using variable MOVEIT_TEST_RESOURCES_DIR provided by config.h
  instead of calling ros::package::getPath()
* adapted paths to moveit_resources
  (renamed folder moveit_resources/test to moveit_resources/pr2_description)
* Contributors: Chittaranjan Srinivas Swaminathan, Dave Coleman, Jochen Sprickerhof, Maarten de Vries, Michael Görner, Robert Haschke

0.8.3 (2016-08-21)
------------------
* this was implemented in a different way
* add kinematics constraint aware
* add kinematics_cache
* Update README.md
* Update README.md
* Create README.md
* copy collision_distance_field from moveit_core
* rename some headers
* add collision_distance_field_ros
* add kinematics_planner_ros
* added kinematics_cache_ros from moveit-ros
* moved from moveit_core
* Contributors: Ioan Sucan, isucan
=======
0.7.12 (2017-08-06)
-------------------

0.7.11 (2017-06-21)
-------------------

0.7.10 (2017-06-07)
-------------------

0.7.9 (2017-04-03)
------------------

0.7.8 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* Contributors: Dmitry Rozhkov

0.7.7 (2017-02-06)
------------------
* clang-format upgraded to 3.8 (`#404 <https://github.com/ros-planning/moveit/issues/404>`_)
* Contributors: Dave Coleman

0.7.6 (2016-12-30)
------------------

0.7.5 (2016-12-25)
------------------

0.7.4 (2016-12-22)
------------------
* [indigo][changelog] Remove wrong version entries (see https://github.com/ros-planning/moveit/issues/386#issuecomment-268689110).
* Contributors: Isaac I.Y. Saito

0.7.3 (2016-12-20)
------------------
* [ROS Indigo] Initial release from `ros-planning/moveit <https://github.com/ros-planning/moveit>`_ repository.
* [fix] exported plugin xml for collision_distance_field (`#280 <https://github.com/ros-planning/moveit/issues/280>`_)
* [fix] CHOMP planner and CollisionDistanceField (`#155 <https://github.com/ros-planning/moveit/issues/155>`_)
* [maintenance] add full VERSIONs / SONAMEs to all libraries (`#273 <https://github.com/ros-planning/moveit/issues/273>`_)
* [maintenance] Auto code formatted Indigo branch using clang-format (`#313 <https://github.com/ros-planning/moveit/issues/313>`_)
* Contributors: Chittaranjan Srinivas Swaminathan, Dave Coleman, Michael Goerner, Robert Haschke

>>>>>>> upstream/indigo-devel
