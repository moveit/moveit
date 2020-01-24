^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package chomp_motion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.18 (2020-01-24)
-------------------

0.9.17 (2019-07-09)
-------------------

0.9.16 (2019-06-29)
-------------------
* [maintanance] Cleanup Chomp packages (`#1282 <https://github.com/ros-planning/moveit/issues/1282>`_, `#1221 <https://github.com/ros-planning/moveit/issues/1221>`_)
* [maintanance] Fix clang issues (`#1233 <https://github.com/ros-planning/moveit/issues/1233>`_, `#1214 <https://github.com/ros-planning/moveit/issues/1214>`_)
* [fix]         Set last_state only for active joints in chomp_planner (`#1222 <https://github.com/ros-planning/moveit/issues/1222>`_)
* Contributors: Michael Görner, Robert Haschke, Shingo Kitagawa

0.9.15 (2018-10-29)
-------------------

0.9.14 (2018-10-24)
-------------------

0.9.13 (2018-10-24)
-------------------
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [fix] changelogs: migration from tf -> tf2 only accidentally became part of 0.9.12's changelog
* [capability] Addition of CHOMP planning adapter for optimizing result of other planners (`#1012 <https://github.com/ros-planning/moveit/issues/1012>`_)
* [capability] Failure recovery options for CHOMP by tweaking parameters (`#987 <https://github.com/ros-planning/moveit/issues/987>`_)
* [capability] cleanup of unused parameters and code + addition of trajectory initialization methods (linear, cubic, quintic-spline) (`#960 <https://github.com/ros-planning/moveit/issues/960>`_)
* Contributors: Adrian Zwiener, Raghavender Sahdev, Robert Haschke

0.9.12 (2018-05-29)
-------------------
* [fix] for chomp fixed base joint bug (`#870 <https://github.com/ros-planning/moveit/issues/870>`_)
* [maintenance] switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* Contributors: Bence Magyar, Dave Coleman, Ian McMahon, Mike Lautman, Xiaojian Ma

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------

0.9.9 (2017-08-06)
------------------
* [improve] Chomp use PlanningScene (`#546 <https://github.com/ros-planning/moveit/issues/546>`_) to partially address `#305 <https://github.com/ros-planning/moveit/issues/305>`_
* Contributors: Simon Schmeisser

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------

0.9.5 (2017-03-08)
------------------

0.9.4 (2017-02-06)
------------------

0.9.3 (2016-11-16)
------------------

0.9.2 (2016-11-05)
------------------
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito

0.9.0 (2016-10-19)
------------------
* Use shared_ptr typedefs in collision_distance_field and chomp.
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
* Contributors: Chittaranjan Srinivas Swaminathan, Maarten de Vries

0.8.3 (2016-08-21)
------------------
