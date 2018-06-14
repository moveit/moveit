^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package chomp_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.12 (2018-05-29)
-------------------
* [fix] dependencies for chomp interface test (`#778 <https://github.com/ros-planning/moveit/issues/778>`_)
* Contributors: Bence Magyar, Dave Coleman, Ian McMahon, Mikael Arguedas, Robert Haschke, Stephan, Will Baker

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [package.xml] Update maintainers (Add a release-maintainer etc.)
  For the reasoning, see https://github.com/ros-planning/moveit/issues/259
* Contributors: Isaac I.Y. Saito

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
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

0.9.3 (2016-11-16)
------------------
* Merge pull request `#330 <https://github.com/ros-planning/moveit/issues/330>`_ from davetcoleman/kinetic-package.xml
  Updated package.xml maintainers and author emails
* Updated package.xml maintainers and author emails
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------

0.9.0 (2016-10-19)
------------------
* Replace broken Eigen3 with correctly spelled EIGEN3 (`#254 <https://github.com/ros-planning/moveit/issues/254>`_)
  * Fix Eigen3 dependency throughout packages
  * Eigen 3.2 does not provide EIGEN3_INCLUDE_DIRS, only EIGEN3_INCLUDE_DIR
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
* Contributors: Chittaranjan Srinivas Swaminathan, Dave Coleman, Maarten de Vries

0.8.3 (2016-08-21)
------------------
