^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package chomp_motion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

<<<<<<< HEAD
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
=======
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

0.7.8 (2017-03-08)
------------------

0.7.7 (2017-02-06)
------------------

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
* [fix] CHOMP planner and CollisionDistanceField (`#155 <https://github.com/ros-planning/moveit/issues/155>`_)
* [maintenance] add full VERSIONs / SONAMEs to all libraries (`#273 <https://github.com/ros-planning/moveit/issues/273>`_)
* [maintenance] Auto code formatted Indigo branch using clang-format (`#313 <https://github.com/ros-planning/moveit/issues/313>`_)
* Contributors: Chittaranjan Srinivas Swaminathan, Dave Coleman, Michael Goerner
>>>>>>> upstream/indigo-devel
