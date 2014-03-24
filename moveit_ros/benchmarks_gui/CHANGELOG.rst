^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_benchmarks_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.18 (2014-03-23)
-------------------
* add pkg-config as dep
* find PkgConfig before using pkg_check_modules
  PC specific functions mustn't be used before including PkgConfig
* Contributors: Ioan Sucan, v4hn

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* back out problematic ogre fixes
* robot_interaction: split InteractionHandler into its own file
* rviz: prepare for Ogre1.10
* Contributors: Acorn Pooley

0.5.14 (2014-02-06)
-------------------
* fixing name in changelog
* Contributors: Sachin Chitta

0.5.13 (2014-02-06)
-------------------
* Fix Parse error at "BOOST_JOIN" error
  See: https://bugreports.qt-project.org/browse/QTBUG-22829
* Contributors: Benjamin Chre'tien

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* renamed collision check function to be clearer
* fixed bug in collision detection for goal pose queries
* fixed dirty transform bugs
* Fixed one crash bug and fixed goal-selection bug `#339 <https://github.com/ros-planning/moveit_ros/issues/339>`_.
* Benchmarking GUI: removed code redundancy, added link_name and frame_id values to saved goal constraints as needed for benchmarking
* Changed Warehouse Connect button to match wording and color of Rviz Motion Planning Plugin equivalent

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* porting to new robot API

0.5.4 (2013-08-14)
------------------

* allow creating goals uniformly distributed in a bounding box
* Duplicate goals with Ctrl-D instead of Ctrl-C
* better error checking
* adding the possibility to select planners, nb of runs and timeout from the GUI
* allow the user to modify start and goal regex before starting benchmark
* Allow user to start the benchmark pipeline from the GUI
* make headers and author definitions aligned the same way; white space fixes
* move background_processing lib to core

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------
* More advanced parameter sweeping implmented, workspace bounds added
* Added parameter sweeping to benchmarking

0.4.4 (2013-06-26)
------------------
* waits until the planning scene monitor has been created when loading a robot
