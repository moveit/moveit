^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.19 (2014-06-23)
-------------------
* Changed rviz plugin action server wait to non-simulated time
* Fix [-Wreorder] warning.
* Fix RobotState rviz plugin to not display when disabled
* Add check for planning scene monitor connection, with 5 sec delay
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.18 (2014-03-23)
-------------------
* add pkg-config as dep
* find PkgConfig before using pkg_check_modules
  PC specific functions mustn't be used before including PkgConfig
* Contributors: Ioan Sucan, v4hn

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* back out problematic ogre fixes
* robot_interaction: split InteractionHandler into its own file
* Switched from isStateColliding to isStateValid
* Changed per PR review
* Clean up debug output
* Added ability to set a random <collision free> start/goal position
* Merge branch 'hydro-devel' of https://github.com/ros-planning/moveit_ros into acorn_rviz_stereo
* rviz: prepare for Ogre1.10
* Contributors: Acorn Pooley, Dave Coleman

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* remove debug printfs
* planning_scene_display: use requestPlanningSceneState()
  Get current planning scene state when planning scene display is
  enabled and/or model is loaded.
* Fix Parse error at "BOOST_JOIN" error
  See: https://bugreports.qt-project.org/browse/QTBUG-22829
* Contributors: Acorn Pooley, Benjamin Chretien

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------
* Added back-link to tutorial and updated moveit website URL.
* Ported MoveIt RViz plugin tutorial to sphinx.
* Contributors: Dave Hershberger

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* correcting maintainer email
* Fixed an occasional crash bug in rviz plugin caused by gui calls in non-gui thread.
* Added planning feedback to gui, refactored states tab
* Stored states are auto loaded when warehouse database is connected

0.5.8 (2013-10-11)
------------------
* Added option to rviz plugin to show scene robot collision geometry

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* Fix crash when the destructor is called before onInitialize
* remove call for getting the combined joint limits of a group
* bugfixes
* porting to new RobotState API
* use new helper class from rviz for rendering meshes

0.5.4 (2013-08-14)
------------------

* Added manipulation tab, added plan id to manipulation request
* make headers and author definitions aligned the same way; white space fixes
* using action client for object recognition instead of topic
* move background_processing lib to core
* display collision pairs instead of simply colliding links

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* fix `#275 <https://github.com/ros-planning/moveit_ros/issues/275>`_
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* remove root_link_name property
* add status tab to Rviz plugin
