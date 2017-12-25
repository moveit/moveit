^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] don't crash on empty robot_description in RobotState plugin `#688 <https://github.com/ros-planning/moveit/issues/688>`_
* [fix] RobotState rviz previewer: First message from e.g. latching publishers is not applied to robot state correctly (`#596 <https://github.com/ros-planning/moveit/issues/596>`_)
* [doc] Document auto scale in Rviz plugin (`#602 <https://github.com/ros-planning/moveit/issues/602>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito, Simon Schmeisser, axelschroth

0.9.9 (2017-08-06)
------------------
* [fix] RobotStateVisualization: clear before load to avoid segfault `#572 <https://github.com/ros-planning/moveit/pull/572>`_
* Contributors: v4hn

0.9.8 (2017-06-21)
------------------
* [fix] TrajectoryVisualization crash if no window_context exists (`#523 <https://github.com/ros-planning/moveit/issues/523>`_, `#525 <https://github.com/ros-planning/moveit/issues/525>`_)
* [fix] robot display: Don't reload robot model upon topic change (Fixes `#528 <https://github.com/ros-planning/moveit/issues/528>`_)
* [build] add Qt-moc guards for boost 1.64 compatibility (`#534 <https://github.com/ros-planning/moveit/issues/534>`_)
* [enhance] rviz display: stop trajectory visualization on new plan. Fixes `#526 <https://github.com/ros-planning/moveit/issues/526>`_ (`#531 <https://github.com/ros-planning/moveit/issues/531>`_, `#510 <https://github.com/ros-planning/moveit/issues/510>`_).
* Contributors: Isaac I.Y. Saito, Simon Schmeisser, Yannick Jonetzko, henhenhen, v4hn


0.9.7 (2017-06-05)
------------------
* [capability] New panel with a slider to control the visualized trajectory (`#491 <https://github.com/ros-planning/moveit/issues/491>`_) (`#508 <https://github.com/ros-planning/moveit/issues/508>`_)
* [fix] Build for Ubuntu YZ by adding BOOST_MATH_DISABLE_FLOAT128 (`#505 <https://github.com/ros-planning/moveit/issues/505>`_)
* Contributors: Dave Coleman, Mikael Arguedas

0.9.6 (2017-04-12)
------------------
* [fix] RViz plugin some cosmetics and minor refactoring `#482 <https://github.com/ros-planning/moveit/issues/482>`_
* [fix] rviz panel: Don't add object marker if the wrong tab is selected `#454 <https://github.com/ros-planning/moveit/pull/454>`_
* [improve] RobotState display [kinetic] (`#465 <https://github.com/ros-planning/moveit/issues/465>`_)
* Contributors: Jorge Nicho, Michael Goerner, Yannick Jonetzko

0.9.5 (2017-03-08)
------------------
* [fix] correct "simplify widget handling" `#452 <https://github.com/ros-planning/moveit/pull/452>`_ This reverts "simplify widget handling (`#442 <https://github.com/ros-planning/moveit/issues/442>`_)" 
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_ 
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman, Isaac I.Y. Saito, Yannick Jonetzko

0.9.4 (2017-02-06)
------------------
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [enhancement] Add colours to trajectory_visualisation display (`#362 <https://github.com/ros-planning/moveit/issues/362>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Bence Magyar, Dave Coleman, Robert Haschke

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
* cleanup cmake tests, fix empty output
* added missing rostest dependency (`#680 <https://github.com/ros-planning/moveit_ros/issues/680>`_), fixes c6d0ede (`#639 <https://github.com/ros-planning/moveit_ros/issues/639>`_)
* [moveit joy] Add friendlier error message
* relax Qt-version requirement
  Minor Qt version updates are ABI-compatible with each other:
  https://wiki.qt.io/Qt-Version-Compatibility
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* always (re)create collision object marker
  other properties than pose (such as name of the marker) need to be adapted too
* use getModelFrame() as reference frame for markers
* moved "Publish Scene" button to "Scene Objects" tab
  previous location on "Context" tab was weird
* cherry-pick PR `#635 <https://github.com/ros-planning/moveit_ros/issues/635>`_ from indigo-devel
* unify Qt4 / Qt5 usage across cmake files
  - fetch Qt version from rviz
  - define variables/macros commonly used for Qt4 and Qt5
  - QT_LIBRARIES
  - qt_wrap_ui()
* leave frame transforms to rviz
  The old code
  (1.) reimplemented frame transforms in rviz
  although it could simply utilize rviz' FrameManager
  (2.) assumed the transform between the model-frame
  and the fixed_frame was constant and only needed to be updated
  if the frame changes (ever tried to make the endeffector
  your fixed frame?)
  (3.) was broken because on startup calculateOffsetPosition was called
  *before* the robot model is loaded, so the first (and usually only)
  call to calculateOffsetPosition failed.
  Disabling/Enabling the display could be used to work around this...
  This fixes all three issues.
* display planned path in correct rviz context
  This was likely a typo.
* Solved parse error with Boost 1.58. Fixes `#653 <https://github.com/ros-planning/moveit_ros/issues/653>`_
* Enable optional build against Qt5, use -DUseQt5=On to enable it
* explicitly link rviz' default_plugin library
  The library is not exported anymore and now is provided separately from rviz_LIBRARIES.
  See https://github.com/ros-visualization/rviz/pull/979 for details.
* merge indigo-devel changes (PR `#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* correctly handle int and float parameters
  Try to parse parameter as int and float (in that series)
  and use IntProperty or FloatProperty on success to have
  input checking.
  Floats formatted without decimal dot, e.g. "0", will be
  considered as int!
  All other parameters will be handled as string.
* access planner params in rviz' MotionPlanningFrame
* new method MoveGroup::getDefaultPlannerId(const std::string &group)
  ... to retrieve default planner config from param server
  moved corresponding code from rviz plugin to MoveGroup interface
  to facilitate re-use
* correctly initialize scene robot's parameters after initialization
  - loaded parameters were ignored
  - changed default alpha value to 1 to maintain previous behaviour
* load default_planner_config from default location
  instead of loading from `/<ns>/default_planner_config`, use
  `/<ns>/move_group/<group>/default_planner_config`, which is the default
  location for `planner_configs` too
* Merge pull request `#610 <https://github.com/ros-planning/moveit_ros/issues/610>`_: correctly update all markers after robot motion
* fixing conflicts, renaming variable
* Merge pull request `#612 <https://github.com/ros-planning/moveit_ros/issues/612>`_ from ubi-agni/interrupt-traj-vis
  interrupt trajectory visualization on arrival of new display trajectory
* cherry-picked PR `#611 <https://github.com/ros-planning/moveit_ros/issues/611>`_
  fix segfault when disabling and re-enabling TrajectoryVisualization
* cherry-picked PR `#609 <https://github.com/ros-planning/moveit_ros/issues/609>`_
  load / save rviz' workspace config
  fixed tab order of rviz plugin widgets
  use move_group/default_workspace_bounds as a fallback for workspace bounds
* fixup! cleanup TrajectoryVisualization::update
  only enter visualization loop when displaying_trajectory_message_ is defined
* added missing initialization
* correctly setAlpha for new trail
* fixed race condition for trajectory-display interruption
  - TrajectoryVisualization::update() switches to new trajectory
  automatically when it has finished displaying the old one
  - TrajectoryVisualization::interruptCurrentDisplay() might interrupt
  this newly started trajectory
  consequences:
  - protect switching of trajectory with mutex
  - interrupt only if trajectory display progressed past first waypoint
  - removed obsolete signal timeToShowNewTrail:
  update() automatically switches and updates trail in sync
* cleanup TrajectoryVisualization::update
  simplified code to switch to new trajectory / start over animation in loop mode
* new GUI property to allow immediate interruption of displayed trajectory
* immediately show trajectory after planning (interrupting current display)
* fix segfault when disabling and re-enabling TrajectoryVisualization
  animating_path_ was still true causing update() to access
  displaying_trajectory_message_, which was reset onDisable().
* update pose of all markers when any marker moved
  Having several end-effector markers attached to a group (e.g. a multi-
  fingered hand having an end-effector per fingertip and an end-effector
  for the hand base), all markers need to update their pose on any motion
  of any marker. In the example: if the hand base is moved, the fingertip
  markers should be moved too.
* use move_group/default_workspace_bounds as a fallback for workspace bounds
* code style cleanup
* fixed tab order of rviz plugin widgets
* load / save rviz' workspace config
* saves robot name to db from moveit. also robot name accessible through robot interface python wrapper
* Added install rule to install moveit_joy.py.
* motion_planning_frame_planning: use /default_planner_config parma to specify default planning algorithm
* Avoid adding a slash if getMoveGroupNS() is empty.
  If the getMoveGroupNS() returns an empty string, ros::names::append() inserts a slash in front of 'right', which changes it to a global name.
  Checking getMoveGroupNS() before calling append removes the issue.
  append() behaviour will not be changed in ros/ros_comm.
* Contributors: Ammar Najjar, Dave Coleman, Isaac I.Y. Saito, Jochen Welle, Kei Okada, Michael Ferguson, Michael Görner, Robert Haschke, Sachin Chitta, Simon Schmeisser (isys vision), TheDash, Thomas Burghout, dg, v4hn

0.6.5 (2015-01-24)
------------------
* update maintainers
* Created new trajectory display, split from motion planning display
* Added new trajectory display inside of motion planning display
* Fix bug with alpha property in trajectory robot
* Optimized number of URDFs loaded
* Changed motion planning Rviz icon to MoveIt icon
* Add time factor support for iterative_time_parametrization
* Contributors: Dave Coleman, Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* fix duplicate planning attempt box, also fix warning about name
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------
* Fixed joystick documentation
* Joystick documentation and queue_size addition
* Contributors: Dave Coleman

0.6.0 (2014-10-27)
------------------
* Added move_group capability for clearing octomap.
* Fix coding style according to the moveit style
* Better user output, kinematic solver error handling, disclaimer
* Remove sample launch file for joystick and update
  joystick python script.
  1) Use moveit-python binding to parse SRDF.
  2) Make the speed slower to control the marker from joystick.
  3) Change joystick button mapping to be suitable for the users.
* Update joystick documentation and rename the
  the launch file for joy stick program.
  Shorten the message the check box to toggle
  communication with joy stick script.
* add checkbox to toggle if moveit rviz plugin subscribes
  the topics to be used for communication to the external ros nodes.
  update moveit_joy.py to parse srdf to know planning_groups and the
  names of the end effectors and support multi-endeffector planning groups.
* motion_planning_rviz_plugin: add move_group namespace option
  This allows multiple motion_planning_rviz_plugin /
  planning_scene_rviz_plugin to be used in RViz and connect to
  differently-namespaced move_group nodes.
* moved planning_attempts down one row in gui to maintain gui width
* Added field next to planning_time for planning_attempts
  Now, ParallelPlanner terminates either due to timeout, or due to this many attempts.
  Note, that ParallelPlanner run's Dijkstra's on all the nodes of all the sucessful plans (hybridize==true).
* adding PoseStamped topic to move the interactive marker from other ros nodes
  such as joystick programs.
* motion_planning_rviz_plugin: add move_group namespace option
  This allows multiple motion_planning_rviz_plugin /
  planning_scene_rviz_plugin to be used in RViz and connect to
  differently-namespaced move_group nodes.
* Contributors: Chris Lewis, Dave Coleman, Dave Hershberger, Jonathan Bohren, Ryohei Ueda, Sachin Chitta

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
