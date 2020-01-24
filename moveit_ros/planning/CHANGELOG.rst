^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.18 (2020-01-24)
-------------------

0.9.17 (2019-07-09)
-------------------

0.9.16 (2019-06-29)
-------------------
* [maintanance] Cleanup Chomp packages (`#1282 <https://github.com/ros-planning/moveit/issues/1282>`_)
* [maintanance] Disable (unused) dependencies (`#1256 <https://github.com/ros-planning/moveit/issues/1256>`_)
* [maintanance] Resolve catkin lint issues (`#1137 <https://github.com/ros-planning/moveit/issues/1137>`_)
* [maintanance] Improve clang-format (`#1214 <https://github.com/ros-planning/moveit/issues/1214>`_)
* [maintanance] Change dynamic reconfigure limits for allowed_goal_duration_margin to 30s (`#993 <https://github.com/ros-planning/moveit/issues/993>`_)
* [fix]         Use correct trajectory_initialization_method param (`#1237 <https://github.com/ros-planning/moveit/issues/1237>`_)
* Contributors: Hamal Marino, Ludovic Delval, Michael Görner, Robert Haschke, Stephan

0.9.15 (2018-10-29)
-------------------
* [fix] Build regression (`#1134 <https://github.com/ros-planning/moveit/issues/1134>`_) 
* [improvement] Exploit the fact that our transforms are isometries (instead of general affine transformations). `#1091 <https://github.com/ros-planning/moveit/issues/1091>`_
* Contributors: Robert Haschke

0.9.14 (2018-10-24)
-------------------

0.9.13 (2018-10-24)
-------------------
* [fix] Chomp package handling issue `#1086 <https://github.com/ros-planning/moveit/issues/1086>`_ that was introduced in `ubi-agni/hotfix-#1012 <https://github.com/ubi-agni/hotfix-/issues/1012>`_
* [fix] PlanningSceneMonitor lock `#1033 <https://github.com/ros-planning/moveit/issues/1033>`_: Fix `#868 <https://github.com/ros-planning/moveit/issues/868>`_ (`#1057 <https://github.com/ros-planning/moveit/issues/1057>`_)
* [fix] CurrentStateMonitor update callback for floating joints to handle non-identity joint origins `#984 <https://github.com/ros-planning/moveit/issues/984>`_
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix] reset moveit_msgs::RobotState.is_diff to false (`#968 <https://github.com/ros-planning/moveit/issues/968>`_) This fixes a regression introduced in `#939 <https://github.com/ros-planning/moveit/issues/939>`_.
* [capability][chomp] Addition of CHOMP planning adapter for optimizing result of other planners (`#1012 <https://github.com/ros-planning/moveit/issues/1012>`_)
* [capability] new dynamic-reconfigure parameter wait_for_trajectory_completion to disable waiting for convergence independently from start-state checking. (`#883 <https://github.com/ros-planning/moveit/issues/883>`_)
* [capability] Option for controller-specific duration parameters (`#785 <https://github.com/ros-planning/moveit/issues/785>`_)
* [enhancement] do not wait for robot convergence, when trajectory_execution_manager finishes with status != SUCCEEDED (`#1011 <https://github.com/ros-planning/moveit/issues/1011>`_)
* [enhancement] allow execution of empty trajectories (`#940 <https://github.com/ros-planning/moveit/issues/940>`_)
* [enhancement] avoid warning spam: "Unable to update multi-DOF joint" (`#935 <https://github.com/ros-planning/moveit/issues/935>`_)
* Contributors: 2scholz, Adrian Zwiener, Kei Okada, Michael Görner, Mohmmad Ayman, Raghavender Sahdev, Robert Haschke, Simon Schmeisser, Xaver Kroischke, mike lautman, srsidd

0.9.12 (2018-05-29)
-------------------
* CSM: copy dynamics to output if enabled (`#922 <https://github.com/ros-planning/moveit/issues/922>`_)
* move PlanExecution::executeAndMonitor() into public API (`#812 <https://github.com/ros-planning/moveit/issues/812>`_)
* [fix] explicitly enforce updateSceneWithCurrentState() in waitForCurrentRobotState() (`#824 <https://github.com/ros-planning/moveit/issues/824>`_)
* Support static TFs for multi-DOF joints in CurrentStateMonitor (`#799 <https://github.com/ros-planning/moveit/issues/799>`_)
* support xacro args (`#796 <https://github.com/ros-planning/moveit/issues/796>`_)
* CSM: wait for *active* joint states only (`#792 <https://github.com/ros-planning/moveit/issues/792>`_)
* skip non-actuated joints for execution (`#754 <https://github.com/ros-planning/moveit/issues/754>`_)
* Iterative cubic spline interpolation (`#441 <https://github.com/ros-planning/moveit/issues/441>`_)
* Floating Joint Support in CurrentStateMonitor (`#748 <https://github.com/ros-planning/moveit/issues/748>`_)
* validate multi-dof trajectories before execution (`#713 <https://github.com/ros-planning/moveit/issues/713>`_)
* Contributors: 2scholz, Bruno Brito, Dave Coleman, Ian McMahon, Ken Anderson, Michael Görner, Mikael Arguedas, Robert Haschke

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] Avoid segfault when validating a multidof-only trajectory (`#691 <https://github.com/ros-planning/moveit/issues/691>`_). Fixes `#539 <https://github.com/ros-planning/moveit/issues/539>`_
* [fix] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* [capability] Multi DOF Trajectory only providing translation not velocity (`#555 <https://github.com/ros-planning/moveit/issues/555>`_)
* Contributors: Isaac I.Y. Saito, Michael Görner, Mikael Arguedas, Troy Cordie

0.9.9 (2017-08-06)
------------------
* [fix] Change getCurrentExpectedTrajectory index so collision detection is still performed even if the path timing is not known (`#550 <https://github.com/ros-planning/moveit/issues/550>`_)
* [fix] Support for MultiDoF only trajectories `#553 <https://github.com/ros-planning/moveit/pull/553>`_
* [fix] ros_error macro name (`#544 <https://github.com/ros-planning/moveit/issues/544>`_)
* [fix] check plan size for plan length=0 `#535 <https://github.com/ros-planning/moveit/issues/535>`_
* Contributors: Cyrille Morin, Michael Görner, Mikael Arguedas, Notou, Unknown

0.9.8 (2017-06-21)
------------------
* [fix] Include callback of execution status if trajectory is invalid. (`#524 <https://github.com/ros-planning/moveit/issues/524>`_)
* Contributors: dougsm

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* [fix] undefined symbol in planning_scene_monitor (`#463 <https://github.com/ros-planning/moveit/issues/463>`_)
* Contributors: Dmitry Rozhkov, Ruben Burger

0.9.5 (2017-03-08)
------------------
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar

0.9.4 (2017-02-06)
------------------
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [maintenance] Use static_cast to cast to const. (`#433 <https://github.com/ros-planning/moveit/issues/433>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Maarten de Vries, Robert Haschke

0.9.3 (2016-11-16)
------------------
* [fix] cleanup urdfdom compatibility (`#319 <https://github.com/ros-planning/moveit/issues/319>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Robert Haschke

0.9.2 (2016-11-05)
------------------
* [Capability] compatibility to urdfdom < 0.4 (`#317 <https://github.com/ros-planning/moveit/issues/317>`_)
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Robert Haschke

0.6.6 (2016-06-08)
------------------
* Add library moveit_collision_plugin_loader as an exported catkin library (`#678 <https://github.com/ros-planning/moveit_ros/issues/678>`_)
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* Fix compilation with C++11.
* Enable optional build against Qt5, use -DUseQt5=On to enable it
* merge indigo-devel changes (PR `#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* Optional ability to copy velocity and effort to RobotState
* cherry-picked PR `#614 <https://github.com/ros-planning/moveit_ros/issues/614>`_
  fixed segfault on shutdown
* fixed segfault on shutdown
  use of pluginlib's createUnmanagedInstance() is strongly discouraged:
  http://wiki.ros.org/class_loader#Understanding_Loading_and_Unloading
  here, the kinematics plugin libs were unloaded before destruction of corresponding pointers
* Deprecate shape_tools
* CurrentStateMonitor no longer requires hearing mimic joint state values.
* Fix crash due to robot state not getting updated (moveit_ros `#559 <https://github.com/ros-planning/moveit_ros/issues/559>`_)
* Contributors: Dave Coleman, Dave Hershberger, Isaac I.Y. Saito, Levi Armstrong, Maarten de Vries, Robert Haschke, Simon Schmeisser (isys vision), kohlbrecher

0.6.5 (2015-01-24)
------------------
* update maintainers
* perception: adding RAII-based locking for OccMapTree
* perception: adding locks to planning scene monitor
* Add time factor support for iterative_time_parametrization
* Contributors: Jonathan Bohren, Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------
* Namespaced "traj_execution" for all trajectory_execution_manager logging
* planning_scene_monitor: add ros parameter for adding a wait-for-transform lookup time
  fixes `#465 <https://github.com/ros-planning/moveit_ros/issues/465>`_
* Contributors: Dave Coleman, Jonathan Bohren

0.6.3 (2014-12-03)
------------------
* add plugin interface for collision detectors
* fix missing return value
* trivial fixes for warnings
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------
* re-add libqt4 dependency (previously came from pcl-all)
* Contributors: Michael Ferguson

0.6.0 (2014-10-27)
------------------
* Removed leadings slash from rosparam for robot padding
* Added move_group capability for clearing octomap.
* Made loading octomap_monitor optional in planning_scene_monitor when using WorldGeometryMonitor
* Contributors: Dave Coleman, Dave Hershberger, Sachin Chitta, ahb

0.5.19 (2014-06-23)
-------------------
* Updated doxygen comment in TrajectoryExecutionManager.
* Added more informative error message text when cant' find controllers.
* robot_model_loader.cpp: added call to KinematicsBase::supportsGroup().
* Fix [-Wreorder] warning.
* Fix broken log & output statements.
  - Address [cppcheck: coutCerrMisusage] and [-Werror=format-extra-args] errors.
  - ROS_ERROR -> ROS_ERROR_NAMED.
  - Print size_t values portably.
* Address [-Wreturn-type] warning.
* Address [cppcheck: postfixOperator] warning.
* Address [cppcheck: duplicateIf] error.
  The same condition was being checked twice, and the same action was being taken.
* Add check for planning scene monitor connection, with 5 sec delay
* Fix for building srv_kinematics_plugin
* New ROS service call-based IK plugin
* Allow planning groups to have more than one tip
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Dave Hershberger

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Namespace a debug message
* Minor non-functional changes to KDL
* Contributors: Dave Coleman, Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* Copy paste error fix
* Contributors: fivef

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* remove debug printfs
* planning_scene_monitor: add requestPlanningSceneState()
* planning_scene_monitor: fix race condition
* planning_scene_monitor: fix state update bug
  The rate of state updates is limited to dt_state_update per second.
  When an update arrived it was not processed if another was recently
  processed.  This meant that if a quick sequence of state updates
  arrived and then no updates arrive for a while that the last update(s)
  were not seen until another arrives (which may be much later or
  never). This fixes the bug by periodically checking for pending
  updates and running them if they have been pending longer than
  dt_state_update.
* add default_robot_link_padding/scale, set padding/scale value for each robot link, see https://github.com/ros-planning/moveit_ros/issues/402
* fix LockedPlanningSceneRW docs
  fix the text that was originally copied from another class
  (from LockedPlanningSceneRO)
  it mentioned an incorrect return value type of
  LockedPlanningSceneRW::operator->()
* Contributors: Acorn Pooley, Filip Jares, Kei Okada

0.5.12 (2014-01-03)
-------------------
* Fixed trailing underscores in CHANGELOGs.
* Contributors: Dave Hershberger

0.5.11 (2014-01-03)
-------------------
* planning_scene_monitor: slight code simplification
* planning_scene_monitor: fix scope of local vars
* planning_scene_monitor: fix init bug
  scene_const\_ not set if scene passed to constructor.
* kdl_kinematics_plugin: fix warning
* Contributors: Acorn Pooley

0.5.10 (2013-12-08)
-------------------
* fixing how joint names are filled up, fixed joints were getting included earlier, also resizing consistency limits for when random positions near by function is being called
* Contributors: Sachin Chitta

0.5.9 (2013-12-03)
------------------
* Doxygen: added warnings and details to planning_scene_monitor.h
* correcting maintainer email
* remove duplicate header
* Fixed exported targets
* Fixed dependency issue
* fixing joint limits setup for mimic joints
* implement feature requests
* clear monitored octomap when needed (see `#315 <https://github.com/ros-planning/moveit_ros/issues/315>`_)
* fix the adapter for fixing path constraints for initial states
* fixed computation of dimension\_.
* bugfixes in indexing added states for path adapters
* fixes for mimic joints and redundant joints

0.5.8 (2013-10-11)
------------------
* add executable for publishing scene geometry from text
* Made the goal duration margin and scaling optional rosparameters
* bugfixes

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* fix the event triggered on updating attached objects
* make scene monitor trigger updates only when needed
* fix loading of default params
* port to new RobotState API, new messages
* make sure we do not overwrite attached bodies, when updating the current state
* fix `#308 <https://github.com/ros-planning/moveit_ros/issues/308>`_
* fix `#304 <https://github.com/ros-planning/moveit_ros/issues/304>`_
* fix issue with sending trajectories for passive/mimic/fixed joints
* pass effort along

0.5.4 (2013-08-14)
------------------

* remove CollisionMap, expose topic names in PlanningSceneMonitor, implement detach / attach operations as requested by `#280 <https://github.com/ros-planning/moveit_ros/issues/280>`_
* make headers and author definitions aligned the same way; white space fixes
* move background_processing lib to core
* add option to disable trajectory monitoring

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* minor doc fixes
* add docs for planning pipeline
* cleanup build system
* fixing approximate ik calculation
* white space fixes (tabs are now spaces)
* adding check for approximate solution flag
* adding options struct to kinematics base
* port to new base class for planning_interface (using planning contexts)

0.4.5 (2013-07-03)
------------------
* Namespaced ROS_* log messages for better debug fitering - added 'kdl' namespace
* remove dep
* make searchPositionIK actually const, and thread-safe
* Made debug output look better

0.4.4 (2013-06-26)
------------------
* fix `#210 <https://github.com/ros-planning/moveit_ros/issues/210>`_
* added dynamic reconfigure parameters to allow enabling/disabling of trajectory duration monitoring. fixes `#256 <https://github.com/ros-planning/moveit_ros/issues/256>`_
* add state operations evaluation tool
* warn when time parametrization fails
* moved exceptions headers
