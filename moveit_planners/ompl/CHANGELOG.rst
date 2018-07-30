^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_planners_ompl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.12 (2018-05-29)
-------------------
* forward OMPL logging to rosconsole (`#916 <https://github.com/ros-planning/moveit/issues/916>`_)
* switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* Make trajectory interpolation in MoveIt consistent to OMPL (`#869 <https://github.com/ros-planning/moveit/issues/869>`_)
* Contributors: Bryce Willey, Ian McMahon, Mikael Arguedas, Robert Haschke, Xiaojian Ma, Zachary Kingston

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [maintenance][kinetic onward] Remove OutputHandlerROS from ompl_interface (`#609 <https://github.com/ros-planning/moveit/issues/609>`_)
* Contributors: Bence Magyar

0.9.9 (2017-08-06)
------------------
* [improve][moveit_planners_ompl] Optional forced use of JointModelStateSpaceFactory (`#541 <https://github.com/ros-planning/moveit/issues/541>`_)
* Contributors: henhenhen

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* Always update initial robot state to prevent dirty robot state error.
* Contributors: Henning Kayser

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* Contributors: Dave Coleman

0.9.4 (2017-02-06)
------------------
* [enhancement] ompl_interface: uniform & simplified handling of the default planner (`#371 <https://github.com/ros-planning/moveit/issues/371>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Michael Goerner

0.9.3 (2016-11-16)
------------------
* [capability] Exposed planners from latest ompl release. (`#338 <https://github.com/ros-planning/moveit/issues/338>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Ruben Burger

0.9.2 (2016-11-05)
------------------

0.7.0 (2016-01-30)
------------------
* Removed trailing whitespace from entire repository
* Fixed include directory order to make ros package shadowing work.
* fixing internal storing of config settings
* Make sure an overlayed OMPL is used instead of the ROS one.
* fix simplifySolutions(bool) setter
  The method simplifySolutions(bool) always set the simplify_solutions member to true and the input variable "flag" was ignored.
  The method is fixed by setting the simplify_solutions member to the value of the input variable "flag".
* changed location of getDefaultPlanner
* Contributors: Bastian Gaspers, Christian Dornhege, Dave Coleman, Dave Hershberger, Sachin Chitta

0.6.7 (2014-10-28)
------------------
* Changed OMPL SimpleSetup member variable to shared pointer, passed MotionPlanningRequest to child function
* Simplified number of solve() entry points in moveit_planners_ompl
* Fixed uninitialized ``ptc_`` pointer causing a crash.
* renamed newGoal to new_goal for keeping with formatting
* setting GroupStateValidityCallbackFn member for constraint_sampler member and implementing callbacks for state validity checking
* added functions to check validit of state, and also to act as callback for constraint sampler
* Added copy function from MoveIt! robot_state joint values to ompl state
* fix for demo constraints database linking error
* Namespaced less useful debug output to allow to be easily silenced using ros console
* Contributors: Dave Coleman, Dave Hershberger, Sachin Chitta, arjungm

0.6.6 (2014-07-06)
------------------
* indigo version of moveit planners
* fix compile error on Indigo
* Fix for getMeasure() virtual function OMPL change
* Move OMPL paths before catkin to avoid compilation against ROS OMPL package when specifying a different OMPL installation
* Fixed bug which limited the number of plans considered to the number of threads.
* Contributors: Alexander Stumpf, Chris Lewis, Dave Coleman, Ryan Luna, Sachin Chitta

0.5.5 (2014-03-22)
------------------
* update build system for ROS indigo
* Removed duplicate call to setPlanningScene(), added various comments
* Contributors: Dave Coleman, Ioan Sucan

0.5.4 (2014-02-06)
------------------
* fix segfault when multiple goals are passed to move_group

0.5.3 (2013-10-11)
------------------
* update to new API

0.5.2 (2013-09-23)
------------------
* porting to new robot state

0.5.1 (2013-08-13)
------------------
* make headers and author definitions aligned the same way; white space fixes
* namespace change for profiler

0.5.0 (2013-07-15)
------------------

0.4.2 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)
* port ompl plugin to new base class for planning_interface (using planning contexts)

0.4.1 (2013-07-04)
------------------
* use new location of RRTstar, add PRMstar
* Added new cost function that takes into account robot joint movements
* Added ability for parameter sweeping by allowing parameters to be changed in planning contexts
* Added ability to alter configs in a cache

0.4.0 (2013-05-27)
------------------
* propagating changes from moveit_core

0.3.11 (2013-05-02)
-------------------
* remove some debug output and add some fixes
* some fixes for planning with constraint approximations
* more refactoring; what used to work (including using constraint approximations) works fine. explicitly storing motions is not yet done
* refactor constraints storage stuff
* display random motions in a slightly more robust way
* remove follow constraints API
* combine ompl_interface and ompl_interface_ros
* don't print status
* remove option for ordering constraint approximations (and fix `#12 <https://github.com/ros-planning/moveit_planners/issues/12>`_)
* add test for jumping configs
* use project() instead of sample() for producing goals
* minor fixes and add demo database construction code
* switch to using the profiler in moveit and add one more debug tool

0.3.10 (2013-04-17)
-------------------
* Merge branch 'groovy-devel' of github.com:ros-planning/moveit_planners into groovy-devel
* remove incorrect dep
* add dynamic reconfigure options for `#2 <https://github.com/ros-planning/moveit_planners/issues/2>`_

0.3.9 (2013-04-16 13:39)
------------------------
* disable old style benchmarking

0.3.8 (2013-04-16 11:23)
------------------------
* fix `#8 <https://github.com/ros-planning/moveit_planners/issues/8>`_
* use namespace option in ompl plugin
* remove unused functions
* add buildtool depends
* Fixed state deserialization: now update var transform too
* collapse OMPL plugin to one package
* robustness fix
* Fixed github url name

0.3.7 (2013-03-09)
------------------
* Remove configure from PlanningScene
* add multi-collision to PlanningScene
* renaming kinematic_model to robot_model

0.3.6 (2013-02-02)
------------------
* complete renaming process
* rename KinematicState to RobotState, KinematicTrajectory to RobotTrajectory
* propagating fixes from moveit_core
* use new robot_trajectory lib

0.3.5 (2013-01-28)
------------------
* fix reporting of goal collisions
* add some verbose output for failing goals
* port to new DisplayTrajectory message
* propagate API changes from planning_interface
* minor fix
* use the project() method to improve constraint following algorithm
* change default build flags

0.3.4 (2012-12-20 23:59)
------------------------
* dynamic_reconfigure workaroung

0.3.3 (2012-12-20 21:51)
------------------------
* update dyn reconfig call

0.3.2 (2012-12-20 13:45)
------------------------
* fix call to obsolete function

0.3.1 (2012-12-19)
------------------
* using the constraint sampler loading library
* make sure sampled goals are valid
* fix buildtool tag

0.3.0 (2012-12-10)
------------------
* add a debug msg
* re-enable heuristic
* first working version of follow planner
* most of the follow alg, but not 100% complete yet
* pass valid state samplers into the follow algorithm
* add constrained valid state sampler
* minor fixes
* fixes some catkin CMakeLists issues
* add code to allow execution of follow()
* port test to groovy
* placeholder for to-be-added algorithm
* minor touch-ups; no real functional changes other than a bias for state samplers wrt dimension of the space (when sampling in a ball of dimension D, focus the sampling towards the surface of the ball)
* minor & incomplete fix

0.2.5 (2012-11-26)
------------------
* update to new message API

0.2.4 (2012-11-23)
------------------
* improve error message
* stricter error checking
* update include path

0.2.3 (2012-11-21 22:47)
------------------------
* use generalized version of getMaximumExtent()

0.2.2 (2012-11-21 22:41)
------------------------
* more fixes to planners
* removed bad include dir
* fixed some plugin issues
* fixed include dirs in ompl ros interface
* added gitignore for ompl/ros

0.2.1 (2012-11-06)
------------------
* update install location of include/

0.2.0 (2012-11-05)
------------------
* udpate install targets

0.1.2 (2012-11-01)
------------------
* bump version
* install the plugin lib as well
* add TRRT to the list of options

0.1.1 (2012-10-29)
------------------
* fixes for build against groovy

0.1.0 (2012-10-28)
------------------
* port to groovy
* added some groovy build system files
* more moving around of packages
