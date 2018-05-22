^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_move_group
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#833 <https://github.com/ros-planning/moveit/issues/833>`_ from davetcoleman-melodic-execute-service
* MoveIt! tf2 migration (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
  migration from tf to tf2 API, resolves issue `#745 <https://github.com/ros-planning/moveit/issues/745>`_
  - All type conversions now depend on geometry2 ROS packages, rather than geometry
  (see https://github.com/ros/geometry2/pull/292 and
  https://github.com/ros/geometry2/pull/294 for details of the new conversions)
  - Removes all boost::shared_ptr<tf::TransformListener> from the API,
  and replaced them with std::shared_ptr<tf2_ros::Buffer>'s
  - Utilize new tf2 API in the tf::Transformer library to access the internal tf2::Buffer of RViz
  (see https://github.com/ros/geometry/pull/163 for details of the new API)
  - Removes prepending of forward slashes ('/') for transforms frames as this is deprecated in tf2
  - Replaced deprecated tf2 _getLatestCommonTime
* Remove deprecated ExecuteTrajectoryServiceCapability in Melodic
  - Add MIGRATION.md file
* update include statements to use new pluginlib and class_loader headers (`#827 <https://github.com/ros-planning/moveit/issues/827>`_)
* Cleanup source of execute trajectory action capability (`#834 <https://github.com/ros-planning/moveit/issues/834>`_)
* Add namespace capabilities to moveit_commander (`#835 <https://github.com/ros-planning/moveit/issues/835>`_)
  * Add python goodies
  * Update planning interface tests for namespace arg
  * Add namespace tests
  * Update moveit commander for namespacing
  * Add moveit commander tests
  * Add movegroup test in namespace
  * Update param scopes in template and launch files
  * Code formatting
  * Add BSD License and name
  * Add description
  * Fix joy test
* disable flaky unittest (`#771 <https://github.com/ros-planning/moveit/issues/771>`_)
  ... introduced in `#756 <https://github.com/ros-planning/moveit/issues/756>`_
* MoveAction capability can drop cancel request if it is sent shortly after goal is sent (`#756 <https://github.com/ros-planning/moveit/issues/756>`_)
  * add unittests to check for dropped cancelling of move action
  * prevent cancelling of move actionstarts to be dropped
* Contributors: Dave Coleman, Ian McMahon, Mikael Arguedas, Robert Haschke, Will Baker, khupilz

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] always return true in MoveGroupPlanService callback `#674 <https://github.com/ros-planning/moveit/pull/674>`_
* [improve] adding swp's to gitignore and removing redundant capabilites from capability_names.h (`#704 <https://github.com/ros-planning/moveit/issues/704>`_)
* Contributors: Mike Lautman, Shingo Kitagawa

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
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Robert Haschke

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------

0.6.6 (2016-06-08)
------------------
* added missing validity check
  iterator found with `configs.find()` needs to be validated before use...
* Removed trailing whitespace from entire repository
* moved planner params services into existing capability QueryPlannerInterfaces
* capability plugin MoveGroupPlannerParamsService to get/set planner params
* Fixed bug(?) in move_group::MoveGroupKinematicsService::computeIK link name selection.
* Contributors: Dave Coleman, Mihai Pomarlan, Robert Haschke

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
* Merge pull request `#522 <https://github.com/ros-planning/moveit_ros/issues/522>`_ from mikeferguson/indigo-devel
  add dependency on std_srvs (part of octomap clearing service)
* Contributors: Ioan A Sucan, Michael Ferguson

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Added move_group capability for clearing octomap.
* Contributors: Dave Hershberger, Sachin Chitta

0.5.19 (2014-06-23)
-------------------
* Address [cppcheck: duplicateIf] error.
  The same condition was being checked twice, and the same action was being taken.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* empty state should be a diff state, otherwise attached objects are deleted
* Contributors: sachinc

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* Re-ordered movegroup's initialization, so capabilities start after monitors.
* correcting maintainer email
* Added planning feedback to gui, refactored states tab

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* porting to new RobotState API
* more console output

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes
* Dependency for move_group_capabilities_base fixed.

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)
* port to new base class for planning_interface (using planning contexts)

0.4.5 (2013-07-03)
------------------
* Fixed for moveit_msgs/JointLimits.h no such file or directory

0.4.4 (2013-06-26)
------------------
* fix `#259 <https://github.com/ros-planning/moveit_ros/issues/259>`_ and `#260 <https://github.com/ros-planning/moveit_ros/issues/260>`_.
