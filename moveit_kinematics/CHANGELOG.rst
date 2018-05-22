^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#906 <https://github.com/ros-planning/moveit/issues/906>`_ from ubi-agni/compile-fixes
  various fixes for melodic build
* KDL solvers: provide dummy updateInternalDataStructures()
  ... for new abstract method in KDL 1.4.0 of Melodic
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
* update include statements to use new pluginlib and class_loader headers (`#827 <https://github.com/ros-planning/moveit/issues/827>`_)
* [Fix] switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
  * [Fix] switch to ROS_LOGGER from CONSOLE_BRIDGE
  * [Fix] clang format
  * [Fix] manually fix bad clang-formatting in strings
* Merge pull request `#863 <https://github.com/ros-planning/moveit/issues/863>`_ from ubi-agni/fix-buid-farm-issues
  * fix various cmake warnings
  * remove not required test dependency
  * replaced 2nd find_package(catkin ...)
* fix various cmake warnings
* Improve ikfast kinematics plugin (`#808 <https://github.com/ros-planning/moveit/issues/808>`_)
  * Initialize constant num_joints\_ in constructor.
  * Check size of input array in getPositionFK.
  * Fix implementation error.
  * Add name variable to ikfast plugin.
* Cached ik kinematics plugin (`#612 <https://github.com/ros-planning/moveit/issues/612>`_)
  add caching wrapper for IK solvers
  * - Remove OMPL dependency, add slightly modified versions of OMPL code to this repo
  - Change camelCase variable names to underscore_names
  - Use /** ... */ for doxygen comments instead of /// ...
  - Fix memory leak
  - Various small fixes
  * handle optional TRAC-IK dependency a different way
  * remove OMPL_INCLUDE_DIRS
  * include trac_ik headers optionally *after* moveit includes
  Sorting dependency-includes via catkin breaks either way around
  with multiple `find_package` calls and can always break
  overlaying workspaces.
  However, ${catkin_INCLUDE_DIRS} contains many more dependencies
  than ${trac_ik_kinematics_plugin_INCLUDE_DIRS}, so avoid more
  breakage than necessary by including catkin_INCLUDE_DIRS first.
  * add missing include
  * Don't write configured file to build dir. Can't write to src dir either since it might be read-only. Instead, just comment out the parts that are for development / testing purposes only.
  * ik cache: use lookupParam for plugin parameters
* Contributors: Ian McMahon, Mark Moll, Mikael Arguedas, Robert Haschke, Xiaojian Ma, martiniil

0.9.11 (2017-12-25)
-------------------
* Merge pull request `#714 <https://github.com/ros-planning/moveit/issues/714>`_ from henhenhen/kinetic-devel_lookup-param
  Use lookupParam() in kinematics plugins
* Replace param() with lookupParam() in srv_kinematics_plugin
* Replace param() with lookupParam() in lma_kinematics_plugin
* Replace param() with lookupParam() in kdl_kinematics_plugin
* Replace param() with lookupParam() in ikfast_kinematics_plugin
* Remove redundant parameter query
* Contributors: Henning Kayser, Isaac I.Y. Saito

0.9.10 (2017-12-09)
-------------------
* [fix][kinetic onward] Fix create_ikfast_moveit_plugin to comply with format 2 of the package.xml. Remove collada_urdf dependency `#666 <https://github.com/ros-planning/moveit/pull/666>`_
* [fix] create_ikfast_moveit_plugin: fixed directory variable for templates that were moved to ikfast_kinematics_plugin `#620 <https://github.com/ros-planning/moveit/issues/620>`_
* [improve] IKFastTemplate: Expand solutions to full joint range in searchPositionIK `#598 <https://github.com/ros-planning/moveit/issues/598>`_
* [improve] IKFastTemplate: searchPositionIK now returns collision-free solution which is nearest to seed state. (`#585 <https://github.com/ros-planning/moveit/issues/585>`_)
* Contributors: Dennis Hartmann, G.A. vd. Hoorn, Michael Görner, fsuarez6

0.9.9 (2017-08-06)
------------------
* [improve] Modify ikfast_template for getPositionIK single solution results (`#537 <https://github.com/ros-planning/moveit/issues/537>`_)
* Contributors: nsnitish

0.9.8 (2017-06-21)
------------------
* [build] ikfast_kinematics_plugin: Write XML files as UTF-8 (`#514 <https://github.com/ros-planning/moveit/issues/514>`_)
* [build] adjust cmake_minimum_required for add_compile_options (`#521 <https://github.com/ros-planning/moveit/issues/521>`_)
* [build] ikfast_kinematics_plugin: Add c++11 compile option. This is required for Kinetic.
* Contributors: Martin Guenther, Michael Goerner

0.9.7 (2017-06-05)
------------------
* [fix][Kinetic+] ikfast_kinematics_plugin: Add c++11 compile option `#515 <https://github.com/ros-planning/moveit/pull/515>`_
* [fix][Indigo] moveit_kinematics Eigen3 dependency (`#470 <https://github.com/ros-planning/moveit/issues/470>`_)
* Contributors: Martin Guenther, YuehChuan

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
* [fix] Replace unused service dependency with msg dep (`#361 <https://github.com/ros-planning/moveit/issues/361>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.9.0 (2016-10-19)
------------------
* Add dependency on new moveit_kinematics package
* Move moveit_ikfast into moveit_kinematics
* Moved kinematics plugins to new pkg moveit_kinematics
* Contributors: Dave Coleman

0.8.3 (2016-08-21)
------------------
