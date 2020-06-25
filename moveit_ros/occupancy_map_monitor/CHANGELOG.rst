^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_occupancy_map_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [fix]   Add error message on failure to initialize occupancy map monitor (`#1873 <https://github.com/ros-planning/moveit/issues/1873>`_)
* [fix]   Update occupancy grid when loaded from file (`#1594 <https://github.com/ros-planning/moveit/issues/1594>`_)
* [maint] Apply clang-tidy fix (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Windows build fixes
  * Fix header inclusion and other MSVC build errors (`#1636 <https://github.com/ros-planning/moveit/issues/1636>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 for portability (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [maint] move occupancy_map_monitor into its own package (`#1533 <https://github.com/ros-planning/moveit/issues/1533>`_)
* Contributors: Bjar Ne, Dale Koenig, Raphael Druon, Robert Haschke, Sean Yen, Simon Schmeisser, Yu, Yan, jschleicher