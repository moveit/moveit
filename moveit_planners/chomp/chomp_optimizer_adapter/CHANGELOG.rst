^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_chomp_optimizer_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2021-05-23)
------------------
* Enable mesh filter (`#2448 <https://github.com/ros-planning/moveit/issues/2448>`_)
* Sanitize CHOMP initialization method parameter (`#2540 <https://github.com/ros-planning/moveit/issues/2540>`_)
* Contributors: Cong Liu, Jafar Abdi

1.0.7 (2020-11-20)
------------------

1.0.6 (2020-08-19)
------------------
* [maint] Migrate to clang-format-10
* Contributors: Robert Haschke

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [maint] Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* Contributors: Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------
* [fix] Chomp planning adapter: Fix spurious error message Fix "Found empty JointState message"
* Contributors: Robert Haschke

1.0.1 (2019-03-08)
------------------
* [fix] segfault in chomp adapter (`#1377 <https://github.com/ros-planning/moveit/issues/1377>`_)
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* Contributors: Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [maintenance] Moved CHOMP optimizer into own package (`#1251 <https://github.com/ros-planning/moveit/issues/1251>`_)
* rename default_planner_request_adapters/CHOMPOptimizerAdapter -> chomp/OptimizerAdapter
* Contributors: Robert Haschke
