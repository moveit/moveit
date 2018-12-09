^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_warehouse
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.6 (2018-12-09)
-------------------

0.10.2 (2018-10-24)
-------------------
* [fix] Text refrences to MoveIt! (`#1020 <https://github.com/ros-planning/moveit/issues/1020>`_)
* [enhancement] warehouse: added params for timeout + #retries (`#1008 <https://github.com/ros-planning/moveit/issues/1008>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* Contributors: Mohmmad Ayman, Robert Haschke, dg-shadow, mike lautman

0.10.1 (2018-05-25)
-------------------
* [maintenance] migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* Contributors: Ian McMahon

0.9.10 (2017-12-09)
-------------------
* [package.xml] Add a release-maintainer. Cleanup `#649 <https://github.com/ros-planning/moveit/pull/649>`_

0.9.6 (2017-04-12)
------------------
* [fix] warehouse services (`#474 <https://github.com/ros-planning/moveit/issues/474>`_)
* Contributors: Beatriz Leon

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_ 
* Contributors: Dave Coleman

0.9.4 (2017-02-06)
------------------
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

0.6.6 (2016-06-08)
------------------
* Removed trailing whitespace from entire repository
* comments addressed
* changed to global node handle so warehouse connection params work correctly
* camelCase
* removed extraneous includes
* added delete and rename
* now takes port + host from param server
* removed defunct code
* checks db connection is good
* services advertised
* Contributors: Dave Coleman, dg

0.6.5 (2015-01-24)
------------------
* update maintainers
* Contributors: Michael Ferguson

0.5.17 (2014-03-22)
-------------------
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.5 (2013-09-23)
------------------
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)
