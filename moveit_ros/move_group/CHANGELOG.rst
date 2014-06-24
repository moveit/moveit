^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_move_group
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
