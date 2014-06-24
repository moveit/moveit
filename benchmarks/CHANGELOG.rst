^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_benchmarks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.19 (2014-06-23)
-------------------
* benchmarks: add missing include.
* Fix broken log & output statements.
  - Address [cppcheck: coutCerrMisusage] and [-Werror=format-extra-args] errors.
  - ROS_ERROR -> ROS_ERROR_NAMED.
  - Print size_t values portably.
* Address [-Wsign-compare] warning.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Benjamin Chretien

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------

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
* Cleaned up var names and debug output

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes

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
* Fixed per Ioan's code review
* Code cleanup
* Merge branch 'groovy-devel' of github.com:davetcoleman/moveit_ros into groovy-devel
* Changed for fractional factorial analysis
* More advanced parameter sweeping implmented, workspace bounds added
* Added parameter sweeping to benchmarking
* Added ability to store the goal name - the query, constraint, traj constraint, etc
* Added new command line arguments and ability to export all experiments to csv file

0.4.4 (2013-06-26)
------------------
* bugfixes

