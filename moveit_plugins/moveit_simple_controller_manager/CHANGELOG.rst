^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_simple_controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.7 (2017-04-03)
------------------
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* Contributors: Dave Coleman

0.8.6 (2017-03-08)
------------------

0.8.4 (2017-02-06)
------------------

0.8.3 (2016-08-19)
------------------

0.5.7 (2016-01-30)
------------------
* expose headers of moveit_simple_controller_manager
* Removed redundant logging information
* More informative warning message about multi-dof trajectories.
* Contributors: Dave Coleman, Dave Hershberger, Mathias Luedtke

0.5.6 (2014-03-23)
------------------
* Allow simple controller manager to ignore virtual joints without failing
* Contributors: Dave Coleman

0.5.5 (2013-09-30)
------------------
* properly fill in the gripper command effort
* allow trajectories with >1 points, use the last point of any trajectory
* added better error reporting for FollowJointTrajectoryControllers

0.5.4 (2013-09-24)
------------------

0.5.3 (2013-09-23)
------------------
* make things a bit more robust
* make headers and author definitions aligned the same way; white space fixes
* fix `#1 <https://github.com/ros-planning/moveit_plugins/issues/1>`_

0.5.1 (2013-07-30)
------------------
* ns parameter is now action_ns, get rid of defaults

0.5.0 (2013-07-16)
------------------
* white space fixes (tabs are now spaces)

0.4.1 (2013-07-03)
------------------
* minor updates to package.xml

0.4.0 (2013-06-06)
------------------
* debs look good, bump to 0.4.0

0.1.0 (2013-06-05)
------------------
* add metapackage, clean up build in controller manager
* remove the now dead loaded controller stuff
* break out follow/gripper into separate headers
* initial working version
