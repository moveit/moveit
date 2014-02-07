^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.4 (2014-02-06)
------------------

* Install moveit_commander_cmdline.py into package specific directory, not to global bin.
* Fix typos in comments

0.5.3 (2014-01-03)
------------------
* work around name bug
  move group interface python programs cannot be launched from launch files if
  the __name:= argument is used.  This works around the problem and allows using
  launch files to launch python moveit programs.
* Added Travis Continuous Integration

0.5.2 (2013-09-23)
------------------
* add support for setting joint targets from approximate IK
* no longer depend on manipulation_msgs
* expand functionality of MoveGroupInterface

0.5.1 (2013-08-13)
------------------
* make pick() more general
* use msg serialization
* use new attach / detach operations
* fix header for demo code
* Duration class bug fixed in commander conversion.

0.5.0 (2013-07-18)
------------------
* move msgs to common_msgs
* fixed ground command
