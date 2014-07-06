^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.7 (2014-07-05)
------------------
* Merge pull request `#21 <https://github.com/ros-planning/moveit_commander/issues/21>` from pirobot/hydro-devel
  Added set_support_surface_name function to move_group.py
* Added set_support_surface_name function to move_group.py
* Contributors: Patrick Goebel, Sachin Chitta

0.5.6 (2014-03-24)
------------------
* Added the calls necessary to manage path constraints. 
* fix joint and link acces on __getattr__  when trying to acces a joint and its paramaters throught
* Contributors: Acorn, Emili Boronat, Sachin Chitta

0.5.5 (2014-02-27)
------------------
* adding get for active joints
* Contributors: Acorn, Sachin Chitta

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
