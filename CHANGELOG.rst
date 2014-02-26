^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ikfast
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.7 (2014-02-25)
------------------
* Fix issue `#16 <https://github.com/ros-planning/moveit_ikfast/issues/16>`_: install 'templates' directory into pkg share directory
* Fix issue `#21 <https://github.com/ros-planning/moveit_ikfast/issues/21>`_: move catkin pkgs to catkin_depends in generated CMakeLists
* Fix issue `#18 <https://github.com/ros-planning/moveit_ikfast/issues/18>`_: remove install rule for ikfast.h from generated package CMakeLists.txt
* Contributors: Dave Coleman, gavanderhoorn

3.0.6 (2014-01-03)
------------------
* adding sphinx documentation
* added branch name to build status
* added travis build status indicator in README.md
* added travis support

3.0.5 (2013-09-23)
------------------
* use GetIkType() instead of #defines
  This removes the need to manually specify the IK type using `#define
  IKTYPE_...` and closes `#5 <https://github.com/ros-planning/moveit_ikfast/issues/5>`_.
* Fix to check all depencies are in package.xml
* Merge pull request `#10 <https://github.com/ros-planning/moveit_ikfast/issues/10>`_ from ros-planning/auto_update_script
* Merge pull request `#9 <https://github.com/ros-planning/moveit_ikfast/issues/9>`_ from fsuarez6/master
  Fix IKfast plugin undefined symbol poseMsgToKDL
* Check for file existance first
* Added auto update script for keeping your plugin up to date with changes in the kinematics plugin format
* Merge branch 'master' into multiple_plugins
* Added tf_conversions dependency to expose its functions to the plugin (e.g. poseMsgToKDL)
* fix whitespace

3.0.4 (2013-07-17)
------------------
* Changed order of parameters consistency_limits and solution to match the searchPositionIK signature.

3.0.3 (2013-07-17)
------------------
* fixed IKFast code generator template to fit MoveIt interface
* added error message in case of malformed SRDF
* fix IK and FK for IK types other than Transform6D
