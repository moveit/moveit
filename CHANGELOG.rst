^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ikfast
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
