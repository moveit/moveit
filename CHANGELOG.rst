^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ikfast
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2014-10-07)
------------------
* Search for cheapest IK solution
* Specify search mode during code generation
* Fix issue: min_count must be negative (negative direction was ignored!)
* Added G.A. vd. Hoorn as second package maintainer
* Fix for issue `#34 <https://github.com/davetcoleman/moveit_ikfast/issues/34>`_: add liblapack-dev dependency to generated pkg manifest
* scripts: add dependency on liblapack-dev to generated pkg manifest. Fix `#34 <https://github.com/davetcoleman/moveit_ikfast/issues/34>`_.
* Use getIkType() in getPositionIK()
* Added support for the Translation3D-type IK
* Fix for missing run_depends in generated pkg manifest
* Make sure run_depends get added to pkg manifest as well. Fix `#30 <https://github.com/davetcoleman/moveit_ikfast/issues/30>`_.
  Short-circuit evaluation of the statement determining the value of
  'modified_pkg' caused the second call to update_deps(.., 'run_depend', ..)
  to never happen. Using binary or to work around that.
* Fixes for 23, 27 and 28
* Eigen is not actually a dependency anymore, so remove build_dep on cmake_modules.
* Remove dependency on deprecated tip_frame_ variable
* scripts: any change to run or build depends should trigger manifest updating.
* templates: include planning group name in plugin desc install rule. Fix `#28 <https://github.com/davetcoleman/moveit_ikfast/issues/28>`_.
* templates: remove superfluous link_directories(). Fix `#23 <https://github.com/davetcoleman/moveit_ikfast/issues/23>`_.
* templates: cmake_modules provides FindEigen, so explicitly depend on it. Fix `#27 <https://github.com/davetcoleman/moveit_ikfast/issues/27>`_.
  Also change order of `find_package(..)` statements to reflect this and
  update dependency checking in pkg generator script to allow for different
  run and build depends (`cmake_modules` is only a `build_depend`).
* Remove dependency on deprecated tip_frame_ variable (now supports multiple tip frames)
* Added support for the Translation3D-type IK
  Modified the ikfast moveit template to work with OpenRAVE's
  Translation3D-type IK. This is useful for manipulators with few DOF
  (i.e., 3 or 4) that just want to go to a certain position without caring
  so much about the end effector's orientation.
* Contributors: Dave Coleman, G.A. vd. Hoorn, Ioan A Sucan, Nicolas Alt, Steve Levine, gavanderhoorn, nalt

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
