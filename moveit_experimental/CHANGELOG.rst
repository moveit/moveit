^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_experimental
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.3 (2016-12-20)
------------------
* [fix] exported plugin xml for collision_distance_field (`#280 <https://github.com/ros-planning/moveit/issues/280>`_)
* [fix] CHOMP planner and CollisionDistanceField (`#155 <https://github.com/ros-planning/moveit/issues/155>`_)
* [maintenance] add full VERSIONs / SONAMEs to all libraries (`#273 <https://github.com/ros-planning/moveit/issues/273>`_)
* [maintenance] Auto code formatted Indigo branch using clang-format (`#313 <https://github.com/ros-planning/moveit/issues/313>`_)
* Contributors: Chittaranjan Srinivas Swaminathan, Dave Coleman, Michael Goerner, Robert Haschke

0.9.3 (2016-11-16)
------------------
* changelog 0.9.3
* Merge pull request `#330 <https://github.com/ros-planning/moveit/issues/330>`_ from davetcoleman/kinetic-package.xml
  Updated package.xml maintainers and author emails
* Updated package.xml maintainers and author emails
* Contributors: Dave Coleman, Ian McMahon, Isaac I.Y. Saito

0.9.2 (2016-11-05)
------------------
* changelog 0.9.2
* Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito

0.9.1 (2016-10-21)
------------------
* add full VERSIONs / SONAMEs to all libraries (`#273 <https://github.com/ros-planning/moveit/issues/273>`_)
  * add full VERSIONs / SONAMEs to all core libraries
  As a result the libraries do not install as `libmoveit_xyz.so` anymore,
  but as `libmoveit_xyz.so.${MOVEIT_VERSION}` and only provide `libmoveit_xyz.so`
  as a symlink pointing to the versioned file.
  Because this sets each library's SONAME to the *full version*, this enforces
  that *every* binary links with the versioned library file from now on and
  has to be relinked with *each* new release of MoveIt!.
  The alternative would be to set the SONAME to `$MAJOR.$MINOR` and ignore the patch version,
  but because we currently stay with one `$MAJOR.$MINOR` number within each ROS distribution,
  we had (and likely will have) ABI changes in the `$PATCH` version releases too.
  The reason for this commit is that it is practically impossible to maintain full ABI compatibility
  within each ROS distribution and still add the the features/patches the community asks for.
  This has resulted in more than one ABI-incompatible MoveIt! release in the recent past
  within a ROS distribution. Because the libraries have not been versioned up to now,
  there was no way to indicate the incompatible changes and users who did not rebuild
  their whole workspace with the new release encountered weird and hard-to-track segfaults
  or broken behavior.
  * add SONAMES to all non-core libraries too
* Version down and consolidattion once again; having an issue with catkin_prepare_release to bump all packages (asked at http://answers.ros.org/question/245969/catkin_prepare_release-not-bumping-packages-in-a-certain-folder).
* 0.9.0
* More version consolidattion for all package.xml in the moveit repo (addition to https://github.com/ros-planning/moveit/commit/4d66734926dc62eec9e0d5bbcab7c3e918dfed46. Seems like catkin_prepare_release requires all packages to be in equal version).
* Replace broken Eigen3 with correctly spelled EIGEN3 (`#254 <https://github.com/ros-planning/moveit/issues/254>`_)
  * Fix Eigen3 dependency throughout packages
  * Eigen 3.2 does not provide EIGEN3_INCLUDE_DIRS, only EIGEN3_INCLUDE_DIR
* fix exported plugin xml for collision_distance_field (`#280 <https://github.com/ros-planning/moveit/issues/280>`_)
  Otherwise the xml can not be found on an installed workspace
* Cleanup readme (`#258 <https://github.com/ros-planning/moveit/issues/258>`_)
* Convert collision_distance_field to std::shared_ptr.
* Use shared_ptr typedefs in collision_distance_field and chomp.
* Use srdf::ModelPtr typedefs.
* Switch to std::make_shared.
* [moveit_experimental] Fix incorrect dependency on FCL in kinetic
  [moveit_experimental] Fix Eigen3 warning
* Remove deprecated package shape_tools
  Fix OcTree boost::shared_ptr
  Remove deprecated CMake dependency
  Fix distanceRobot() API with verbose flags
* Fix CHOMP planner and CollisionDistanceField (`#155 <https://github.com/ros-planning/moveit/issues/155>`_)
  * Copy collision_distance_field package
  * Resurrect chomp
  * remove some old Makefiles and manifests
  * Correct various errors
  * Code formatting, author, description, version, etc
  * Add definitions for c++11. Nested templates problem.
  * Add name to planner plugin.
  * Change getJointModels to getActiveJointModels.
  * Call robot_state::RobotState::update in setRobotStateFromPoint.
  * Create README.md
  * Improve package.xml, CMake config and other changes suggested by jrgnicho.
  * Remove some commented code, add scaling factors to computeTimeStampes
  * Add install targets in moveit_experimental and chomp
  * Add install target for headers in chomp pkgs.
  * Remove unnecessary debugging ROS_INFO.
  * Port collision_distance_field test to indigo.
  * Remove one assertion that makes collision_distance_field test to fail.
* Use urdf::*SharedPtr instead of boost::shared_ptr
  urdfdom_headers uses C++ std::shared_ptr. As it exports it as custom
  *SharedPtr type, we can use them to stay compatible.
  Note that there is no std:shared_ptr<const urdf::ModelInterface>
  typedef, so I replaced it with urdf::ModelInterfaceSharedPtr (loosing a
  const).
  Also, there is no conversion from boost::shared_ptr<urdf::Model> to
  std:shared_ptr<urdf::ModelInterface>, so I used a preprocessor
  directive.
* fetch moveit_resources path at compile time
  using variable MOVEIT_TEST_RESOURCES_DIR provided by config.h
  instead of calling ros::package::getPath()
* adapted paths to moveit_resources
  (renamed folder moveit_resources/test to moveit_resources/pr2_description)
* Contributors: Chittaranjan Srinivas Swaminathan, Dave Coleman, Isaac I.Y. Saito, Jochen Sprickerhof, Maarten de Vries, Michael Görner, Robert Haschke

0.8.3 (2016-08-21)
------------------
* this was implemented in a different way
* add kinematics constraint aware
* add kinematics_cache
* Update README.md
* Update README.md
* Create README.md
* copy collision_distance_field from moveit_core
* rename some headers
* add collision_distance_field_ros
* add kinematics_planner_ros
* added kinematics_cache_ros from moveit-ros
* moved from moveit_core
* Contributors: Ioan Sucan, isucan
