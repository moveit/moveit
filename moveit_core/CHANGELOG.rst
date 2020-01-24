^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.18 (2020-01-24)
-------------------
* [maintanance] Removed dependency moveit_core -> moveit_experimental again
* Contributors: Robert Haschke

0.9.17 (2019-07-09)
-------------------

0.9.16 (2019-06-29)
-------------------
* [fix]         Invert waypoint velocities on reverse (`#1335 <https://github.com/ros-planning/moveit/issues/1335>`_)
* [fix]         Added missing robot state update to iterative spline parameterization (`#1298 <https://github.com/ros-planning/moveit/issues/1298>`_)
* [fix]         Fix race condition when world object was removed (`#1306 <https://github.com/ros-planning/moveit/issues/1306>`_)
* [fix]         Fixed calculation of Jacobian for prismatic joints (`#1192 <https://github.com/ros-planning/moveit/issues/1192>`_)
* [maintanance] Improve code quality (`#1340 <https://github.com/ros-planning/moveit/issues/1340>`_)
* [maintanance] Resolve catkin lint issues (`#1137 <https://github.com/ros-planning/moveit/issues/1137>`_)
* [maintanance] Cleanup Chomp packages (`#1282 <https://github.com/ros-planning/moveit/issues/1282>`_)
                Moved collision_distance_field to moveit_core
                Add dependency moveit_core -> moveit_experimental
* [maintanance] Add (manual) coverage analysis (`#1133 <https://github.com/ros-planning/moveit/issues/1133>`_)
* [maintanance] ConstraintSampler cleanup (`#1247 <https://github.com/ros-planning/moveit/issues/1247>`_)
* [maintanance] Fix clang issues (`#1233 <https://github.com/ros-planning/moveit/issues/1233>`_, `#1214 <https://github.com/ros-planning/moveit/issues/1214>`_)
* [feature]     Helper function to construct constraints from ROS params (`#1253 <https://github.com/ros-planning/moveit/issues/1253>`_)
* [feature]     Allow appending of only a part of a trajectory (`#1213 <https://github.com/ros-planning/moveit/issues/1213>`_)
* Contributors: Alexander Gutenkunst, Ludovic Delval, Martin Oehler, Michael Görner, Mike Lautman, Milutin Nikolic, Robert Haschke

0.9.15 (2018-10-29)
-------------------
* [improvement] Exploit the fact that our transforms are isometries (instead of general affine transformations). `#1091 <https://github.com/ros-planning/moveit/issues/1091>`_
* [code] cleanup, improvements (`#1099 <https://github.com/ros-planning/moveit/issues/1099>`_, `#1108 <https://github.com/ros-planning/moveit/issues/1108>`_)
* Contributors: Robert Haschke, Simon Schmeisser

0.9.14 (2018-10-24)
-------------------

0.9.13 (2018-10-24)
-------------------
* [fix] TFs in subgroups of rigidly-connected links (`#912 <https://github.com/ros-planning/moveit/issues/912>`_)
* [fix] Chomp package handling issue `#1086 <https://github.com/ros-planning/moveit/issues/1086>`_ that was introduced in `ubi-agni/hotfix-#1012 <https://github.com/ubi-agni/hotfix-/issues/1012>`_
* [fix] CurrentStateMonitor update callback for floating joints to handle non-identity joint origins `#984 <https://github.com/ros-planning/moveit/issues/984>`_
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix] illegal pointer access (`#989 <https://github.com/ros-planning/moveit/issues/989>`_)
* [fix] reset moveit_msgs::RobotState.is_diff to false (`#968 <https://github.com/ros-planning/moveit/issues/968>`_) This fixes a regression introduced in `#939 <https://github.com/ros-planning/moveit/issues/939>`_.
* [fix] continous joint limits are always satisfied (`#729 <https://github.com/ros-planning/moveit/issues/729>`_)
* [maintenance] using LOGNAME variable rather than strings (`#1079 <https://github.com/ros-planning/moveit/issues/1079>`_)
* [capability][chomp] Addition of CHOMP planning adapter for optimizing result of other planners (`#1012 <https://github.com/ros-planning/moveit/issues/1012>`_)
* [enhancement] Add missing distance check functions to allValid collision checker (`#986 <https://github.com/ros-planning/moveit/issues/986>`_)
* [enhancement] Allow chains to have only one active joint (`#983 <https://github.com/ros-planning/moveit/issues/983>`_)
* [enhancement] collision_detection convenience (`#957 <https://github.com/ros-planning/moveit/issues/957>`_)
* [doc] Document why to use only one IK attempt in computeCartesianPath (`#1076 <https://github.com/ros-planning/moveit/issues/1076>`_)
* Contributors: Adrian Zwiener, Andrey Troitskiy, Dave Coleman, Jonathan Binney, Michael Görner, Mike Lautman, Mohmmad Ayman, Raghavender Sahdev, Robert Haschke, Simon Schmeisser, dcconner, mike lautman

0.9.12 (2018-05-29)
-------------------
* consider max linear+rotational eef step in computeCartesianPath() (`#884 <https://github.com/ros-planning/moveit/issues/884>`_)
* clang-tidy moveit_core (`#880 <https://github.com/ros-planning/moveit/issues/880>`_) (`#911 <https://github.com/ros-planning/moveit/issues/911>`_)
* Allow to retrieve Jacobian of a child link of a move group. (`#877 <https://github.com/ros-planning/moveit/issues/877>`_)
* Switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* Add ability to request detailed distance information from fcl (`#662 <https://github.com/ros-planning/moveit/issues/662>`_)
* allow checking for absolute joint-space jumps in Cartesian path (`#843 <https://github.com/ros-planning/moveit/issues/843>`_)
* Simplify adding colored CollisionObjects (`#810 <https://github.com/ros-planning/moveit/issues/810>`_)
* updateMimicJoint(group->getMimicJointModels()) -> updateMimicJoints(group)
* improve RobotState::updateStateWithLinkAt() (`#765 <https://github.com/ros-planning/moveit/issues/765>`_)
* fix computation of shape_extents\_ of links w/o shapes (`#766 <https://github.com/ros-planning/moveit/issues/766>`_)
* RobotModel::getRigidlyConnectedParentLinkModel()
  ... to compute earliest parent link that is rigidly connected to a given link
* Iterative cubic spline interpolation (`#441 <https://github.com/ros-planning/moveit/issues/441>`_)
* Contributors: Bryce Willey, Ian McMahon, Ken Anderson, Levi Armstrong, Maarten de Vries, Martin Pecka, Michael Görner, Mike Lautman, Patrick Holthaus, Robert Haschke, Victor Lamoine, Xiaojian Ma

0.9.11 (2017-12-25)
-------------------
* [fix] #723; attached bodies are not shown in trajectory visualization anymore `#724 <https://github.com/ros-planning/moveit/issues/724>`_
* [fix] Shortcomings in kinematics plugins `#714 <https://github.com/ros-planning/moveit/issues/714>`_
* Contributors: Henning Kayser, Michael Görner, Robert Haschke

0.9.10 (2017-12-09)
-------------------
* [fix] Add missing logWarn argument (`#707 <https://github.com/ros-planning/moveit/issues/707>`_)
* [fix] IKConstraintSampler: Fixed transform from end-effector to ik chain tip. `#582 <https://github.com/ros-planning/moveit/issues/582>`_
* [fix] robotStateMsgToRobotState: is_diff==true => not empty `#589 <https://github.com/ros-planning/moveit/issues/589>`_
* [capability] Multi DOF Trajectory only providing translation not velocity (`#555 <https://github.com/ros-planning/moveit/issues/555>`_)
* [capability] Adds parameter lookup function for kinematics plugins (`#701 <https://github.com/ros-planning/moveit/issues/701>`_)
* [improve] Make operator bool() explicit `#696 <https://github.com/ros-planning/moveit/pull/696>`_
* [improve] Get msgs from Planning Scene `#663 <https://github.com/ros-planning/moveit/issues/663>`_
* [improve] moveit_core: export DEPENDS on LIBFCL `#632 <https://github.com/ros-planning/moveit/pull/632>`_
* [improve] RobotState: Changed multi-waypoint version of computeCartesianPath to test joint space jumps after all waypoints are generated. (`#576 <https://github.com/ros-planning/moveit/issues/576>`_)
* [improve] Better debug output for IK tip frames (`#603 <https://github.com/ros-planning/moveit/issues/603>`_)
* [improve] New debug console colors YELLOW PURPLE (`#604 <https://github.com/ros-planning/moveit/issues/604>`_)
* Contributors: Dave Coleman, Dennis Hartmann, Henning Kayser, Isaac I.Y. Saito, Jorge Nicho, Michael Görner, Phil, Sarah Elliott, Simon Schmeisser, TroyCordie, v4hn

0.9.9 (2017-08-06)
------------------
* [fix][moveit_core] segfault due to missing string format parameter. (`#547 <https://github.com/ros-planning/moveit/issues/547>`_)
* [fix][moveit_core] doc-comment for robot_state::computeAABB (`#516 <https://github.com/ros-planning/moveit/issues/516>`_)
* Contributors: Martin Pecka, henhenhen

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------
* [fix] checks for empty name arrays messages before parsing the robot state message data (`#499 <https://github.com/ros-planning/moveit/issues/499>`_)
* Contributors: Jorge Nicho, Michael Goerner

0.9.6 (2017-04-12)
------------------
* [fix] PlanarJointModel::getVariableRandomPositionsNearBy (`#464 <https://github.com/ros-planning/moveit/issues/464>`_)
* Contributors: Tamaki Nishino

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] PlanningScene: Don't reset color information of existing objects when new entries are added (`#410 <https://github.com/ros-planning/moveit/issues/410>`_)
* [fix] update link transforms in UnionConstraintSampler::project (`#384 <https://github.com/ros-planning/moveit/issues/384>`_)
* [capability Addition of Set Joint Model Group Velocities and Accelerations Functions (`#402 <https://github.com/ros-planning/moveit/issues/402>`_)
* [capability] time parameterization: use constants (`#380 <https://github.com/ros-planning/moveit/issues/380>`_)
* [enhancement] multiple shapes in an attached collision object `#421 <https://github.com/ros-planning/moveit/pull/421>`_
* [maintenance] Use static_cast to cast to const. (`#433 <https://github.com/ros-planning/moveit/issues/433>`_)
* [maintenance] ompl_interface: uniform & simplified handling of the default planner (`#371 <https://github.com/ros-planning/moveit/issues/371>`_)
* Contributors: Dave Coleman, Maarten de Vries, Michael Goerner, Mike Lautman, Ruben

0.9.3 (2016-11-16)
------------------
* [fix] Replace unused service dependency with msg dep (`#361 <https://github.com/ros-planning/moveit/issues/361>`_)
* [fix] cleanup urdfdom compatibility (`#319 <https://github.com/ros-planning/moveit/issues/319>`_)
* [fix] Fix missing compatibility header for Wily `#364 <https://github.com/ros-planning/moveit/issues/364>`_)
* [enhancement] Improved RobotState feedback for setFromIK() (`#342 <https://github.com/ros-planning/moveit/issues/342>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Robert Haschke

0.9.2 (2016-11-05)
------------------
* [Fix] CHANGELOG encoding for 0.9.1 (Fix `#318 <https://github.com/ros-planning/moveit/issues/318>`_). (`#327 <https://github.com/ros-planning/moveit/issues/327>`_)
* [Capability] compatibility to urdfdom < 0.4 (`#317 <https://github.com/ros-planning/moveit/issues/317>`_)
* [Capability] New isValidVelocityMove() for checking maximum velocity between two robot states given time delta
* [Maintenance] Travis check code formatting (`#309 <https://github.com/ros-planning/moveit/issues/309>`_)
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Isaac I. Y. Saito, Robert Haschke

0.8.2 (2016-06-17)
------------------
* [feat] planning_scene updates: expose success state to caller. This is required to get the information back for the ApplyPlanningSceneService. `#296 <https://github.com/ros-planning/moveit_core/issues/297>`_
* [sys] replaced cmake_modules dependency with eigen
* Contributors: Michael Ferguson, Robert Haschke, Michael Goerner, Isaac I. Y. Saito

0.8.1 (2016-05-19)
------------------
* Corrected check in getStateAtDurationFromStart (cherry-picking `#291 <https://github.com/ros-planning/moveit_core/issues/291>`_ from indigo-devel)
* Contributors: Hamal Marino

0.8.0 (2016-05-18)
------------------
* [feat] Added file and trajectory_msg to RobotState conversion functions `#267 <https://github.com/ros-planning/moveit_core/issues/267>`_
* [feat] Added setJointVelocity and setJointEffort functions `#261 <https://github.com/ros-planning/moveit_core/issues/261>`_
* [feat] KinematicsBase changes `#248 <https://github.com/ros-planning/moveit_core/issues/248>`_
* [feat] added an ik_seed_state argument to the new getPositionIK(...) method
* [feat] added new interface method for computing multiple ik solutions for a single pose
* [fix] RevoluteJointModel::computeVariablePositions `#282 <https://github.com/ros-planning/moveit_core/issues/282>`_
* [fix] getStateAtDurationFromStart would never execute as the check for number of waypoints was inverted `#289 <https://github.com/ros-planning/moveit_core/issues/289>`_
* [fix] Revert "Use libfcl-dev rosdep key in kinetic" `#287 <https://github.com/ros-planning/moveit_core/issues/287>`_
* [fix] memory leak in RobotState::attachBody `#276 <https://github.com/ros-planning/moveit_core/issues/276>`_. Fixing `#275 <https://github.com/ros-planning/moveit_core/issues/275>`_
* [fix] New getOnlyOneEndEffectorTip() function `#262 <https://github.com/ros-planning/moveit_core/issues/262>`_
* [fix] issue `#258 <https://github.com/ros-planning/moveit_core/issues/258>`_ in jade-devel `#266 <https://github.com/ros-planning/moveit_core/issues/266>`_
* [fix] Segfault in parenthesis operator `#254 <https://github.com/ros-planning/moveit_core/issues/254>`_
* [fix] API Change of shape_tools `#242 <https://github.com/ros-planning/moveit_core/issues/242>`_
* [fix] Fixed bug in KinematicConstraintSet::decide that makes it evaluate only joint_constraints. `#250 <https://github.com/ros-planning/moveit_core/issues/250>`_
* [fix] Prevent divide by zero `#246 <https://github.com/ros-planning/moveit_core/issues/246>`_
* [fix] removed the 'f' float specifiers and corrected misspelled method name
* [fix] typo MULTIPLE_TIPS_NO_SUPPORTED -> MULTIPLE_TIPS_NOT_SUPPORTED
* [sys] Upgrade to Eigen3 as required in Jade `#293 <https://github.com/ros-planning/moveit_core/issues/293>`_
* [sys] [cmake] Tell the compiler about FCL include dirs `#263 <https://github.com/ros-planning/moveit_core/issues/263>`_
* [sys] Install static libs `#251 <https://github.com/ros-planning/moveit_core/issues/251>`_
* [enhance] Allow a RobotTrajectory to be initialized with a pointer joint model group `#245 <https://github.com/ros-planning/moveit_core/issues/245>`_
* [doc] Better documentation and formatting `#244 <https://github.com/ros-planning/moveit_core/issues/244>`_
* Contributors: Alexis Ballier, Bastian Gaspers, Christian Dornhege, Dave Coleman, Gary Servin, Ioan A Sucan, Isaac I.Y. Saito, Jim Mainprice, Levi Armstrong, Michael Ferguson, Mihai Pomarlan, Robert Haschke, Sachin Chitta, Sam Pfeiffer, Steven Peters, Severin Lemaignan, jrgnicho, ros-devel, simonschmeisser

0.6.15 (2015-01-20)
-------------------
* add ptr/const ptr types for distance field
* update maintainers
* Contributors: Ioan A Sucan, Michael Ferguson

0.6.14 (2015-01-15)
-------------------
* Add time factor to iterative_time_parametrization
* Contributors: Dave Coleman, Michael Ferguson, kohlbrecher

0.6.13 (2014-12-20)
-------------------
* add getShapePoints() to distance field
* update distance_field API to no longer use geometry_msgs
* Added ability to remove all collision objects directly through API (without using ROS msgs)
* Planning Scene: Ability to offset geometry loaded from stream
* Namespaced pr2_arm_kinematics_plugin tests to allow DEBUG output to be suppressed during testing
* Contributors: Dave Coleman, Ioan A Sucan, Michael Ferguson

0.6.12 (2014-12-03)
-------------------
* Merge pull request `#214 <https://github.com/ros-planning/moveit_core/issues/214>`_ from mikeferguson/collision_plugin
  moveit_core components of collision plugins
* Merge pull request `#210 <https://github.com/ros-planning/moveit_core/issues/210>`_ from davetcoleman/debug_model
  Fix truncated debug message
* Fixed a number of tests, all are now passing on buildfarm
* Merge pull request `#208 <https://github.com/ros-planning/moveit_core/issues/208>`_ from mikeferguson/update_fcl_api
  update to use non-deprecated call
* Contributors: Dave Coleman, Ioan A Sucan, Michael Ferguson

0.6.11 (2014-11-03)
-------------------
* Merge pull request `#204 <https://github.com/ros-planning/moveit_core/issues/204>`_ from mikeferguson/indigo-devel
  forward port `#198 <https://github.com/ros-planning/moveit_core/issues/198>`_ to indigo
* forward port `#198 <https://github.com/ros-planning/moveit_core/issues/198>`_ to indigo
* Contributors: Ioan A Sucan, Michael Ferguson

0.6.10 (2014-10-27)
-------------------
* Made setVerbose virtual in constraint_sampler so that child classes can override
* Manipulability Index Error for few DOF
  When the group has fewer than 6 DOF, the Jacobian is of the form 6xM and when multiplied by its transpose, forms a 6x6 matrix that is singular and its determinant is always 0 (or NAN if the solver cannot calculate it).
  Since calculating the SVD of a Jacobian is a costly operation, I propose to retain the calculation of the Manipulability Index through the determinant for 6 or more DOF (where it produces the correct result), but use the product of the singular values of the Jacobian for fewer DOF.
* Fixed missing test depends for tf_conversions
* Allow setFromIK() with multiple poses to single IK solver
* Improved debug output
* Removed duplicate functionality poseToMsg function
* New setToRandomPositions function with custom rand num generator
* Moved find_package angles to within CATKIN_ENABLE_TESTING
* Getter for all tips (links) of every end effector in a joint model group
* New robot state to (file) stream conversion functions
* Added default values for iostream in print statements
* Change PlanningScene constructor to RobotModelConstPtr
* Documentation and made printTransform() public
* Reduced unnecessary joint position copying
* Added getSubgroups() helper function to joint model groups
* Maintain ordering of poses in order that IK solver expects
* Added new setToRandomPositions function that allows custom random number generator to be specified
* Split setToIKSolverFrame() into two functions
* Add check for correct solver type
* Allowed setFromIK to do whole body IK solving with multiple tips
* Contributors: Acorn, Dave Coleman, Ioan A Sucan, Jonathan Weisz, Konstantinos Chatzilygeroudis, Sachin Chitta, hersh

0.5.10 (2014-06-30)
-------------------
* making Saucy and Trusty version of includes to be compatible with upstream packaging. re: https://github.com/ros/rosdistro/issues/4633
* Contributors: Tully Foote

0.5.9 (2014-06-23)
------------------
* Fixed bug in RevoluteJointModel::distance() giving large negative numbers.
* kinematics_base: added an optional RobotState for context.
* fix pick/place approach/retreat on indigo/14.04
* Fixed bug in RevoluteJointModel::distance() giving large negative numbers.
* IterativeParabolicTimeParameterization now ignores virtual joints.
* kinematics_base: added an optional RobotState for context.
* Removed check for multi-dof joints in iterative_time_parameterization.cpp.
* fix pick/place approach/retreat on indigo/14.04
* IterativeParabolicTimeParameterization now ignores virtual joints.
  When checking if all joints are single-DOF, it accepts multi-DOF joints only if they are
  also virtual.
* Fix compiler warnings
* Address [cppcheck: unreadVariable] warning.
* Address [cppcheck: postfixOperator] warning.
* Address [cppcheck: stlSize] warning.
* Address [-Wunused-value] warning.
* Address [-Wunused-variable] warning.
* Address [-Wreturn-type] warning.
* Address [-Wsign-compare] warning.
* Address [-Wreorder] warning.
* Allow joint model group to have use IK solvers with multiple tip frames
* KinematicsBase support for multiple tip frames and IK requests with multiple poses
* dynamics_solver: fix crashbug
  Ignore joint that does not exist (including the virtual joint if it is part of
  the group).
* Changed KinematicsBase::supportsGroup() to use a more standard call signature.
* Merged with hydro-devel
* Removed unnecessary error output
* Removed todo
* Added support for legacy IK calls without solution_callback
* Merge branch 'hydro-devel' into kinematic_base
* Changed KinematicsBase::supportsGroup() to use a more standard call signature.
* Added empty check.
* computeCartesianPath waypoints double-up fix
  computeCartesianPath appends full trajectories between waypoints when given a vector of waypoints. As trajectories include their endpoints, this leads to the combined trajectory being generated with duplicate points at waypoints, which can lead to pauses or stuttering.
  This change skips the first point in trajectories generated between waypoints.
* avoid unnecessary calculations
* Created supportsGroup() test for IK solvers
* from ros-planning/more-travis-tests
  More Travis test fixes.
* Commented out failing test.
  run_tests_moveit_ros_perception requires glut library, and thus a video card or X server, but I haven't had any luck making such things work on Travis.
* avoid unnecessary calculations
  If we are not going to use the missing vector then we should not create it
  (avoid an expensive operation).
* Code cleanup
* Allow joint model group to have use IK solvers with multiple tip frames
* Authorship
* Fixed missing removeSlash to setValues()
* Feedback and cleaned up comment lengths
* Cleaned up commit
* KinematicsBase support for multiple tip frames and IK requests with multiple poses
* More Travis test fixes.
  Switched test_constraint_samplers.cpp from build-time to run-time reference to moveit_resources.
  Added passing run_tests_moveit_core_gtest_test_robot_state_complex test to .travis.yml.
  Added 'make tests' to .travis.yml to make all tests, even failing ones.
* Contributors: Acorn Pooley, Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Dave Hershberger, Martin Szarski, Michael Ferguson, Sachin Chitta, hersh, sachinc

0.5.8 (2014-03-03)
------------------
* Dix bad includes after upstream catkin fix
* update how we find eigen: this is needed for indigo
* Contributors: Ioan A Sucan, Dirk Thomas, Vincent Rabaud

0.5.7 (2014-02-27)
------------------
* Constraint samplers bug fix and improvements
* fix for reverting PR #148
* Fix joint variable location segfault
* Better enforce is_valid as a flag that indicated proper configuration has been completed, added comments and warning
* Fix fcl dependency in CMakeLists.txt
* Fixed asymmetry between planning scene read and write.
* Improved error output for state conversion
* Added doxygen for RobotState::attachBody() warning of danger.
* Improved error output for state converstion
* Debug and documentation
* Added new virtual getName() function to constraint samplers
* Made getName() const with static variable
* KinematicsMetrics crashes when called with non-chain groups.
* Added prefixes to debug messages
* Documentation / comments
* Fixed asymmetry between planning scene read and write.
* Added new virtual getName function to constraint samplers for easier debugging and plugin management
* KinematicsMetrics no longer crashes when called with non-chain groups.
* Added doxygen for RobotState::attachBody() warning of danger.
* resolve full path of fcl library
  Because it seems to be common practice to ignore ${catkin_LIBRARY_DIRS}
  it's more easy to resolve the full library path here instead.
* Fix fcl dependency in CMakeLists.txt
  See http://answers.ros.org/question/80936 for details
  Interestingly collision_detection_fcl already uses the correct
  variable ${LIBFCL_LIBRARIES} although it wasn't even set before
* Contributors: Dave Coleman, Dave Hershberger, Ioan A Sucan, Sachin Chitta, sachinc, v4hn

0.5.6 (2014-02-06)
------------------
* fix mix-up comments, use getCollisionRobotUnpadded() since this function is checkCollisionUnpadded.
* Updated tests to new run-time usage of moveit_resources.
* robot_state: comment meaning of default
* Trying again to fix broken tests.
* document RobotState methods
* transforms: clarify comment
* Fixed build of test which depends on moveit_resources.
* Removed debug-write in CMakeLists.txt.
* Added running of currently passing tests to .travis.yml.
* Add kinematic options when planning for CartesianPath
* -Fix kinematic options not getting forwarded, which can lead to undesired behavior in some cases
* Added clarifying doxygen to collision_detection::World::Object.

0.5.5 (2013-12-03)
------------------
* Fix for computing jacobian when the root_joint is not an active joint.
* RobotState: added doxygen comments clarifying action of attachBody().
* Always check for dirty links.
* Update email addresses.
* Robot_state: fix copy size bug.
* Corrected maintainer email.
* Fixed duration in robottrajectory.swap.
* Fixing distance field bugs.
* Compute associated transforms bug fixed.
* Fixing broken tests for changes in robot_state.
* Fixed doxygen function-grouping.
* Fix `#95 <https://github.com/ros-planning/moveit_core/issues/95>`_.
* More docs for RobotState.

0.5.4 (2013-10-11)
------------------
* Add functionality for enforcing velocity limits; update API to better naming to cleanly support the new additions
* Adding Travis Continuous Integration to MoveIt
* remember if a group could be a parent of an eef, even if it is not the default one

0.5.3 (2013-09-25)
------------------
* remove use of flat_map

0.5.2 (2013-09-23)
------------------
* Rewrite RobotState and significantly update RobotModel; lots of optimizations
* add support for diffs in RobotState
* fix `#87 <https://github.com/ros-planning/moveit_core/issues/87>`_
* add non-const variants for getRobotMarkers
* use trajectory_msgs::JointTrajectory for object attach information instead of sensor_msgs::JointState
* add effort to robot state
* do not include mimic joints or fixed joints in the set of joints in a robot trajectory
* voxel_grid: finish adding Eigen accessors
* voxel_grid: add Eigen accessors
* eliminate determineCollisionPoints() and distance_field_common.h
* propagation_distance_field: make getNearestCell() work with max_dist cells
* distance_field: fix bug in adding shapes
* propagation_distance_field: add getNearestCell()

0.5.1 (2013-08-13)
------------------
* remove CollisionMap message, allow no link name in for AttachedCollisionObject REMOVE operations
* make headers and author definitions aligned the same way; white space fixes
* move background_processing lib to core
* enable RTTI for CollisionRequest
* added ability to find attached objects for a group
* add function for getting contact pairs

0.5.0 (2013-07-15)
------------------
* move msgs to common_msgs

0.4.7 (2013-07-12)
------------------
* doc updates
* white space fixes (tabs are now spaces)
* update root joint if needed, after doing backward fk
* adding options struct to kinematics base
* expose a planning context in the planning_interface base library

0.4.6 (2013-07-03)
------------------
* Added ability to change planner configurations in the interface
* add docs for controller manager
* fix computeTransformBackward()

0.4.5 (2013-06-26)
------------------
* add computeBackwardTransform()
* bugfixes for voxel_grid, distance_field
* slight improvements to profiler
* Fixes compile failures on OS X with clang
* minor speedup in construction of RobotState
* fix time parametrization crash due to joints that have #variables!=1
* remove re-parenting of URDF models feature (we can do it cleaner in a different way)

0.4.4 (2013-06-03)
------------------
* fixes for hydro
* be careful about when to add a / in front of the frame name

0.4.3 (2013-05-31)
------------------
* remove distinction of loaded and active controllers

0.4.2 (2013-05-29)
------------------
* generate header with version information

0.4.1 (2013-05-27)
------------------
* fix `#66 <https://github.com/ros-planning/moveit_core/issues/66>`_
* rename getTransforms() to copyTransforms()
* refactor how we deal with frames; add a separate library
* remove direction from CollisionResult

0.4.0 (2013-05-23)
------------------
* attempt to fix `#241 <https://github.com/ros-planning/moveit_core/issues/241>`_ from moveit_ros
* update paths so that files are found in the globally installed moveit_resources package
* remove magical 0.2 and use of velocity_map
* Work on issue `#35 <https://github.com/ros-planning/moveit_core/issues/35>`_.

0.3.19 (2013-05-02)
-------------------
* rename getAttachPosture to getDetachPosture
* add support for attachment postures and implement MOVE operation for CollisionObject
* add ability to fill in planning scene messages by component
* when projection from start state fails for IK samplers, try random states
* bugfixes

0.3.18 (2013-04-17)
-------------------
* allow non-const access to kinematic solver
* bugfix: always update variable transform

0.3.17 (2013-04-16)
-------------------
* bugfixes
* add console colors
* add class fwd macro
* cleanup API of trajectory lookup
* Added method to get joint type as string
* fixing the way mimic joints are updated
* fixed tests

0.3.16 (2013-03-15)
-------------------
* bugfixes
* robot_state::getFrameTransform now returns a ref instead of a pointer; fixed a bug in transforming Vector3 with robot_state::Transforms, add planning_scene::getFrameTransform
* add profiler tool (from ompl)

0.3.15 (2013-03-08)
-------------------
* Remove configure from PlanningScene
* return shared_ptr from getObject() (was ref to shared_ptr)
* use NonConst suffix on PlanningScene non-const get functions.
* make setActiveCollisionDetector(string) return bool status
* use CollisionDetectorAllocator in PlanningScene
* add World class
* bodies attached to the same link should not collide
* include velocities in conversions
* Added more general computeCartesianPath, takes vector of waypoints
* efficiency improvements

0.3.14 (2013-02-05)
-------------------
* initialize controller state by default
* fix `#157 <https://github.com/ros-planning/moveit_core/issues/157>`_ in moveit_ros
* fix moveit_ros/`#152 <https://github.com/ros-planning/moveit_core/issues/152>`_

0.3.13 (2013-02-04 23:25)
-------------------------
* add a means to get the names of the known states (as saved in SRDF)
* removed kinematics planner

0.3.12 (2013-02-04 13:16)
-------------------------
* Adding comments to voxel grid
* Adding in octree constructor and some additional fields and tests
* Getting rid of obstacle_voxel set as it just slows things down
* Removing pf_distance stuff, adding some more performance, getting rid of addCollisionMapToField function
* Fixing some bugs for signed distance field and improving tests
* Merging signed functionality into PropagateDistanceField, adding remove capabilities, and adding a few comments and extra tests

0.3.11 (2013-02-02)
-------------------
* rename KinematicState to RobotState, KinematicTrajectory to RobotTrajectory
* remove warnings about deprecated functions, use a deque instead of vector to represent kinematic trajectories

0.3.10 (2013-01-28)
-------------------
* fix `#28 <https://github.com/ros-planning/moveit_core/issues/28>`_
* improves implementation of metaball normal refinement for octomap
* add heuristic to detect jumps in joint-space distance
* make it such that when an end effector is looked up by group name OR end effector name, things work as expected
* removed urdf and srdf from configure function since kinematic model is also passed in
* make sure decoupling of scenes from parents that are themselves diffs to other scenes actually works
* Fix KinematicState::printStateInfo to actually print to the ostream given.
* add option to specify whether the reference frame should be global or not when computing Cartesian paths
* update API for trajectory smoother
* add interpolation function that takes joint velocities into account, generalize setDiffFromIK
* add option to reverse trajectories
* add computeCartesianPath()
* add ability to load & save scene geometry as text
* compute jacobian with kdl
* fix `#15 <https://github.com/ros-planning/moveit_core/issues/15>`_

0.3.9 (2013-01-05)
------------------
* adding logError when kinematics solver not instantiated, also changing @class
* move some functions to a anonymous namespace
* add doc for kinematic_state ns

0.3.8 (2013-01-03)
------------------
* add one more CATKIN dep

0.3.7 (2012-12-31)
------------------
* add capabilities related to reasoning about end-effectors

0.3.6 (2012-12-20)
------------------
* add ability to specify external sampling constraints for constraint samplers

0.3.5 (2012-12-19 01:40)
------------------------
* fix build system

0.3.4 (2012-12-19 01:32)
------------------------
* add notion of default number of IK attempts
* added ability to use IK constraints in sampling with IK samplers
* fixing service request to take proper group name, check for collisions
* make setFromIK() more robust

0.3.3 (2012-12-09)
------------------
* adding capability for constraint aware kinematics + consistency limits to joint state group
* changing the way consistency limits are specified
* speed up implementation of infinityNormDistance()
* adding distance functions and more functions to sample near by
* remove the notion of PlannerCapabilities

0.3.2 (2012-12-04)
------------------
* robustness checks + re-enabe support for octomaps
* adding a bunch of functions to sample near by

0.3.1 (2012-12-03)
------------------
* update debug messages for dealing with attached bodies, rely on the conversion functions more
* changing manipulability calculations
* adding docs
* log error if joint model group not found
* cleaning up code, adding direct access api for better efficiency

0.3.0 (2012-11-30)
------------------
* added a helper function

0.2.12 (2012-11-29)
-------------------
* fixing payload computations
* Changing pr2_arm_kinematics test plugin for new kinematics_base changes
* Finished updating docs, adding tests, and making some small changes to the function of UnionConstraintSampler and ConstraintSamplerManager
* Some extra logic for making sure that a set of joint constraints has coverage for all joints, and some extra tests and docs for constraint sampler manager
* adding ik constraint sampler tests back in, and modifying dependencies such that everything builds
* Changing the behavior of default_constraint_sampler JointConstraintSampler to support detecting conflicting constraints or one constraint that narrows another value, and adding a new struct for holding data.  Also making kinematic_constraint ok with values that are within 2*epsilon of the limits

0.2.11 (2012-11-28)
-------------------
* update kinematics::KinematicBase API and add the option to pass constraints to setFromIK() in KinematicState

0.2.10 (2012-11-25)
-------------------
* minor reorganization of code
* fix `#10 <https://github.com/ros-planning/moveit_core/issues/10>`_

0.2.9 (2012-11-23)
------------------
* minor bugfix

0.2.8 (2012-11-21)
------------------
* removing deprecated functions

0.2.7 (2012-11-19)
------------------
* moving sensor_manager and controller_manager from moveit_ros

0.2.6 (2012-11-16 14:19)
------------------------
* reorder includes
* add group name option to collision checking via planning scene functions

0.2.5 (2012-11-14)
------------------
* update DEPENDS
* robustness checks

0.2.4 (2012-11-12)
------------------
* add setVariableBounds()
* read information about passive joints from srdf

0.2.3 (2012-11-08)
------------------
* using srdf info for `#6 <https://github.com/ros-planning/moveit_core/issues/6>`_
* fix `#6 <https://github.com/ros-planning/moveit_core/issues/6>`_

0.2.2 (2012-11-07)
------------------
* add processPlanningSceneWorldMsg()
* Adding and fixing tests
* Adding docs
* moves refineNormals to new file in collision_detection
* Fixed bugs in PositionConstraint, documented Position and Orientation constraint, extended tests for Position and OrientationConstraint and started working on tests for VisibilityConstraint
* more robust checking of joint names in joint constraints
* adds smoothing to octomap normals; needs better testing

0.2.1 (2012-11-06)
------------------
* revert some of the install location changes

0.2.0 (2012-11-05)
------------------
* update install target locations

0.1.19 (2012-11-02)
-------------------
* add dep on kdl_parser

0.1.18 (2012-11-01)
-------------------
* add kinematics_metrics & dynamics_solver to build process

0.1.17 (2012-10-27 18:48)
-------------------------
* fix DEPENDS libs

0.1.16 (2012-10-27 16:14)
-------------------------
* more robust checking of joint names in joint constraints
* KinematicModel and KinematicState are independent; need to deal with transforms and conversions next

0.1.15 (2012-10-22)
-------------------
* moving all headers under include/moveit/ and using console_bridge instead of rosconsole

0.1.14 (2012-10-20 11:20)
-------------------------
* fix typo

0.1.13 (2012-10-20 10:51)
-------------------------
* removing no longer needed deps
* add ``moveit_`` prefix for all generated libs

0.1.12 (2012-10-18)
-------------------
* porting to new build system
* moved some libraries to moveit_planners
* add access to URDF and SRDF in planning_models
* Adding in path constraints for validating states, needs more testing

0.1.11 (2012-09-20 12:55)
-------------------------
* update conversion functions for kinematic states to support attached bodies

0.1.10 (2012-09-20 10:34)
-------------------------
* making JointConstraints + their samplers work with local variables for multi_dof joints
* Remove fast time parameterization and zero out waypoint times
* setting correct error codes
* bugfixes
* changing the way subgroups are interpreted

0.1.9 (2012-09-14)
------------------
* bugfixes

0.1.8 (2012-09-12 20:56)
------------------------
* bugfixes

0.1.7 (2012-09-12 18:56)
------------------------
* bugfixes

0.1.6 (2012-09-12 18:39)
------------------------
* add install targets, fix some warnings and errors

0.1.5 (2012-09-12 17:25)
------------------------
* first release
