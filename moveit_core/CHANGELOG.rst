^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* add moveit_ prefix for all generated libs

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
