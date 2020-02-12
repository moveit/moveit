^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_trajectory_generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.10 (2019-12-04)
-------------------

0.4.9 (2019-11-28)
------------------

0.4.8 (2019-11-22)
------------------

0.4.7 (2019-09-10)
------------------
* Fix clang-tidy issues
* integrate clang-tidy via CMake flag
* Contributors: Pilz GmbH and Co. KG

0.4.6 (2019-09-04)
------------------

0.4.5 (2019-09-03)
------------------
* Adapt to changes in pilz_robots
* add static code analyzing (clang-tidy)
* drop deprecated isRobotStateEqual()
* Contributors: Pilz GmbH and Co. KG

0.4.4 (2019-06-19)
------------------
* fixed an error that led to trajectories not strictly increasing in time
* Contributors: Pilz GmbH and Co. KG

0.4.3 (2019-04-08)
------------------
* update dependencies of trajectory_generation
* fix CIRC path generator and increase test coverage
* adopt strictest limits in ptp planner (refactor JointLimitsContainer and TrajectoryGeneratorPTP)
* Enable gripper commands inside a sequence
* Contributors: Pilz GmbH and Co. KG

0.4.2 (2019-03-13)
------------------
* re-adapt to new RobotState API: remove #attempts
* Contributors: Pilz GmbH and Co. KG

0.4.1 (2019-02-27)
------------------

0.3.6 (2019-02-26)
------------------
* refactor the testdataloader
* adapt to new RobotState API: remove #attempts
* Contributors: Pilz GmbH and Co. KG

0.3.5 (2019-02-06)
------------------
* Increase line coverage for blending to 100%
* refactor determining the trajectory alignment in the blend implementation
* extend and refactor unittest of blender_transition_window
* add planning group check to blender_transition_window
* add more details to blend algorithm description
* change handling of empty sequences in capabilities to be non-erroneous
* rename command_planner -> pilz_command_planner
* use pilz_testutils package for blend test
* use collision-aware ik calculation
* Contributors: Pilz GmbH and Co. KG

0.4.0 (2018-12-18)
------------------
* Use Eigen::Isometry3d to keep up with the recent changes in moveit
* Contributors: Chris Lalancette

0.3.1 (2018-12-17)
------------------

0.3.0 (2018-11-28)
------------------
* add append method for avoiding duplicate points in robot_trajectory trajectories
* Relax the precondition on trajectory generators from v_start==0 to |v_start| < 1e-10 to gain robustness
* Set last point of generated trajectories to have vel=acc=0 to match the first point.
* add sequence action and service capabilities to concatenate multiple requests
* Contributors: Pilz GmbH and Co. KG

0.2.2 (2018-09-26)
------------------

0.2.1 (2018-09-25)
------------------

0.1.1 (2018-09-25)
------------------
* port to melodic
* drop unused dependencies
* Contributors: Pilz GmbH and Co. KG

0.2.0 (2018-09-14)
------------------
* Changes for melodic
* Contributors: Pilz GmbH and Co. KG

0.1.0 (2018-09-14)
------------------
* Created trajectory generation package with ptp, lin, circ and blend planner
* Contributors: Pilz GmbH and Co. KG
