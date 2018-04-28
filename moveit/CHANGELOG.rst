^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

<<<<<<< HEAD
0.9.11 (2017-12-25)
-------------------
* [fix][moveit_core] #723; attached bodies are not shown in trajectory visualization anymore `#724 <https://github.com/ros-planning/moveit/issues/724>`_
* [fix][moveit_core] Shortcomings in kinematics plugins `#714 <https://github.com/ros-planning/moveit/issues/714>`_
* Contributors: Henning Kayser, Michael Görner, Robert Haschke

0.9.10 (2017-12-09)
=======
0.7.13 (2017-12-25)
>>>>>>> upstream/indigo-devel
-------------------
* [fix][moveit_ros_planning] Avoid segfault when validating a multidof-only trajectory (`#691 <https://github.com/ros-planning/moveit/issues/691>`_). Fixes `#539 <https://github.com/ros-planning/moveit/issues/539>`_
* [fix][moveit_ros_planning] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* [fix][moveit_ros_visualization] don't crash on empty robot_description in RobotState plugin `#688 <https://github.com/ros-planning/moveit/issues/688>`_
* [fix][moveit_ros_visualization] RobotState rviz previewer: First message from e.g. latching publishers is not applied to robot state correctly (`#596 <https://github.com/ros-planning/moveit/issues/596>`_)
* [fix][moveit_ros_planning_interface] MoveGroupInterface: Fixed computeCartesianPath to use selected end-effector. (`#580 <https://github.com/ros-planning/moveit/issues/580>`_)
* [fix][moveit_ros_move_group] always return true in MoveGroupPlanService callback `#674 <https://github.com/ros-planning/moveit/pull/674>`_
<<<<<<< HEAD
* [fix][moveit_ros_benchmarks] benchmarks: always prefer local header over system installations `#630 <https://github.com/ros-planning/moveit/issues/630>`_
* [fix][moveit_setup_assistant][kinetic onward] msa: use qt4-compatible API for default font (`#682 <https://github.com/ros-planning/moveit/issues/682>`_)
* [fix][moveit_setup_assistant][kinetic onward] replace explicit use of Arial with default application font (`#668 <https://github.com/ros-planning/moveit/issues/668>`_)
* [fix][moveit_setup_assistant] add moveit_fake_controller_manager to run_depend of moveit_config_pkg_template/package.xml.template (`#613 <https://github.com/ros-planning/moveit/issues/613>`_)
* [fix][moveit_setup_assistant] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* [fix][moveit_kinematics][kinetic onward] Fix create_ikfast_moveit_plugin to comply with format 2 of the package.xml. Remove collada_urdf dependency `#666 <https://github.com/ros-planning/moveit/pull/666>`_
=======
* [fix][moveit_setup_assistant] add moveit_fake_controller_manager to run_depend of moveit_config_pkg_template/package.xml.template (`#613 <https://github.com/ros-planning/moveit/issues/613>`_)
* [fix][moveit_setup_assistant] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
>>>>>>> upstream/indigo-devel
* [fix][moveit_kinematics] create_ikfast_moveit_plugin: fixed directory variable for templates that were moved to ikfast_kinematics_plugin `#620 <https://github.com/ros-planning/moveit/issues/620>`_
* [fix][moveit_experimental] remove explicit fcl depends `#632 <https://github.com/ros-planning/moveit/pull/632>`_
* [fix][moveit_core] Add missing logWarn argument (`#707 <https://github.com/ros-planning/moveit/issues/707>`_)
* [fix][moveit_core] IKConstraintSampler: Fixed transform from end-effector to ik chain tip. `#582 <https://github.com/ros-planning/moveit/issues/582>`_
* [fix][moveit_core] robotStateMsgToRobotState: is_diff==true => not empty `#589 <https://github.com/ros-planning/moveit/issues/589>`_
* [fix][moveit_commander] Bugs in moveit_commander/robot.py (`#621 <https://github.com/ros-planning/moveit/issues/621>`_)
* [fix][moveit_commander] pyassimp regression workaround  (`#581 <https://github.com/ros-planning/moveit/issues/581>`_)
* [capability][moveit_ros_planning] Multi DOF Trajectory only providing translation not velocity (`#555 <https://github.com/ros-planning/moveit/issues/555>`_)
<<<<<<< HEAD
* [capability][moveit_ros_planning_interface][kinetic onward] Adapt pick pipeline to function without object (`#599 <https://github.com/ros-planning/moveit/issues/599>`_)
* [capability][moveit_simple_controller_manager][kinetic onward] optionally wait for controllers indefinitely (`#695 <https://github.com/ros-planning/moveit/issues/695>`_)
=======
>>>>>>> upstream/indigo-devel
* [capability] Multi DOF Trajectory only providing translation not velocity (`#555 <https://github.com/ros-planning/moveit/issues/555>`_)
* [capability] Adds parameter lookup function for kinematics plugins (`#701 <https://github.com/ros-planning/moveit/issues/701>`_)
* [improve][moveit_ros_planning_interface] Disabled copy constructors and added a move constructor to MoveGroupInterface (`#664 <https://github.com/ros-planning/moveit/issues/664>`_)
* [improve][moveit_ros_perception] removed deprecated pluginlib macro (`#677 <https://github.com/ros-planning/moveit/issues/677>`_)
* [improve][moveit_ros_move_group] adding swp's to gitignore and removing redundant capabilites from capability_names.h (`#704 <https://github.com/ros-planning/moveit/issues/704>`_)
* [improve][moveit_kinematics] IKFastTemplate: Expand solutions to full joint range in searchPositionIK `#598 <https://github.com/ros-planning/moveit/issues/598>`_
* [improve][moveit_kinematics] IKFastTemplate: searchPositionIK now returns collision-free solution which is nearest to seed state. (`#585 <https://github.com/ros-planning/moveit/issues/585>`_)
* [improve][moveit_core] Make operator bool() explicit `#696 <https://github.com/ros-planning/moveit/pull/696>`_
* [improve][moveit_core] Get msgs from Planning Scene `#663 <https://github.com/ros-planning/moveit/issues/663>`_
* [improve][moveit_core] moveit_core: export DEPENDS on LIBFCL `#632 https://github.com/ros-planning/moveit/pull/632>`_
* [improve][moveit_core] RobotState: Changed multi-waypoint version of computeCartesianPath to test joint space jumps after all waypoints are generated. (`#576 <https://github.com/ros-planning/moveit/issues/576>`_)
* [improve][moveit_core] Better debug output for IK tip frames (`#603 <https://github.com/ros-planning/moveit/issues/603>`_)
* [improve][moveit_core] New debug console colors YELLOW PURPLE (`#604 <https://github.com/ros-planning/moveit/issues/604>`_)
<<<<<<< HEAD
* [maintenance][moveit_planners_ompl][kinetic onward] Remove OutputHandlerROS from ompl_interface (`#609 <https://github.com/ros-planning/moveit/issues/609>`_)
* [doc][moveit_ros_visualization] Document auto scale in Rviz plugin (`#602 <https://github.com/ros-planning/moveit/issues/602>`_)
* Contributors: axelschroth, 2scholz, Bence Magyar, Bruno Brito, Dave Coleman, Dennis Hartmann, fsuarez6, G.A. vd. Hoorn, Henning Kayser, Isaac I.Y. Saito, Jonathan Meyer, Jorge Nicho, Kei Okada, Konstantin Selyunin, Michael Goerner, Mikael Arguedas, Mike Lautman, Phil, Shingo Kitagawa, Simon Schmeisser, Simon Schmeisser, Sarah Elliott, Shingo Kitagawa, Troy Cordie, William Woodall

0.9.9 (2017-08-06)
------------------
* Fixation in the contained packages:

  * [fix][moveit_ros_planning] Change getCurrentExpectedTrajectory index so collision detection is still performed even if the path timing is not known (`#550 <https://github.com/ros-planning/moveit/issues/550>`_)
  * [fix][moveit_ros_planning] check plan size for plan length=0 `#535 <https://github.com/ros-planning/moveit/issues/535>`_
  * [fix][moveit_ros_planning] ros_error macro name (`#544 <https://github.com/ros-planning/moveit/issues/544>`_)
  * [fix][moveit_ros_visualization] RobotStateVisualization: clear before load to avoid segfault `#572 <https://github.com/ros-planning/moveit/pull/572>`_
  * [fix][setup_assistant] Fix for lunar (`#542 <https://github.com/ros-planning/moveit/issues/542>`_) (fix `#506 <https://github.com/ros-planning/moveit/issues/506>`_)
  * [fix][moveit_core] segfault due to missing string format parameter. (`#547 <https://github.com/ros-planning/moveit/issues/547>`_)
  * [fix][moveit_core] doc-comment for robot_state::computeAABB (`#516 <https://github.com/ros-planning/moveit/issues/516>`_) 
* Improvement in the contained packages:

  * [improve][moveit_ros_planning] Chomp use PlanningScene (`#546 <https://github.com/ros-planning/moveit/issues/546>`_) to partially address `#305 <https://github.com/ros-planning/moveit/issues/305>`_
  * [improve][moveit_ros_control_interface] add backward compatibility patch for indigo (`#551 <https://github.com/ros-planning/moveit/issues/551>`_)
  * [improve][moveit_planners_ompl] Optional forced use of JointModelStateSpaceFactory (`#541 <https://github.com/ros-planning/moveit/issues/541>`_)
  * [improve][moveit_kinematics] Modify ikfast_template for getPositionIK single solution results (`#537 <https://github.com/ros-planning/moveit/issues/537>`_)
* Contributors: Cyrille Morin, henhenhen, Martin Pecka, Simon Schmeisser, Michael Goerner, Mikael Arguedas, nsnitish

0.9.8 (2017-06-21)
------------------
=======
* [doc][moveit_ros_visualization] Document auto scale in Rviz plugin (`#602 <https://github.com/ros-planning/moveit/issues/602>`_)
* Contributors: axelschroth, 2scholz, Bence Magyar, Bruno Brito, Dave Coleman, Dennis Hartmann, fsuarez6, G.A. vd. Hoorn, Henning Kayser, Isaac I.Y. Saito, Jonathan Meyer, Jorge Nicho, Kei Okada, Konstantin Selyunin, Michael Goerner, Mikael Arguedas, Mike Lautman, Phil, Shingo Kitagawa, Simon Schmeisser, Simon Schmeisser, Sarah Elliott, Shingo Kitagawa, Troy Cordie, William Woodall

0.7.12 (2017-08-06)
-------------------
* [fix][moveit_ros/planning] Support for MultiDoF only trajectories `#553 <https://github.com/ros-planning/moveit/pull/553>`_
* [fix][moveit_core] segfault due to missing string format parameter. (`#547 <https://github.com/ros-planning/moveit/issues/547>`_)
* [fix][moveit_core] doc-comment for robot_state::computeAABB (`#516 <https://github.com/ros-planning/moveit/issues/516>`_)
* [fix][moveit_commander] numpy.ndarray indices bug (`#563 <https://github.com/ros-planning/moveit/issues/563>`_, from `#86 <https://github.com/ros-planning/moveit/issues/86>`_, `#450 <https://github.com/ros-planning/moveit/issues/450>`_)
* [fix][moveit_ros_visualization] RobotStateVisualization: clear before load to avoid segfault `#572 <https://github.com/ros-planning/moveit/pull/572>`_
* [enhancement][moveit_commander][moveit_ros][moveit_planners] Optional forced use of JointModelStateSpaceFactory (`#541 <https://github.com/ros-planning/moveit/issues/541>`_)
* [enhancement][moveit_setup_assistant] support loading xacros that use Jade+ extensions on Indigo `#540 <https://github.com/ros-planning/moveit/issues/540>`_
* Contributors: Christopher Schindlbeck, G.A. vd. Hoorn, Cyrille Morin, Martin Pecka, gavanderhoorn, henhenhen, v4hn

0.7.11 (2017-06-21)
-------------------
>>>>>>> upstream/indigo-devel
* [fix][moveit_ros_visualization] TrajectoryVisualization crash if no window_context exists (`#523 <https://github.com/ros-planning/moveit/issues/523>`_, `#525 <https://github.com/ros-planning/moveit/issues/525>`_)
* [fix][moveit_ros_visualization]  robot display: Don't reload robot model upon topic change (Fixes `#528 <https://github.com/ros-planning/moveit/issues/528>`_)
* [fix][moveit_ros_planning] Include callback of execution status if trajectory is invalid. (`#524 <https://github.com/ros-planning/moveit/issues/524>`_)
* [fix][simple_controller_manager] include order (`#529 <https://github.com/ros-planning/moveit/issues/529>`_)
* [enhance][moveit_ros_visualization]  rviz display: stop trajectory visualization on new plan. Fixes `#526 <https://github.com/ros-planning/moveit/issues/526>`_ (`#531 <https://github.com/ros-planning/moveit/issues/531>`_, `#510 <https://github.com/ros-planning/moveit/issues/510>`_).
<<<<<<< HEAD
* [enhance][moveit_setup_assistant] setup assistant: add use_gui param to demo.launch (`#532 <https://github.com/ros-planning/moveit/issues/532>`_)
* [build][moveit_kinematics] adjust cmake_minimum_required for add_compile_options (`#521 <https://github.com/ros-planning/moveit/issues/521>`_)
* [build][moveit_kinematics] ikfast_kinematics_plugin: Add c++11 compile option. This is required for Kinetic.
* [build][moveit_kinematics] ikfast_kinematics_plugin: Write XML files as UTF-8 (`#514 <https://github.com/ros-planning/moveit/issues/514>`_)
* [build][moveit_ros_visualization] add Qt-moc guards for boost 1.64 compatibility (`#534 <https://github.com/ros-planning/moveit/issues/534>`_)
* Contributors: dougsm, Martin Guenther, Michael Goerner, Isaac I.Y. Saito, Simon Schmeisser, Yannick Jonetzko, henhenhen

0.9.7 (2017-06-05)
------------------
* [fix][ikfast_kinematics_plugin][Kinetic+] Add c++11 compile option `#515 <https://github.com/ros-planning/moveit/pull/515>`_
* [fix][moveit_kinematics][Indigo] Eigen3 dependency (`#470 <https://github.com/ros-planning/moveit/issues/470>`_)
* [fix][moveit_ros] Build for Ubuntu YZ by adding BOOST_MATH_DISABLE_FLOAT128 (`#505 <https://github.com/ros-planning/moveit/issues/505>`_)
* [fix][moveit_core] checks for empty name arrays messages before parsing the robot state message data (`#499 <https://github.com/ros-planning/moveit/issues/499>`_)
* [capability][visualization] New panel with a slider to control the visualized trajectory (`#491 <https://github.com/ros-planning/moveit/issues/491>`_) (`#508 <https://github.com/ros-planning/moveit/issues/508>`_)
* [improve][MSA] Open a directory where setup_assistant.launch was started. (`#509 <https://github.com/ros-planning/moveit/issues/509>`_)
* Contributors: Jorge Nicho, Michael Goerner, Martin Guenther, YuehChuan, Dave Coleman, Isaac I.Y. Saito, Mikael Arguedas

0.9.6 (2017-04-12)
------------------
* [fix] warehouse services (`#474 <https://github.com/ros-planning/moveit/issues/474>`_)
* [fix][moveit_ros_visualization] RViz plugin some cosmetics and minor refactoring `#482 <https://github.com/ros-planning/moveit/issues/482>`_
* [fix][moveit_ros_visualization] rviz panel: Don't add object marker if the wrong tab is selected `#454 <https://github.com/ros-planning/moveit/pull/454>`_
* [fix][moveit_ros_robot_interaction] `catkin_make -DCMAKE_ENABLE_TESTING=0` failure (`#478 <https://github.com/ros-planning/moveit/issues/478>`_)
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* [fix][moveit_ros_manipulation] Set planning frame correctly in evaluation of reachable and valid pose filter (`#476 <https://github.com/ros-planning/moveit/issues/476>`_)
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* [fix] undefined symbol in planning_scene_monitor (`#463 <https://github.com/ros-planning/moveit/issues/463>`_)
* [fix][moveit_planners_ompl] Always update initial robot state to prevent dirty robot state error. `#448 <https://github.com/ros-planning/moveit/pull/448>`_
* [fix][moveit_core] PlanarJointModel::getVariableRandomPositionsNearBy (`#464 <https://github.com/ros-planning/moveit/issues/464>`_)
* [improve][moveit_ros_visualization] RobotState display [kinetic] (`#465 <https://github.com/ros-planning/moveit/issues/465>`_)
* [improve][moveit_ros_planning_interface] MoveGroupInterface: add public interface to construct the MotionPlanRequest (`#461 <https://github.com/ros-planning/moveit/issues/461>`_)
* [improve][moveit_ros_benchmarks] Add install rule for examples, statistics script
* [improve] Add warning if no IK solvers found (`#485 <https://github.com/ros-planning/moveit/issues/485>`_)
* Contributors: Ruben Burger, Dave Coleman, Yannick Jonetzko, Henning Kayser, Beatriz Leon, Bence Magyar, Jorge Nicho, Tamaki Nishino, Michael Goerner, Dmitry Rozhkov, Isaac I.Y. Saito


0.9.5 (2017-03-08)
------------------
* [fix] correct "simplify widget handling" `#452 <https://github.com/ros-planning/moveit/pull/452>`_ This reverts "simplify widget handling (`#442 <https://github.com/ros-planning/moveit/issues/442>`_)" 
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_ 
* [fix] Regression on Ubuntu Xenial; numpy.ndarray indices bug (from `#86 <https://github.com/ros-planning/moveit/issues/86>`_) (`#450 <https://github.com/ros-planning/moveit/issues/450>`_).
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* [enhancement][MoveGroup] Add getLinkNames function (`#440 <https://github.com/ros-planning/moveit/issues/440>`_)
* [doc][moveit_commander] added description for set_start_state (`#447 <https://github.com/ros-planning/moveit/issues/447>`_) 
* Contributors: Adam Allevato, Dave Coleman, Bence Magyar, Dave Coleman, Isaac I.Y. Saito, Yannick Jonetzko, Ravi Prakash Joshi

0.9.4 (2017-02-06)
------------------

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------

0.9.0 (2016-10-19)
------------------
* Initial release into ROS Kinetic
=======
* [build][moveit_kinematics] adjust cmake_minimum_required for add_compile_options (`#521 <https://github.com/ros-planning/moveit/issues/521>`_)
* [build][moveit_kinematics] ikfast_kinematics_plugin: Add c++11 compile option. This is required for Kinetic.
* Contributors: dougsm, Martin Guenther, Michael Goerner, Isaac I.Y. Saito, Simon Schmeisser, Yannick Jonetzko, henhenhen

0.7.10 (2017-06-07)
-------------------
* [fix][moveit_core] checks for empty name arrays messages before parsing the robot state message data (`#499 <https://github.com/ros-planning/moveit/issues/499>`_) (`#518 <https://github.com/ros-planning/moveit/issues/518>`_)
* [fix] moveit rviz panel name `#482 <https://github.com/ros-planning/moveit/pull/482>`_
* [fix] Build for Ubuntu YZ by adding BOOST_MATH_DISABLE_FLOAT128 (`#505 <https://github.com/ros-planning/moveit/issues/505>`_)
* [fix][moveit_ros/visualization] Tentative encoding workaround (https://github.com/ros-infrastructure/catkin_pkg/issues/181).
* [capability][vizualization] New panel with a slider to control the visualized trajectory (`#491 <https://github.com/ros-planning/moveit/issues/491>`_) (`#508 <https://github.com/ros-planning/moveit/issues/508>`_)
* Contributors: Dave Coleman, Mikael Arguedas, Jorge Nicho, Isaac I.Y. Saito, Yannick Jonetzko

0.7.9 (2017-04-03)
------------------
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* [fix][moveit_ros_planning] undefined symbol in planning_scene_monitor (`#463 <https://github.com/ros-planning/moveit/issues/463>`_)
* [fix][moveit_ros_manipulation] Set planning frame correctly in evaluation of reachable and valid pose filter (`#476 <https://github.com/ros-planning/moveit/issues/476>`_)
* [fix][moveit_planners_ompl] Always update initial robot state to prevent dirty robot state error. `#448 <https://github.com/ros-planning/moveit/pull/448>`_
* [fix][moveit_core] PlanarJointModel::getVariableRandomPositionsNearBy (`#464 <https://github.com/ros-planning/moveit/issues/464>`_)
* [fix][moveit_ros_visualization] rviz panel: Don't add object marker if the wrong tab is selected `#454 <https://github.com/ros-planning/moveit/pull/454>`_
* Contributors: Yannick Jonetzko, Henning Kayser, Tamaki Nishino, Dmitry Rozhkov, Ruben Burger, Michael Goerner

0.7.8 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [fix] correct "simplify widget handling" `#452 <https://github.com/ros-planning/moveit/pull/452>`_ This reverts "simplify widget handling (`#442 <https://github.com/ros-planning/moveit/issues/442>`_)"
* [fix][moveit_ros_planning] Remove unnecessary dependency on Qt4
* [enhancement][MoveGroup] Add getLinkNames function (`#440 <https://github.com/ros-planning/moveit/issues/440>`_)
* [enhancement] Add set_max_acceleration_scaling_factor to moveit_commander. `#377 <https://github.com/ros-planning/moveit/issues/377>`_, `#437 <https://github.com/ros-planning/moveit/issues/437>`_, `#451 <https://github.com/ros-planning/moveit/issues/451>`_
* [doc][moveit_commander] added description for set_start_state (`#447 <https://github.com/ros-planning/moveit/issues/447>`_)
* Contributors: Dmitry Rozhkov, Yannick Jonetzko, Isaac I.Y. Saito, henhenhen, Ravi Prakash Joshi

0.7.7 (2017-02-06)
------------------

0.7.6 (2016-12-30)
------------------
* [fix][Indigo] re-enable support for cmake 2.8.11 `#391 <https://github.com/ros-planning/moveit/pull/391>`_
* Contributors: Michael Goerner

0.7.5 (2016-12-25)
------------------

0.7.4 (2016-12-22)
------------------
* [indigo][changelog] Remove wrong version entries (see https://github.com/ros-planning/moveit/issues/386#issuecomment-268689110).
* Contributors: Isaac I.Y. Saito

0.7.3 (2016-12-20)
------------------
* [ROS Indigo] Initial release from `ros-planning/moveit <https://github.com/ros-planning/moveit>`_ repository.
* Contributors: Isaac I.Y. Saito
>>>>>>> upstream/indigo-devel
