^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.14 (2018-10-20)
-------------------
* [fix][moveit_ros_visualization] Robot model not shown after MSA `#786 <https://github.com/ros-planning/moveit/issues/786>`_
* [fix] race conditions when updating PlanningScene `#350 <https://github.com/ros-planning/moveit/issues/350>`_
* [fix][moveit_core] computation of shape_extents_ of links w/o shapes  `#766 <https://github.com/ros-planning/moveit/issues/766>`_
* [improvement] skip non-actuated joints for trajectory execution (`#753 <https://github.com/ros-planning/moveit/issues/753>`_)
* [capability][revert] Revert "Improved IPTP by fitting a cubic spline (`#382 <https://github.com/ros-planning/moveit/issues/382>`_)"
* Contributors: Dave Coleman, Kei Okada, Martin Pecka, Michael Görner, Robert Haschke, Ryan Keating, Robert Haschke
    
0.7.13 (2017-12-25)
-------------------
* [fix][moveit_ros_planning] Avoid segfault when validating a multidof-only trajectory (`#691 <https://github.com/ros-planning/moveit/issues/691>`_). Fixes `#539 <https://github.com/ros-planning/moveit/issues/539>`_
* [fix][moveit_ros_planning] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* [fix][moveit_ros_visualization] don't crash on empty robot_description in RobotState plugin `#688 <https://github.com/ros-planning/moveit/issues/688>`_
* [fix][moveit_ros_visualization] RobotState rviz previewer: First message from e.g. latching publishers is not applied to robot state correctly (`#596 <https://github.com/ros-planning/moveit/issues/596>`_)
* [fix][moveit_ros_planning_interface] MoveGroupInterface: Fixed computeCartesianPath to use selected end-effector. (`#580 <https://github.com/ros-planning/moveit/issues/580>`_)
* [fix][moveit_ros_move_group] always return true in MoveGroupPlanService callback `#674 <https://github.com/ros-planning/moveit/pull/674>`_
* [fix][moveit_setup_assistant] add moveit_fake_controller_manager to run_depend of moveit_config_pkg_template/package.xml.template (`#613 <https://github.com/ros-planning/moveit/issues/613>`_)
* [fix][moveit_setup_assistant] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* [fix][moveit_kinematics] create_ikfast_moveit_plugin: fixed directory variable for templates that were moved to ikfast_kinematics_plugin `#620 <https://github.com/ros-planning/moveit/issues/620>`_
* [fix][moveit_experimental] remove explicit fcl depends `#632 <https://github.com/ros-planning/moveit/pull/632>`_
* [fix][moveit_core] Add missing logWarn argument (`#707 <https://github.com/ros-planning/moveit/issues/707>`_)
* [fix][moveit_core] IKConstraintSampler: Fixed transform from end-effector to ik chain tip. `#582 <https://github.com/ros-planning/moveit/issues/582>`_
* [fix][moveit_core] robotStateMsgToRobotState: is_diff==true => not empty `#589 <https://github.com/ros-planning/moveit/issues/589>`_
* [fix][moveit_commander] Bugs in moveit_commander/robot.py (`#621 <https://github.com/ros-planning/moveit/issues/621>`_)
* [fix][moveit_commander] pyassimp regression workaround  (`#581 <https://github.com/ros-planning/moveit/issues/581>`_)
* [capability][moveit_ros_planning] Multi DOF Trajectory only providing translation not velocity (`#555 <https://github.com/ros-planning/moveit/issues/555>`_)
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
* [fix][moveit_ros_visualization] TrajectoryVisualization crash if no window_context exists (`#523 <https://github.com/ros-planning/moveit/issues/523>`_, `#525 <https://github.com/ros-planning/moveit/issues/525>`_)
* [fix][moveit_ros_visualization]  robot display: Don't reload robot model upon topic change (Fixes `#528 <https://github.com/ros-planning/moveit/issues/528>`_)
* [fix][moveit_ros_planning] Include callback of execution status if trajectory is invalid. (`#524 <https://github.com/ros-planning/moveit/issues/524>`_)
* [fix][simple_controller_manager] include order (`#529 <https://github.com/ros-planning/moveit/issues/529>`_)
* [enhance][moveit_ros_visualization]  rviz display: stop trajectory visualization on new plan. Fixes `#526 <https://github.com/ros-planning/moveit/issues/526>`_ (`#531 <https://github.com/ros-planning/moveit/issues/531>`_, `#510 <https://github.com/ros-planning/moveit/issues/510>`_).
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
