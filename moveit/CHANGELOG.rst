^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
