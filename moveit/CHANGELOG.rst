^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
