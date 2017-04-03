^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.7 (2017-04-03)
------------------
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* [fix] `catkin_make -DCMAKE_ENABLE_TESTING=0` failure (`#478 <https://github.com/ros-planning/moveit/issues/478>`_)
* [fix][moveit_ros_visualization] rviz panel: Don't add object marker if the wrong tab is selected `#454 <https://github.com/ros-planning/moveit/pull/454>`_
* [fix][moveit_ros_visualization] robot state display: subscribe on enable / unsubscribe on disable (`#455 <https://github.com/ros-planning/moveit/issues/455>`_)
* [fix][moveit_ros_planning] undefined symbol in planning_scene_monitor (`#463 <https://github.com/ros-planning/moveit/issues/463>`_)
* [fix][moveit_ros_manipulation] Set planning frame correctly in evaluation of reachable and valid pose filter (`#476 <https://github.com/ros-planning/moveit/issues/476>`_)
* [fix][moveit_planners_ompl] Always update initial robot state to prevent dirty robot state error. `#448 <https://github.com/ros-planning/moveit/pull/448>`_
* [fix][moveit_core] PlanarJointModel::getVariableRandomPositionsNearBy (`#464 <https://github.com/ros-planning/moveit/issues/464>`_)
* Contributors: Ruben Burger, Dave Coleman, Michael Goerner, Henning Kayser, Tamaki Nishino, Dmitry Rozhkov, Yannick Jonetzko

0.8.6 (2017-03-08)
------------------
* [fix][moveit_ros_visualization] correct "simplify widget handling" `#452 <https://github.com/ros-planning/moveit/pull/452>`_
* [enhancement][MoveGroup] Add getLinkNames function (`#440 <https://github.com/ros-planning/moveit/issues/440>`_)
* [doc][moveit_commander] added description for set_start_state (`#447 <https://github.com/ros-planning/moveit/issues/447>`_)
* Contributors: Yannick Jonetzko, henhenhen, Ravi Prakash Joshi

0.8.4 (2017-02-06)
------------------

0.8.3 (2016-08-19)
------------------
* [Jade] Unify package version numbers (see https://github.com/davetcoleman/moveit_merge/issues/9). (`#79 <https://github.com/ros-planning/moveit/issues/79>`_)
* Add meta package moveit.
* Contributors: Isaac I.Y. Saito
