^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_industrial_motion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.5 (2021-05-23)
------------------
* Allow selecting planning pipeline in MotionSequenceAction (`#2657 <https://github.com/ros-planning/moveit/issues/2657>`_)
* Contributors: Felix von Drigalski

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------
* Skip fixed joints when checking velocity limits (`#2610 <https://github.com/ros-planning/moveit/issues/2610>`_)
* Contributors: Christian Landgraf

1.1.2 (2021-04-08)
------------------
* Fix formatting errors
* Replaced eigen+kdl conversions with tf2_eigen + tf2_kdl (`#2472 <https://github.com/ros-planning/moveit/issues/2472>`_)
* Add missing dependency on joint_limits_interface (`#2487 <https://github.com/ros-planning/moveit/issues/2487>`_)
* Use kinematics solver timeout if not specified otherwise (`#2489 <https://github.com/ros-planning/moveit/issues/2489>`_)
* pilz planner: add string includes (`#2483 <https://github.com/ros-planning/moveit/issues/2483>`_)
* Upgrade cmake_minimum_required to 3.1 (`#2453 <https://github.com/ros-planning/moveit/issues/2453>`_)
* Add Pilz industrial motion planner (`#1893 <https://github.com/ros-planning/moveit/issues/1893>`_)
* Contributors: Pilz GmbH and Co. KG, Christian Henkel, Immanuel Martini, Michael Görner, Robert Haschke, Tyler Weaver, petkovich
