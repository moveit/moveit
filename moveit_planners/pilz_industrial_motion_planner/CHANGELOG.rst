^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_industrial_motion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.11 (2022-09-13)
-------------------

1.0.10 (2022-03-06)
-------------------

1.0.9 (2022-01-09)
------------------
* Use moveit-resources@master (`#2951 <https://github.com/ros-planning/moveit/issues/2951>`_)

  - Simplify launch files to use the test_environment.launch files from moveit_resources@master
  - Provide compatibility to the Noetic-style configuration of (multiple) planning pipelines
    Only a single pipeline can be used at a time, specified via the ~default_planning_pipeline parameter.
  - Rename launch argument execution_type -> fake_execution_type
* Contributors: Michael Görner, Robert Haschke

1.0.8 (2021-05-23)
------------------
* Fix velocity limit error (`#2610 <https://github.com/ros-planning/moveit/issues/2610>`_)
* Add missing dependency on joint_limits_interface (`#2487 <https://github.com/ros-planning/moveit/issues/2487>`_)
* Use kinematics solver timeout if not specified otherwise (`#2489 <https://github.com/ros-planning/moveit/issues/2489>`_)
* Add pilz_industrial_motion_planner to moveit_planners (`#2507 <https://github.com/ros-planning/moveit/issues/2507>`_)
* Contributors: Christian Henkel, Christian Landgraf, Immanuel Martini, Michael Görner, Robert Haschke, Tyler Weaver
