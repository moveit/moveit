^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_assistant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2013-08-13)
------------------
* make headers and author definitions aligned the same way; white space fixes
* add debug flag to demo.launch template
* default scene alpha is now 1.0
* add robot_state_publisher dependency for generated pkgs
* disable mongodb creation by default in demo.launch
* add dependency on joint_state_publisher for generated config pkgs

0.5.0 (2013-07-15)
------------------
* white space fixes (tabs are now spaces)
* fix `#49 <https://github.com/ros-planning/moveit_setup_assistant/issues/49>`_

0.4.1 (2013-06-26)
------------------
* fix `#44 <https://github.com/ros-planning/moveit_setup_assistant/issues/44>`_
* detect when xacro needs to be run and generate planning_context.launch accordingly
* fix `#46 <https://github.com/ros-planning/moveit_setup_assistant/issues/46>`_
* refactor how planners are added to ompl_planning.yaml; include PRM & PRMstar, remove LazyRRT
* change defaults per `#47 <https://github.com/ros-planning/moveit_setup_assistant/issues/47>`_
* SRDFWriter: add initModel() method for initializing from an existing urdf/srdf model in memory.
* SRDFWriter: add INCLUDE_DIRS to catkin_package command so srdf_writer.h can be used by other packages.
* git add option for minimum fraction of 'sometimes in collision'
* fix `#41 <https://github.com/ros-planning/moveit_setup_assistant/issues/41>`_
