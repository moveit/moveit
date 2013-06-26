^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_assistant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
