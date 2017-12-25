^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_assistant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix][kinetic onward] msa: use qt4-compatible API for default font (`#682 <https://github.com/ros-planning/moveit/issues/682>`_)
* [fix][kinetic onward] replace explicit use of Arial with default application font (`#668 <https://github.com/ros-planning/moveit/issues/668>`_)
* [fix] add moveit_fake_controller_manager to run_depend of moveit_config_pkg_template/package.xml.template (`#613 <https://github.com/ros-planning/moveit/issues/613>`_)
* [fix] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* Contributors: Kei Okada, Michael Görner, Mikael Arguedas, William Woodall

0.9.9 (2017-08-06)
------------------
* [setup_assistant] Fix for lunar (`#542 <https://github.com/ros-planning/moveit/issues/542>`_) (fix `#506 <https://github.com/ros-planning/moveit/issues/506>`_)
* Contributors: Dave Coleman

0.9.8 (2017-06-21)
------------------
* [enhance] setup assistant: add use_gui param to demo.launch (`#532 <https://github.com/ros-planning/moveit/issues/532>`_)
* [build] add Qt-moc guards for boost 1.64 compatibility (`#534 <https://github.com/ros-planning/moveit/issues/534>`_)
* Contributors: Michael Goerner

0.9.7 (2017-06-05)
------------------
* [fix] Build for Ubuntu YZ by adding BOOST_MATH_DISABLE_FLOAT128 (`#505 <https://github.com/ros-planning/moveit/issues/505>`_)
* [improve][MSA] Open a directory where setup_assistant.launch was started. (`#509 <https://github.com/ros-planning/moveit/issues/509>`_)
* Contributors: Isaac I.Y. Saito, Mikael Arguedas

0.9.6 (2017-04-12)
------------------
* [improve] Add warning if no IK solvers found (`#485 <https://github.com/ros-planning/moveit/issues/485>`_)
* Contributors: Dave Coleman

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_ 
* Contributors: Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] Qt4/Qt5 compatibility `#413 <https://github.com/ros-planning/moveit/pull/413>`_
* [fix] show disabled collisions as matrix  (`#394 <https://github.com/ros-planning/moveit/issues/394>`_)
* Contributors: Dave Coleman, Robert Haschke, Michael Goerner

0.9.3 (2016-11-16)
------------------
* [capability] Exposed planners from latest ompl release. (`#338 <https://github.com/ros-planning/moveit/issues/338>`_)
* [enhancement] Increase collision checking interval (`#337 <https://github.com/ros-planning/moveit/issues/337>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Ruben Burger

0.9.2 (2016-11-05)
------------------
* [Fix] xacro warnings in Kinetic (`#334 <https://github.com/ros-planning/moveit/issues/334>`_)
  [Capability] Allows for smaller collision objects at the cost of increased planning time
* [Improve] Increase the default discretization of collision checking motions (`#321 <https://github.com/ros-planning/moveit/issues/321>`_)
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.7.1 (2016-06-24)
------------------
* [sys] Qt adjustment. 
  * relax Qt-version requirement.  Minor Qt version updates are ABI-compatible with each other:  https://wiki.qt.io/Qt-Version-Compatibility
  * auto-select Qt version matching the one from rviz `#114 <https://github.com/ros-planning/moveit_setup_assistant/issues/114>`_
  * Allow to conditionally compile against Qt5 by setting -DUseQt5=On
* [sys] Add line for supporting CMake 2.8.11 as required for Indigo
* [sys][travis] Update CI conf for ROS Jade (and optionally added Kinetic) `#116 <https://github.com/ros-planning/moveit_setup_assistant/issues/116>`_
* [feat] add ApplyPlanningScene capability to template
* Contributors: Dave Coleman, Isaac I.Y. Saito, Robert Haschke, Simon Schmeisser (isys vision), v4hn

0.7.0 (2016-01-30)
------------------
* Merge pull request from ipa-mdl/indigo-devel
  Added command-line SRDF updater
* renamed target output to collisions_updater
* formatted code to roscpp style
* More verbose error descriptions, use ROS_ERROR_STREAM
* moved file loader helpers into tools
* added licence header
* Missed a negation sign
* CollisionUpdater class was not really needed
* factored out createFullURDFPath and createFullSRDFPath
* factored out MoveItConfigData::getSetupAssistantYAMLPath
* factored out MoveItConfigData::setPackagePath
* factored out setCollisionLinkPairs into MoveItConfigData
* require output path to be set if SRDF path is overwritten by a xacro file path
* separated xacro parsing from loadFileToString
* make disabled_collisions entries unique
* Added command-line SRDF updater
* Merge pull request from 130s/fix/windowsize
  Shrink window height
* Add scrollbar to the text area that could be squashed.
* Better minimum window size.
* Merge pull request #103  from gavanderhoorn/issue102_cfgrble_db_path
  Fix for issue #102 : allow user to set mongodb db location
* Update warehouse launch file to accept non-standard db location. Fix #102.
  Also update generated demo.launch accordingly.
  The default directory could be located on a non-writable file system, leading
  to crashes of the mongodb wrapper script. This change allows the user to specify
  an alternative location using the 'db_path' argument.
* Update configuration_files_widget.cpp
  Fix link
* Contributors: Dave Coleman, Ioan A Sucan, Isaac IY Saito, Mathias Lüdtke, Nathan Bellowe, Sachin Chitta, gavanderhoorn, hersh

0.6.0 (2014-12-01)
------------------
* Values are now read from kinematics.yaml correctly.
* Simplified the inputKinematicsYAML() code.
* Debug and octomap improvements in launch file templates
* Values are now read from kinematics.yaml correctly. Previously, keys such
  as "kinematics_solver" were not found.
* Added clear octomap service to move_group launch file template
* Added gdb debug helper that allows easier break point addition
* Add launch file for joystick control of MotionPlanningPlugin
* Joint limits comments
* Removed velocity scaling factor
* Added a new 'velocity_scaling_factor' parameter to evenly reduce max joint velocity for all joints. Added documentation.
* Simply renamed kin_model to robot_model for more proper naming convension
* Added new launch file for controll Rviz with joystick
* use relative instead of absolute names for topics (to allow for namespaces)
* Added planner specific parameters to ompl_planning.yaml emitter.
* Added space after every , in function calls
  Added either a space or a c-return before opening {
  Moved & next to the variable in the member function declarations
* Added planner specific parameters to ompl_planning.yaml emitter.
  Each parameter is set to current defaults. This is fragile, as defaults may change.
* Contributors: Chris Lewis, Dave Coleman, Ioan A Sucan, Jim Rothrock, ahb, hersh

0.5.9 (2014-03-22)
------------------
* Fixed bug 82 in a quick way by reducing min size.
* Fix for issue `#70 <https://github.com/ros-planning/moveit_setup_assistant/issues/70>`_: support yaml-cpp 0.5+ (new api).
* Generate joint_limits.yaml using ordered joints
* Ensures that group name changes are reflected in the end effectors and robot poses screens as well
* Prevent dirty transforms warning
* Cleaned up stray cout's
* Contributors: Benjamin Chretien, Dave Coleman, Dave Hershberger, Sachin Chitta

0.5.8 (2014-02-06)
------------------
* Update move_group.launch
  Adding get planning scene service to template launch file.
* Fix `#42 <https://github.com/ros-planning/moveit_setup_assistant/issues/42>` plus cosmetic param name change.
* Contributors: Acorn, Dave Hershberger, sachinchitta

0.5.7 (2014-01-03)
------------------
* Added back-link to tutorial and updated moveit website URL.
* Ported tutorial from wiki to sphinx in source repo.

0.5.6 (2013-12-31)
------------------
* Fix compilation on OS X 10.9 (clang)
* Contributors: Nikolaus Demmel, isucan

0.5.5 (2013-12-03)
------------------
* fix `#64 <https://github.com/ros-planning/moveit_setup_assistant/issues/64>`_.
* Added Travis Continuous Integration

0.5.4 (2013-10-11)
------------------
* Added optional params so user knows they exist - values remain same

0.5.3 (2013-09-23)
------------------
* enable publishing more information for demo.launch
* Added 2 deps needed for some of the launch files generated by the setup assistant
* add source param for joint_state_publisher
* Added default octomap_resolution to prevent warning when move_group starts. Added comments.
* generate config files for fake controllers
* port to new robot state API

0.5.2 (2013-08-16)
------------------
* fix `#50 <https://github.com/ros-planning/moveit_setup_assistant/issues/50>`_
* fix `#52 <https://github.com/ros-planning/moveit_setup_assistant/issues/52>`_

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
