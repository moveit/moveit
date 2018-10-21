^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.14 (2018-10-20)
-------------------

0.7.13 (2017-12-25)
-------------------

0.7.12 (2017-08-06)
-------------------
* Support for MultiDoF only trajectories
* RobotStateVisualization: clear before load to avoid segfault
  rviz::Robot deletes its complete SceneNode structure in the `load()` method.
  However, `RobotStateVisualization::render_shapes\_` keeps raw pointers
  to some of these nodes (the attached objects), so these should be cleared
  to avoid broken pointers.
  Additionally the order of clearing was bad: the attached objects should
  be removed first, and `rviz::Robot` only afterwards to avoid similar problems.
* fixed numpy.ndarray indices bug (from `#86 <https://github.com/ros-planning/moveit/issues/86>`_) (`#450 <https://github.com/ros-planning/moveit/issues/450>`_) (`#563 <https://github.com/ros-planning/moveit/issues/563>`_)
* [moveit_planners] Optional forced use of JointModelStateSpaceFactory (`#541 <https://github.com/ros-planning/moveit/issues/541>`_)
  * Implements optional ompl_planning config parameter 'force_joint_model_state_space'.
  * Renames parameter to 'enforce_joint_model_state_space'.
  Expands workaround comment.
* [moveit_core][moveit_planners] Fixing segfault due to missing string format parameter. (`#547 <https://github.com/ros-planning/moveit/issues/547>`_)
* [moveit_core][moveit_planners] Merge pull request `#540 <https://github.com/ros-planning/moveit/issues/540>`_ from gavanderhoorn/msa_add_jade_xacro_enable_chkbox
  MSA: support loading xacros that use Jade+ extensions on Indigo
* [moveit_core][moveit_planners] setup_assistant: explain purpose of the 'Jade+ xacro' checkbox with tooltip.
* [moveit_core][moveit_planners] setup_assistant: only persist 'jade xacro' key if it's been set.
* [moveit_core][moveit_planners] setup_assistant: persist whether Jade+ xacro is needed when loading existing config.
* [moveit_core][moveit_planners] setup_assistant: support enabling Jade+ xacro in wizard.
  This allows loading xacros that make use of the extensions to xacro that were
  added in Jade and newer ROS releases on Indigo.
* [moveit_core][moveit_planners] Fixed doc-comment for robot_state::computeAABB (`#516 <https://github.com/ros-planning/moveit/issues/516>`_)
  The docstring says the format of the vector is `(minx, miny, minz, maxx, maxy, maxz)`, but according to both the method's implementation and use in moveit, the format is rather `(minx, maxx, miny, maxy, minz, maxz)`.
* [moveit_core][moveit_planners] setup assistant: add use_gui param to demo.launch
  It bugged me for quite some time now that one has to edit the launch file
  just to be able to move the "real" robot around in demo mode.
  Thus I want to expose the option as a parameter in the launch file.
* Contributors: Christopher Schindlbeck, G.A. vd. Hoorn, Martin Pecka, Notou, gavanderhoorn, henhenhen, v4hn

0.7.11 (2017-06-21)
-------------------

0.7.10 (2017-06-07)
-------------------

0.7.9 (2017-04-03)
------------------

0.7.8 (2017-03-08)
------------------

0.7.7 (2017-02-06)
------------------

0.7.6 (2016-12-30)
------------------

0.7.5 (2016-12-25)
------------------

0.7.4 (2016-12-22)
------------------

0.7.3 (2016-12-20)
------------------

0.7.2 (2016-06-20)
------------------

0.7.1 (2016-04-11)
------------------
* [feat] Adding acceleration scaling factor
* [fix] widget naming issues
* [fix] conflict issues
* [fix] Remove OpenMP parallelization (fixes `#563 <https://github.com/ros-planning/moveit_ros/issues/563>`_)
* [sys] explicitly link rviz' default_plugin library. The library is not exported anymore and now is provided separately from rviz_LIBRARIES. See https://github.com/ros-visualization/rviz/pull/979 for details.
* [doc] [move_group.cpp] Print the name of the move group action server that failed to be connected (`#640 <https://github.com/ros-planning/moveit_ros/issues/640>`_)
* Contributors: Stefan Kohlbrecher, v4hn, Dave Coleman, Isaac I.Y. Saito, hemes

0.7.0 (2016-01-30)
------------------
* Removed trailing whitespace from entire repository
* Contributors: Dave Coleman

0.6.5 (2015-01-24)
------------------
* update maintainers
* Contributors: Michael Ferguson

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------

0.5.19 (2014-06-23)
-------------------

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------

0.5.16 (2014-02-27)
-------------------

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* update email addresses
* correcting maintainer email

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------

0.5.4 (2013-08-14)
------------------

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
