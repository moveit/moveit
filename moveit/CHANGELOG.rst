^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
