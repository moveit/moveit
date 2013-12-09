^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* correcting maintainer email
* Fixed an occasional crash bug in rviz plugin caused by gui calls in non-gui thread.
* Added planning feedback to gui, refactored states tab
* Stored states are auto loaded when warehouse database is connected

0.5.8 (2013-10-11)
------------------
* Added option to rviz plugin to show scene robot collision geometry

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* Fix crash when the destructor is called before onInitialize
* remove call for getting the combined joint limits of a group
* bugfixes
* porting to new RobotState API
* use new helper class from rviz for rendering meshes

0.5.4 (2013-08-14)
------------------

* Added manipulation tab, added plan id to manipulation request
* make headers and author definitions aligned the same way; white space fixes
* using action client for object recognition instead of topic
* move background_processing lib to core
* display collision pairs instead of simply colliding links

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* fix `#275 <https://github.com/ros-planning/moveit_ros/issues/275>`_
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* remove root_link_name property
* add status tab to Rviz plugin
