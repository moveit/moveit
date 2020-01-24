^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.18 (2020-01-24)
-------------------

0.9.17 (2019-07-09)
-------------------

0.9.16 (2019-06-29)
-------------------
* [maintanance] Resolve catkin lint issues (`#1137 <https://github.com/ros-planning/moveit/issues/1137>`_)
* [maintanance] Improve clang-format (`#1214 <https://github.com/ros-planning/moveit/issues/1214>`_)
* Contributors: Ludovic Delval, Robert Haschke

0.9.15 (2018-10-29)
-------------------
* [improvement] Exploit the fact that our transforms are isometries (instead of general affine transformations). `#1091 <https://github.com/ros-planning/moveit/issues/1091>`_
* Contributors: Robert Haschke

0.9.14 (2018-10-24)
-------------------

0.9.13 (2018-10-24)
-------------------
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix] DepthImageOctomapUpdater not found error (`#954 <https://github.com/ros-planning/moveit/issues/954>`_)
* [fix] planning scene lock when octomap updates too quickly (`#920 <https://github.com/ros-planning/moveit/issues/920>`_)
* [enhancement] error message in shape_mask (`#828 <https://github.com/ros-planning/moveit/issues/828>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [maintenance] disable unittests for moveit_ros_perception ... due to broken Mesa OpenGL (since version 17.x?) (`#982 <https://github.com/ros-planning/moveit/issues/982>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: Adrian Zwiener, Martin Günther, Michael Görner, Mikael Arguedas, Mohmmad Ayman, Ridhwan Luthra, Robert Haschke, mike lautman

0.9.12 (2018-05-29)
-------------------
* boost::shared_ptr -> std::shared_ptr
* [fix] make OpenGL parts optional (`#698 <https://github.com/ros-planning/moveit/issues/698>`_)
* Contributors: Bence Magyar, Ian McMahon, Lukas Bulwahn, Michael Görner, Mikael Arguedas, Robert Haschke

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [improve] removed deprecated pluginlib macro (`#677 <https://github.com/ros-planning/moveit/issues/677>`_)
* Contributors: Mikael Arguedas

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [fix][moveit_ros_robot_interaction] `catkin_make -DCMAKE_ENABLE_TESTING=0` failure (`#478 <https://github.com/ros-planning/moveit/issues/478>`_)
* Contributors: Michael Goerner

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [maintenance] Remove custom cmake modules (`#418 <https://github.com/ros-planning/moveit/issues/418>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Jochen Sprickerhof

0.9.3 (2016-11-16)
------------------

0.9.2 (2016-11-05)
------------------
* [Maintenace] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.6.6 (2016-06-08)
------------------
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* remove unknown dependency sensor_msgs_generate_cpp
  dependencies are pulled in via ${catkin_LIBRARIES}
* Find X11 for build on OS X 10.11
* set empty display function for glut window
  With freeglut 3.0 moveit aborts over here, printing
  > ERROR: No display callback registered for window 1
  According to https://sourceforge.net/p/freeglut/bugs/229/
  and https://www.opengl.org/resources/libraries/glut/spec3/node46.html
  a callback *must* be registered for each window.
  With this patch moveit starts up as expected.
* Remove OpenMP parallelization, fixes `#563 <https://github.com/ros-planning/moveit_ros/issues/563>`_
* Removed trailing whitespace from entire repository
* last comment
* Added missing dependency on moveit_msgs package
* Contributors: Andriy Petlovanyy, Dave Coleman, Isaac I.Y. Saito, Kentaro Wada, Robert Haschke, Stefan Kohlbrecher, dg, v4hn

0.6.5 (2015-01-24)
------------------
* update maintainers
* adding RAII-based locking for OccMapTree
* moving lazy_free_space_updater into it's own library
* Contributors: Jonathan Bohren, Michael Ferguson

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* port `moveit_ros#445 <https://github.com/ros-planning/moveit_ros/issues/445>`_ to indigo
* disable test that needs display when no display defined
* GL_TYPE() is a function in newer versions of OpenGL, this fixes tests on Ubuntu 14.04
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------
* fix linking error on OSX
* Contributors: Michael Ferguson

0.6.0 (2014-10-27)
------------------
* Fixing invalid iterators if filtered_cloud_topic is not set.
  Adding missing dependency on sensor_msgs.
  Fixing indentation, whitespace, and tabs.
  Incrementing PointCloud2Iterator pixel-at-a-time, not byte-at-a-time.
* remove PCL dependency
* Fixed issue with unordered_map and libc++ (LLVM, Mac OS X Mavericks)
  libc++ doesn't have std::tr1::unordered_map, just std::unordered_map
* Fixing OpenGL gl.h and glu.h inclusion on Mac OS X
* Contributors: Jason Ziglar, Marco Esposito, Sachin Chitta, Vincent Rabaud

0.5.19 (2014-06-23)
-------------------
* Fix [-Wreorder] warning.
* Address [cppcheck: duplicateExpression] error.
  The existing check for NaNs is in fact correct for IEEE-compliant floating
  numbers, i.e., if (a == a) then a is not a NaN, but confuses static code
  analyzers. This fix instead uses the isnan(a) macro from <cmath>.
* Prevent future conflicts between STL and Boost.
  mesh_filter_base.cpp was doing:
  using namespace std;
  using namespace boost;
  Considering that Boost is a testing ground for future standard additions,
  bringing the two namespaces into scope in the same translation unit is not
  the best idea. In this particular file, there's a potential conflict between
  C++'s and Boost's shared_ptr implementation.
* Make creation of std::pairs future-compiler-proof.
  Details:
  http://stackoverflow.com/questions/14623958/breaking-change-in-c11-with-make-pair-ty1-val1-const-ty2-val2
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Contributors: Ioan Sucan

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
* comply to the new Table.msg
* Contributors: Vincent Rabaud

0.5.9 (2013-12-03)
------------------
* fix cloud offset

0.5.8 (2013-10-11)
------------------
* adds compliance for mesa versions <9.2

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------
* fix `#320 <https://github.com/ros-planning/moveit_ros/issues/320>`_.
* fix `#318 <https://github.com/ros-planning/moveit_ros/issues/318>`_.

0.5.5 (2013-09-23)
------------------
* remove dep on pcl (pcl_conversions is sufficient)

0.5.4 (2013-08-14)
------------------
* add dependency on OpenCV2
* Pointcloud_octomap_updater compilation flags fixed

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------
* find PCL separately

0.5.0 (2013-07-12)
------------------
* use pcl_conversions instead of pcl_ros
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* Fixes linkedit error on OS X
