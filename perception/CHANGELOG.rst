^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
