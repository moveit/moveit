## CHOMP Motion Planner

### Test/run CHOMP

* Clone the repository into your workspace's source directory.

> git clone https://github.com/ros-planning/moveit.git

* cd to the directory you just cloned and checkout the indigo-devel branch.

> cd moveit

> git checkout indigo-devel

* Make the workspace with `catkin_make` and source your workspace's setup file.
* Get the `moveit_resources` package. This package contains test robots and configs to test CHOMP.

> git clone https://github.com/ksatyaki/moveit_resources.git

* Run the demo.

> roslaunch moveit_resources demo_chomp.launch
