## CHOMP Motion Planner

### Test/run CHOMP

* Clone the repository into your workspace's source directory.

> git clone https://github.com/ksatyaki/moveit.git

* Make the workspace with `catkin_make` and source your workspace's setup file.
* Get the `moveit_resources` package. This package contains test robots and configs to test CHOMP.

> git clone https://github.com/ksatyaki/moveit_resources.git

* Run the demo.

> roslaunch moveit_resources demo_chomp.launch
