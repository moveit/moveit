# MoveIt! Core

This repository includes core libraries used by MoveIt:
 - representation of kinematic models
 - collision detection interfaces and implementations
 - interfaces for kinematic solver plugins
 - interfaces for motion planning plugins
 - interfaces for controllers and sensors

These libraries do not depend on ROS (except ROS messages) and can be used independently.

## Using clang-tidy.

Install clang-tidy and other clang related tools with
`sudo apt-get install clang libclang-dev clang-tidy clang-format`

To add it to a new package, add `set(CMAKE_EXPORT_COMPILE_COMMANDS ON)` to `CMakeLists.txt` and rebuild.
You can also make a specific clang-tidy build with
```
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build
```

Run it on a specific folder, such as `collision_detection`, with
```
run-clang-tidy-3.8.py -fix -p=/home/brycew/Desktop/moveit_ws/build/moveit_core/  collision_detection
```

