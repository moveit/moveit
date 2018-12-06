# MoveIt! ROS Warehouse

This package provides methods to access the MoveIt! warehouse. 


## Export to text

An executable is available to allow to export the scene and queries that are available in the Moveit! warehouse to text files:

```
rosrun moveit_ros_warehouse moveit_warehouse_save_as_text
```

It has different parameters that can be specified:

* `help`: Show help message
* `host`: Host for the DB. Default 127.0.0.1.
* `port`: Port for the DB. Default 33829.
* `output_directory`: Directory to save the generated files
* `scene`: Saves the scenes available in the warehouse. It generates .scene files. If it is not specified the queries are saved.
* `cartesian`: Save queries in cartesian space (start and end pose of eef). If not specified they will be saved in joint space by default.
* `eef`: Specify the end effector (Only needed when the cartesian option is set). Default: last link. 
* `group_prefix`: Specify the group prefix you'd like to plan with. This is useful if you want to save the queries only for the arm and it has a prefix, so only those joints will be considered (e.g. "ra")

To be able to use this command, the robot should be launched in another terminal first.

Here are a few examples of how to use them:
* Example to export the scenes:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_save_as_text --scene --output_directory /tmp/scene
  ```
  Example of the exported scene (empty space with ground): example_scene.scene
  ```
  example_scene
  * ground__link
  1
  box
  5 5 0.01
  -0.2 0.6 0
  0 0 0 1
  0 0 0 0
  .
  ```
* Example to export the queries in joint space:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_save_as_text --output_directory /tmp/queries --group_prefix ra
  ```
  Example of exporting a query: example_scene.queries
  ```
  example_scene
  example_query
  START
  ra_shoulder_pan_joint = -0.000150601
  ra_shoulder_lift_joint = -0.992381
  ra_elbow_joint = 1.73153
  ra_wrist_1_joint = 2.46467
  ra_wrist_2_joint = -0.000163041
  ra_wrist_3_joint = 3.07979
  .
  GOAL
  joint_constraint
  ra_shoulder_pan_joint = 2.20941
  ra_shoulder_lift_joint = -1.89185
  ra_elbow_joint = 1.81825
  ra_wrist_1_joint = 0.14342
  ra_wrist_2_joint = 2.20853
  ra_wrist_3_joint = 0.0420439
  .
  ```
* Example to export the queries in cartesian space:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_save_as_text --output_directory /tmp/queries --group_prefix ra --cartesian --eef ra_wrist_3_link
  ```
  Example of exporting a query: cartesian_example_scene.queries
  ```
  example_scene
  cartesian_query
  START
  ra_shoulder_pan_joint = 2.64932
  ra_shoulder_lift_joint = -1.99619
  ra_elbow_joint = -1.29736
  ra_wrist_1_joint = 1.87004
  ra_wrist_2_joint = -2.11698
  ra_wrist_3_joint = 2.63777
  .
  GOAL
  position_constraint
  End_effector = ra_wrist_3_link
  Position = -0.317122 -0.130014 0.548275
  Orientation = 0.0291105 0.158759 0.383943 0.90914
  ```
  
## Import from text

An executable is available to allow to import .scenes and .queries files to the Moveit! warehouse:

```
rosrun moveit_ros_warehouse moveit_warehouse_import_from_text
```

It has different parameters that can be specified:
* `help`: Show help message
* `host`: Host for the DB. Default 127.0.0.1.
* `port`: Port for the DB. Default 33829.
* `queries`: Name of file containing motion planning queries.
* `scene`: "Name of file containing motion planning scene."

To be able to use this command, the robot should be launched in another terminal first.

Here are a few examples of how to use them:
* Example to import a scene file:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_import_from_text --scene example_scene.scene
  ```
* Example to import a query file:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_import_from_text --queries example_scene.queries
  ```

## Generate random valid queries

An executable is available to allow to the generation of a specified number of valid (collision free) random queries and save them into the Moveit! warehouse:

```
rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries [name_of_scene] [number_of_random_queries]
```

It has different parameters that can be specified:
* `help`: Show help message
* `host`: Host for the DB. Default 127.0.0.1.
* `port`: Port for the DB. Default 33829.
* `limited_joints`: Limit joints from -pi to pi to avoid a lot of impossible queries
* `group_prefix`: Specify the group prefix you'd like to plan with. This is useful if you want to save the queries only for the arm and it has a prefix, so only those joints will be considered (e.g. "ra")
* `cartesian`: Generate the cartesian space query equivalent to the one generated in joint space
* `eef`: Specify the end effector (Only needed when the cartesian option is set). Default: last link. 
* `clear`: Clears all the random queries for a given scene

After generating the queries, they are saved in the warehouse. You can use the commands above to export them to text.

Here are a few examples of how to use them:
* Example of generating 10 random queries in joint space in the example_scene:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries example_scene 10 --group_prefix ra
  ```
* Example of generating 10 random queries in joint space in the example_scene limiting the joints:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries example_scene 10 --group_prefix ra --limited_joints
  ```
* Example of generating 10 random queries in cartesian space in the example_scene:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries example_scene 10 --group_prefix ra --cartesian --eef ra_wrist_3_link
  ```
* Example of clearing the random queries in example_scene:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries example_scene --clear
  ```