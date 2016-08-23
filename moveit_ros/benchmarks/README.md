moveit_ros_benchmarks
---------------------
This package provides methods to benchmark motion planning algorithms and aggregate/plot statistics.  The core of the functionality lies in the BenchmarkExecutor class.  Using the BenchmarkExecutor requires an instance of the BenchmarkOptions class, which reads benchmark parameters from the ROS parameter server.

Example
-------

An example is provided in the examples folder. The launch file requires a moveit config package 
for the Fanuc M10ia available from here: https://github.com/sachinchitta/moveit_examples

To run:

1. Launch the Fanuc M10ia demo.launch: roslaunch fanuc_m10ia_moveit_config demo.launch db:=true
2. Connect to the database.
3. Save a scene and associated query for the Fanuc M10ia. Name the scene Kitchen1 and the 
   query Pick1.
4. Also save a start state for the robot and name it Start1.
5. The config file demo1.yaml refers to the scenes, queries and start states used for benchmarking. Modify them appropriately.
6. Now, bring down your previous launch file and then run the benchmarks: roslaunch moveit_ros_benchmarks demo_fanuc_m10ia.launch

BenchmarkOptions
----------------
This class reads in parameters and options for the benchmarks to run from the ROS parameter server.  The format of the parameters is assumed to be in the following form:

benchmark_config:

    warehouse:
        host: [hostname/IP address of ROS Warehouse node]                           # Default localhost
        port: [port number of ROS Warehouse node]                                   # Default 33829
        scene_name: [Name of the planning scene to use for benchmarks]              # REQUIRED

    parameters:
        runs: [Number of runs for each planning algorithm on each request]          # Default 10
        group: [The name of the group to plan]                                      # REQUIRED
        timeout: [The maximum time for a single run; seconds]                       # Default 10.0
        output_directory: [The directory to write the output to]                    # Default is current working directory

        start_states: [Regex for the stored start states in the warehouse to try]   # Default ""
        path_constraints: [Regex for the path constraints to benchmark]             # Default ""

        queries: [Regex for the motion plan queries in the warehouse to try]        # Default .*
        goal_constraints: [Regex for the goal constraints to benchmark]             # Default ""
        trajectory_constraints: [Regex for the trajectory constraints to benchmark] # Default ""

        workspace: [Bounds of the workspace the robot plans in.  This is an AABB]   # Optional
            frame_id: [The frame the workspace parameters are specified in]
            min_corner: [Coordinates of the minimum corner of the AABB]
                x: [x-value]
                y: [y-value]
                z: [z-value]
            max_corner: [Coordinates of the maximum corner of the AABB]
                x: [x-value]
                y: [y-value]
                z: [z-value]
    planners:
        - plugin: [The name of the planning plugin the planners are in]             # REQUIRED
          planners:                                                                 # REQUIRED
            - A list of planners
            - from the plugin above
            - to benchmark the
            - queries in.
        - plugin: ...
            - ...

BenchmarkExecutor
-----------------
This class creates a set of MotionPlanRequests that respect the parameters given in the supplied instance of BenchmarkOptions and then executes the requests on each of the planners specified.  From the BenchmarkOptions, queries, goal_constraints, and trajectory_constraints are treated as separate queries.  If a set of start_states is specified, each query, goal_constraints, and trajectory_constraints is attempted with each start state (existing start states from a query are ignored).  Similarly, the (optional) set of path constraints is combined combinatorially with the start-query and start-goal_constraints pairs (existing path_constraints) from a query are ignored).  The workspace, if specified, overrides any existing workspace parameters.

The benchmarks are executed and many interesting parameters are aggregated and written to a logfile.  A script (moveit_benchmark_statistics.py) is supplied to parse this data and plot the statistics.  The benchmarking pipeline does not utilize MoveGroup, and PlanningRequestAdaptors are NOT invoked.

It is possible to customize a benchmark run by deriving a class from BenchmarkExecutor and overriding one or more of the virtual functions.  Additionally, a set of functions exists for ease of customization in derived classes:
    - preRunEvent (invoked immediately before each call to solve)
    - postRunEvent (invoked immediately after each call to solve)
    - plannerSwitchEvent (invoked when the planner changes during benchmarking)
    - querySwitchEvent (invoked before a new benchmark problem begin execution)

Note, in the above, a benchmark is a concrete instance of a PlanningScene, start state, goal constraints / trajectory_constraints, and (optionally) path_constraints.  A run is one attempt by a specific planner to solve the benchmark.


