# This service contains the definition for a request to the motion
# planner and the output it provides

# Parameters for the workspace that the planner should work inside
WorkspaceParameters workspace_parameters

# Starting state updates. If certain joints should be considered
# at positions other than the current ones, these positions should
# be set here
RobotState start_state

# The possible goal states for the model to plan for. Each element of
# the array defines a goal region. The goal is achieved
# if the constraints for a particular region are satisfied
Constraints[] goal_constraints

# No state at any point along the path in the produced motion plan will violate these constraints
Constraints path_constraints

# The name of the motion planner to use. If no name is specified,
# a default motion planner will be used
string planner_id

# The name of the group of joints on which this planner is operating
string group_name

# The number of times this plan is to be computed. Shortest solution
# will be reported.
int32 num_planning_attempts

# The maximum amount of time the motion planner is allowed to plan for
duration allowed_planning_time
