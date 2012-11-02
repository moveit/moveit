int32 SUCCESS = 1
int32 PLACE_OUT_OF_REACH = 2
int32 PLACE_IN_COLLISION = 3
int32 PLACE_UNFEASIBLE = 4
int32 PREPLACE_OUT_OF_REACH = 5
int32 PREPLACE_IN_COLLISION = 6
int32 PREPLACE_UNFEASIBLE = 7
int32 RETREAT_OUT_OF_REACH = 8
int32 RETREAT_IN_COLLISION = 9
int32 RETREAT_UNFEASIBLE = 10
int32 MOVE_ARM_FAILED = 11
int32 PLACE_FAILED = 12
int32 RETREAT_FAILED = 13
int32 result_code

# whether the state of the world was disturbed by this attempt. generally, this flag
# shows if another task can be attempted, or a new sensed world model is recommeded
# before proceeding
bool continuation_possible
