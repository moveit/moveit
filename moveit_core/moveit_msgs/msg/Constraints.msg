# This message contains a list of motion planning constraints.
# All constraints must be satisfied for a goal to be considered valid

string name

JointConstraint[] joint_constraints

PositionConstraint[] position_constraints

OrientationConstraint[] orientation_constraints

VisibilityConstraint[] visibility_constraints
