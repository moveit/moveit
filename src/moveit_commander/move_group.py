# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
import rospy
import tf
from moveit_ros_planning_interface import _moveit_move_group_interface
from exception import MoveItCommanderException
import conversions

class MoveGroupCommander(object):
    """
    Execution of simple commands for a particular group
    """

    def __init__(self, name):
        """ Specify the group name for which to construct this commander instance. Throws an exception if there is an initialization error. """
        self._g = _moveit_move_group_interface.MoveGroup(name, "robot_description")

    def get_name(self):
        """ Get the name of the group this instance was initialized for """
        return self._g.get_name()

    def stop(self):
        """ Stop the current execution, if any """
        self._g.stop()

    def get_joints(self):
        """ Get the joints of this group """
        return self._g.get_joints()

    def get_variable_count(self):
        """ Return the number of variables used to parameterize a state in this group (larger or equal to number of DOF)"""
        return self._g.get_variable_count()

    def has_end_effector_link(self):
        """ Check if this group has a link that is considered to be an end effector """
        return len(self._g.get_end_effector_link()) > 0

    def get_end_effector_link(self):
        """ Get the name of the link that is considered to be an end-effector. Return an empty string if there is no end-effector. """
        return self._g.get_end_effector_link()

    def set_end_effector_link(self, link_name):
        """ Set the name of the link to be considered as an end effector """
        if not self._g.set_end_effector_link(link_name):
            raise MoveItCommanderException("Unable to set end efector link")

    def get_pose_reference_frame(self):
        """ Get the reference frame assumed for poses of end-effectors """
        return self._g.get_pose_reference_frame()

    def set_pose_reference_frame(self, reference_frame):
        """ Set the reference frame to assume for poses of end-effectors """
        self._g.set_pose_reference_frame(reference_frame)
    
    def get_planning_frame(self):
        """ Get the name of the frame where all planning is performed """
        return self._g.get_planning_frame()

    def get_current_joint_values(self):
        """ Get the current configuration of the group (as published on JointState) """
        return self._g.get_current_joint_values()

    def get_current_pose(self, end_effector_link = ""):
        """ Get the current pose of the end-effector of the group. Throws an exception if there is not end-effector. """
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            return conversions.list_to_pose_stamped(self._g.get_current_pose(end_effector_link), self.get_planning_frame())
        else:
            raise MoveItCommanderException("There is no end effector to get the pose of")

    def get_current_rpy(self, end_effector_link = ""):
        """ Get a list of 3 elements defining the [roll, pitch, yaw] of the end-effector. Throws an exception if there is not end-effector. """
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            return self._g.get_current_rpy(end_effector_link)
        else:
            raise MoveItCommanderException("There is no end effector to get the rpy of")

    def get_random_joint_values(self):
        return self._g.get_random_joint_values()

    def get_random_pose(self, end_effector_link = ""):
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            return conversions.list_to_pose_stamped(self._g.get_random_pose(end_effector_link), self.get_planning_frame())
        else:
            raise MoveItCommanderException("There is no end effector to get the pose of")

    def set_joint_value_target(self, name, value = None):
        """ Specify a target joint configuration for the group."""
        if value == None:
            value = name
            if not self._g.set_joint_value_target(value):
                raise MoveItCommanderException("Error setting joint target. Is the target within bounds?")
        else:
            if not self._g.set_joint_value_target(name, value):
                raise MoveItCommanderException("Error setting joint target. Is the target within bounds?")

    def set_rpy_target(self, rpy, end_effector_link = ""):
        """ Specify a target orientation for the end-effector. Any position of the end-effector is acceptable."""
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            if len(rpy) == 3:
                if not self._g.set_rpy_target(rpy[0], rpy[1], rpy[2], end_effector_link):
                    raise MoveItCommanderException("Unable to set orientation target")
            else:
                raise MoveItCommanderException("Expected [roll, pitch, yaw]")
        else:
            raise MoveItCommanderException("There is no end effector to set the pose for")

    def set_orientation_target(self, q, end_effector_link = ""):
        """ Specify a target orientation for the end-effector. Any position of the end-effector is acceptable."""
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            if len(q) == 4:
                if not self._g.set_orientation_target(q[0], q[1], q[2], q[3], end_effector_link):
                    raise MoveItCommanderException("Unable to set orientation target")
            else:
                raise MoveItCommanderException("Expected [qx, qy, qz, qw]")
        else:
            raise MoveItCommanderException("There is no end effector to set the pose for")

    def set_position_target(self, xyz, end_effector_link = ""):
        """ Specify a target position for the end-effector. Any orientation of the end-effector is acceptable."""
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            if not self._g.set_position_target(xyz[0], xyz[1], xyz[2], end_effector_link):
                raise MoveItCommanderException("Unable to set position target")
        else:
            raise MoveItCommanderException("There is no end effector to set the pose for")

    def set_pose_target(self, pose, end_effector_link = ""):
        """ Set the pose of the end-effector, if one is available. The expected input is a Pose message, a PoseStamped message or a list of 6 floats:"""
        """ [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw] """
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            ok = False
            if type(pose) is PoseStamped:
                old = self.get_pose_reference_frame()
                self.set_pose_reference_frame(pose.header.frame_id)
                ok = self._g.set_pose_target(conversions.pose_to_list(pose.pose), end_effector_link)
                self.set_pose_reference_frame(old)
            elif type(pose) is Pose:
                ok = self._g.set_pose_target(conversions.pose_to_list(pose), end_effector_link)
            else:
                ok = self._g.set_pose_target(pose, end_effector_link)
            if not ok:
                raise MoveItCommanderException("Unable to set target pose")
        else:
            raise MoveItCommanderException("There is no end effector to set the pose for")

    def set_pose_targets(self, poses, end_effector_link = ""):
        """ Set the pose of the end-effector, if one is available. The expected input is a list of poses. Each pose can be a Pose message, a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw] """
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            if not self._g.set_pose_targets([conversions.pose_to_list(p) if type(p) is Pose else p for p in poses], end_effector_link):
                raise MoveItCommanderException("Unable to set target poses")
        else:
            raise MoveItCommanderException("There is no end effector to set poses for")

    def shift_pose_target(self, axis, value, end_effector_link = ""):
        """ Get the current pose of the end effector, add value to the corresponding axis (0..5: X, Y, Z, R, P, Y) and set the new pose as the pose target """
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            pose = self._g.get_current_pose(end_effector_link)
            # by default we get orientation as a quaternion list
            # if we are updating a rotation axis however, we convert the orientation to RPY
            if axis > 2:
                (r, p, y) = tf.transformations.euler_from_quaternion(pose[3:])
                pose = [pose[0], pose[1], pose[2], r, p, y]
            if axis >= 0 and axis < 6:
                pose[axis] = pose[axis] + value
                self.set_pose_target(pose, end_effector_link)
            else:
                raise MoveItCommanderException("An axis value between 0 and 5 expected")
        else:
            raise MoveItCommanderException("There is no end effector to set poses for")

    def clear_pose_target(self, end_effector_link):
        """ Clear the pose target for a particular end-effector """
        self._g.clear_pose_target(end_effector_link)
        
    def clear_pose_targets(self):
        """ Clear all known pose targets """
        self._g.clear_pose_targets()

    def set_random_target(self):
        """ Set a random joint configuration target """
        self._g.set_random_target()

    def set_named_target(self, name):
        """ Set a joint configuration by name. The name can be a name previlusy remembered with remember_joint_values() or a configuration specified in the SRDF. """
        if not self._g.set_named_target(name):
            raise MoveItCommanderException("Unable to set target %s. Is the target within bounds?" % name)

    def remember_joint_values(self, name, values = None):
        """ Record the specified joint configuration of the group under the specified name. If no values are specified, the current state of the group is recorded. """
        if values == None:
            values = self.get_current_joint_values()
        self._g.remember_joint_values(name, values)

    def get_remembered_joint_values(self):
        """ Get a dictionary that maps names to joint configurations for the group """
        return self._g.get_remembered_joint_values()
    
    def forget_joint_values(self, name):
        """ Forget a stored joint configuration """
        self._g.forget_joint_values(name)

    def get_goal_tolerance(self):
        """ Return a tuple of goal tolerances: joint, position and orientation. """
        return (self.get_goal_joint_tolerance(), self.get_goal_position_tolerance(), self.get_goal_orientation_tolerance())

    def get_goal_joint_tolerance(self):
        """ Get the tolerance for achieving a joint goal (distance for each joint variable) """
        return self._g.get_goal_joint_tolerance()

    def get_goal_position_tolerance(self):
        """ When moving to a position goal or to a pose goal, the tolerance for the goal position is specified as the radius a sphere around the target origin of the end-effector """
        return self._g.get_goal_position_tolerance()

    def get_goal_orientation_tolerance(self):
        """ When moving to an orientation goal or to a pose goal, the tolerance for the goal orientation is specified as the distance (roll, pitch, yaw) to the target origin of the end-effector """
        return self._g.get_goal_orientation_tolerance()

    def set_goal_tolerance(self, value):
        """ Set the joint, position and orientation goal tolerances simultaneously """
        self._g.set_goal_tolerance(value)

    def set_goal_joint_tolerance(self, value):
        """ Set the tolerance for a target joint configuration """
        self._g.set_goal_joint_tolerance(value)

    def set_goal_position_tolerance(self, value):
        """ Set the tolerance for a target end-effector position """
        self._g.set_goal_position_tolerance(value)

    def set_goal_orientation_tolerance(self, value):
        """ Set the tolerance for a target end-effector orientation """
        self._g.set_goal_orientation_tolerance(value)

    def allow_looking(self, value):
        """ Enable/disable looking around for motion planning """
        self._g.allow_looking(value)

    def allow_replanning(self, value):
        """ Enable/disable replanning """
        self._g.allow_replanning(value)
        
    def get_known_constraints(self):
        """ Get a list of names for the constraints specific for this group, as read from the warehouse """
        return self._g.get_known_constraints()

    def set_path_constraints(self, value):
        """ Specify the path constraints to be used (as read from the database) """
        if value == None:
            self.clear_path_constraints()
        else:
            if not self._g.set_path_constraints(value):
                raise MoveItCommanderException("Unable to set path constraints " + value)

    def clear_path_constraints(self):
        """ Specify that no path constraints are to be used during motion planning """
        self._g.clear_path_constraints()

    def set_constraints_database(self, host, port):
        """ Specify which database to connect to for loading possible path constraints """
        self._g.set_constraints_database(host, port)

    def set_planning_time(self, seconds):
        """ Specify the amount of time to be used for motion planning. """
        self._g.set_planning_time(seconds)

    def get_planning_time(self):
        """ Specify the amount of time to be used for motion planning. """
        return self._g.get_planning_time()

    def set_planner_id(self, planner_id):
        """ Specify which planner to use when motion planning """
        self._g.set_planner_id(planner_id)

    def set_workspace(self, ws):
        """ Set the workspace for the robot as either [], [minX, minY, maxX, maxY] or [minX, minY, minZ, maxX, maxY, maxZ] """
        if len(ws) == 0:
            self._g.set_workspace(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            if len(ws) == 4:
                self._g.set_workspace(ws[0], ws[1], 0.0, ws[2], ws[3], 0.0)
            else:
                if len(ws) == 6:
                    self._g.set_workspace(ws[0], ws[1], ws[2], ws[3], ws[4], ws[5])
                else:
                    raise MoveItCommanderException("Expected 0, 4 or 6 values in list specifying workspace")

    def go(self, joints = None, wait = True):
        """ Set the target of the group and then move the group to the specified target """
        if type(joints) is bool:
            wait = joints
            joints = None

        elif type(joints) is JointState:
            self.set_joint_value_target(joints.position)

        elif type(joints) is Pose:
            self.set_pose_target(joints)

        elif not joints == None:
            try:
                self.set_joint_value_target(self.get_remembered_joint_values()[joints])
            except:
                self.set_joint_value_target(joints)
        if wait:
            return self._g.move()
        else:
            return self._g.async_move()

    def plan(self, joints = None):
        """ Return a motion plan (a RobotTrajectory) to the set goal state (or specified by the joints argument) """
        if type(joints) is JointState:
            self.set_joint_value_target(joints.position)

        elif type(joints) is Pose:
            self.set_pose_target(joints)

        elif not joints == None:
            try:
                self.set_joint_value_target(self.get_remembered_joint_values()[joints])
            except:
                self.set_joint_value_target(joints)
        plan = self._g.compute_plan()
        return conversions.dict_to_trajectory(plan)

    def compute_cartesian_path(self, waypoints, eef_step, jump_threshold, avoid_collisions = True):
        """ Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints. Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. """
        (dpath, fraction) = self._g.compute_cartesian_path([conversions.pose_to_list(p) for p in waypoints], eef_step, jump_threshold, avoid_collisions)
        return (conversions.dict_to_trajectory(dpath), fraction)

    def execute(self, plan_msg):
        """Execute a previously planned path"""
        return self._g.execute(conversions.trajectory_to_dict(plan_msg))

    def pick(self, object_name):
        """Pick the named object"""
        return self._g.pick(object_name)

    def place(self, object_name, pose):
        """Place the named object at a particular location in the environment"""
        result = False
        if type(pose) is PoseStamped:
            old = self.get_pose_reference_frame()
            self.set_pose_reference_frame(pose.header.frame_id)
            result = self._g.place(object_name, conversions.pose_to_list(pose.pose))
            self.set_pose_reference_frame(old)
        else:
            result = self._g.place(object_name, conversions.pose_to_list(pose))
        return result
