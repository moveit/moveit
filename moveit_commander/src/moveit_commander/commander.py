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

    def has_end_effector_link(self):
        """ Check if this group has a link that is considered to be an end effector """
        return len(self._g.get_end_effector_link()) > 0

    def get_end_effector_link(self):
        """ Get the name of the link that is considered to be an end effector """
        return self._g.get_end_effector_link()

    def set_end_effector_link(self, link_name):
        """ Set the name of the link to be considered as an end effector """
        self._g.set_end_effector_link(link_name)

    def get_pose_reference_frame(self):
        """ Get the reference frame assumed for poses of end-effectors """
        return self._g.get_pose_reference_frame()

    def set_pose_reference_frame(self, reference_frame):
        """ Set the reference frame to assume for poses of end-effectors """
        self._g.set_pose_reference_frame(reference_frame)

    def get_robot_root_link(self):
        return self._g.get_robot_root_link()
    
    def get_planning_frame(self):
        return self._g.get_planning_frame()

    def get_current_joint_values(self):
        return self._g.get_current_joint_values()

    def get_current_pose(self, end_effector_link = ""):
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            return conversions.list_to_pose_stamped(self._g.get_current_pose(end_effector_link), self.get_planning_frame())
        else:
            raise MoveItCommanderException("There is no end effector to get the pose of")

    def get_current_rpy(self, end_effector_link = ""):
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
        if value == None:
            value = name
            self._g.set_joint_value_target(value)
        else:
            self._g.set_joint_value_target(name, value)

    def set_rpy_target(self, rpy, end_effector_link = ""):
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            if len(rpy) == 3:
                self._g.set_rpy_target(rpy[0], rpy[1], rpy[2], end_effector_link)
            else:
                raise MoveItCommanderException("Expected [roll, pitch, yaw]")
        else:
            raise MoveItCommanderException("There is no end effector to set the pose for")

    def set_orientation_target(self, q, end_effector_link = ""):
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            if len(q) == 4:
                self._g.set_orientation_target(q[0], q[1], q[2], q[3], end_effector_link)
            else:
                raise MoveItCommanderException("Expected [qx, qy, qz, qw]")
        else:
            raise MoveItCommanderException("There is no end effector to set the pose for")

    def set_position_target(self, xyz, end_effector_link = ""):
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            self._g.set_position_target(xyz[0], xyz[1], xyz[2], end_effector_link)
        else:
            raise MoveItCommanderException("There is no end effector to set the pose for")

    def set_pose_target(self, pose, end_effector_link = ""):
        """ Set the pose of the end-effector, if one is available. The expected input is a Pose message, a PoseStamped message or a list of 6 floats:"""
        """ [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw] """
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            if type(pose) is PoseStamped:
                old = self.get_pose_reference_frame()
                self.set_pose_reference_frame(pose.header.frame_id)
                self._g.set_pose_target(conversions.pose_to_list(pose.pose), end_effector_link)
                self.set_pose_reference_frame(old)
            elif type(pose) is Pose:
                self._g.set_pose_target(conversions.pose_to_list(pose), end_effector_link)
            else:
                self._g.set_pose_target(pose, end_effector_link)
        else:
            raise MoveItCommanderException("There is no end effector to set the pose for")

    def set_pose_targets(self, poses, end_effector_link = ""):
        """ Set the pose of the end-effector, if one is available. The expected input is a list of poses. Each pose can be a Pose message, a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw] """
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            self._g.set_pose_targets([conversions.pose_to_list(p) if type(p) is Pose else p for p in poses], end_effector_link)
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
                self._g.set_pose_target(pose, end_effector_link)
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
        self._g.set_random_target()

    def set_named_target(self, name):
        if not self._g.set_named_target(name):
            raise MoveItCommanderException("Unable to set target " + name)

    def remember_joint_values(self, name, values = None):
        if values == None:
            values = self.get_current_joint_values()
        self._g.remember_joint_values(name, values)

    def get_remembered_joint_values(self):
        return self._g.get_remembered_joint_values()
    
    def forget_joint_values(self, name):
        self._g.forget_joint_values(name)

    def get_goal_tolerance(self):
        return self._g.get_goal_tolerance()

    def set_goal_tolerance(self, value):
        self._g.set_goal_tolerance(value)

    def allow_looking(self, value):
        self._g.allow_looking(value)

    def allow_replanning(self, value):
        self._g.allow_replanning(value)
        
    def get_known_constraints(self):
        return self._g.get_known_constraints()

    def set_path_constraints(self, value):
        if value == None:
            self.clear_path_constraints()
        else:
            if not self._g.set_path_constraints(value):
                raise MoveItCommanderException("Unable to set path constraints " + value)

    def clear_path_constraints(self):
        self._g.clear_path_constraints()

    def set_constraints_database(self, host, port):
        self._g.set_constraints_database(host, port)

    def set_planning_time(self, seconds):
        self._g.set_planning_time(seconds)

    def get_planning_time(self):
        return self._g.get_planning_time()

    def set_planner_id(self, planner_id):
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
            self._g.set_joint_value_target(joints.position)

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
            self._g.set_joint_value_target(joints.position)

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
        (dpath, fraction) = self._g.compute_cartesian_path([conversions.pose_to_list(p) for p in waypoints], eef_step, jump_threshold, avoid_collisions)
        return (conversions.dict_to_trajectory(dpath), fraction)

    def execute(self, plan_msg):
        """Execute a previously planned path"""
        return self._g.execute(conversions.trajectory_to_dict(plan_msg))

    def pick(self, object_name):
        """Pick the named object"""
        return self._g.pick(object_name)

    def place(self, object_name):
        """Place the named object"""
        return self._g.place(object_name)
