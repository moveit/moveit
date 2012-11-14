# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Author: Sarah Elliott

import time, copy, threading
import numpy as np
import roslib
roslib.load_manifest('moveit_commander')
import rospy
import actionlib
import actionlib_msgs.msg
import tf
#import arm_navigation_msgs.msg
#from arm_navigation_msgs.msg import ArmNavigationErrorCodes as ArmNavErrorCodes
#from arm_navigation_msgs.srv import GetStateValidityRequest
#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
#from moveit_msgs.msg import RobotTrajectory
#import moveit_msgs_boost as mmb
#from pr2_python.world_interface import WorldInterface
#from pr2_python.hand_description import HandDescription
#from pr2_python.controller_manager_client import ControllerManagerClient
#from pr2_python.cartesian_controller_interface import CartesianControllerInterface
#from moveit_commander.arm_planner import ArmPlanner
#from moveit_commander import conversions
#from moveit_commander import trajectory_tools
#from pr2_python.exceptions import ArmNavError, ActionFailedError

roslib.load_manifest('move_group_interface')
from move_group_interface import MoveGroup

#DEFAULT_PLANNER_SERVICE_NAME = 'ompl_planning/plan_kinematic_path'

class ArmMover:
    def __init__(self, name):
        self._g = MoveGroup(name)

    def move_arm(self, goal = None):
        """ goal (geometry_msgs.msg.PoseStamped or sensor_msgs.msg.JointState) """
        if type(goal) == JointState:
            self._g.set_joint_value_target(goal.position);

        elif type(goal) == Pose:  
            if self.has_end_effector_link():
                pose = []
                pose.append(goal.position.x)
                pose.append(goal.position.y)
                pose.append(goal.position.z)
                (r, p, y) = tf.transformations.euler_from_quaternion([goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w])
                pose.append(r)
                pose.append(p)
                pose.append(y)
                self._g.set_pose_target(pose)
            else:
                raise "There is no end effector to get the pose of"
        return self._g.move()

    def move_relative(self, x, y, z, rot_x, rot_y, rot_z):
        if self.has_end_effector_link():
            pose = self.get_current_pose()
            pose[0] = pose[0] + x
            pose[1] = pose[1] + y
            pose[2] = pose[2] + z
            pose[3] = pose[3] + rot_x
            pose[4] = pose[4] + rot_y
            pose[5] = pose[5] + rot_z
            self.set_pose_target(pose)
            return self.move_arm()
        
    def plan_relative(self, x, y, z, rot_x, rot_y, rot_z):
        if self.has_end_effector_link():
            pose = self.get_current_pose()
            pose[0] = pose[0] + x
            pose[1] = pose[1] + y
            pose[2] = pose[2] + z
            pose[3] = pose[3] + rot_x
            pose[4] = pose[4] + rot_y
            pose[5] = pose[5] + rot_z
            self.set_pose_target(pose)
            return self.plan()

    def set_pose_target(self, pose):
        """ Set the pose of the end-effector, if one is available. The expected input is a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z]"""
        if self.has_end_effector_link():
            return self._g.set_pose_target(pose)
        else:
            raise "There is no end effector to get the pose of"

    def get_current_joint_values(self):
        """should return JointState.msg"""
        msg = JointState()
        msg.position = self._g.get_current_joint_values()
        return msg

    def get_current_pose(self):
        if self.has_end_effector_link():
            msg = self._g.get_current_pose()
            return msg
        else:
            raise "There is no end effector to get the pose of"

    def get_current_pose_msg(self):
        if self.has_end_effector_link():
            msg = Pose()
            pose = self._g.get_current_pose()
            msg.position.x = pose[0]
            msg.position.y = pose[1]
            msg.position.z = pose[2]
            q = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
            print "Q is :"
            print q
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            return msg
        else:
            raise "There is no end effector to get the pose of"

    def cancel_move(self):
        """ Stop the current execution, if any """
        self._g.stop()

    def plan(self, goal = None):
        if type(goal) == JointState:
            self._g.set_joint_value_target(goal.position);

        elif type(goal) == PoseStamped:
            if self.has_end_effector_link():
                pose = []
                pose.append(goal.position.x)
                pose.append(goal.position.y)
                pose.append(goal.position.z)
                (r, p, y) = tf.transformations.euler_from_quaternion([goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w])
                pose.append(r)
                pose.append(p)
                pose.append(y)
                self._g.set_pose_target(pose)
            else:
                raise "There is no end effector to get the pose of"
        plan = self._g.get_plan()
        plan_msg = RobotTrajectory()
        joint_traj = JointTrajectory()
        joint_traj.joint_names = plan["joint_trajectory"]["joint_names"]
        for point in plan["joint_trajectory"]["points"]:
            joint_traj.points.append(JointTrajectoryPoint(
                positions = point["positions"],
                velocities = point["velocities"],
                accelerations = point["accelerations"]))
        multi_dof_joint_traj = MultiDOFJointTrajectory()
        multi_dof_joint_traj.joint_names = plan["multi_dof_joint_trajectory"]["joint_names"]
        multi_dof_joint_traj.frame_ids = plan["multi_dof_joint_trajectory"]["frame_ids"]
        multi_dof_joint_traj.child_frame_ids = plan["multi_dof_joint_trajectory"]["child_frame_ids"]
        for point in plan["multi_dof_joint_trajectory"]["points"]:
             multi_dof_joint_traj_point = MultiDOFJointTrajectoryPoint()
             for pose in point["poses"]:
                 multi_dof_joint_traj_point.poses.append(Point(
                     position = Point(x = pose["position"]["x"], y = pose["position"]["y"], z = pose["position"]["z"]), 
                     orientation = Quaternion(x = pose["orientation"]["x"], y = pose["orientation"]["y"], 
                         z = pose["orientation"]["z"], w = pose["orientation"]["w"])))
             multi_dof_joint_traj.points.append(multi_dof_joint_traj_point)
        plan_msg.joint_trajectory = joint_traj
        plan_msg.multi_dof_joint_trajectory = multi_dof_joint_traj
        return plan_msg

    def has_end_effector_link(self):
        """ Check if this group has a link that is considered to be an end effector """
        return len(self._g.get_end_effector_link()) > 0

    def remember_joint_values(self, name, values = None):
        if values == None:
            values = self.get_current_joint_values().position
        self._g.remember_joint_values(name, values)

    def get_remembered_joint_values(self):
        return self._g.get_remembered_joint_values()

    def forget_joint_values(self, name):
        self._g.forget_joint_values(name)

    def set_joint_value_target(self, name, value = None):
        if value == None:
            value = name
            self._g.set_joint_value_target(value)
        else:
            self._g.set_joint_value_target(name, value)
