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
roslib.load_manifest('moveit_python_interface')
import rospy
import actionlib
import actionlib_msgs.msg
import tf
#import arm_navigation_msgs.msg
#from arm_navigation_msgs.msg import ArmNavigationErrorCodes as ArmNavErrorCodes
#from arm_navigation_msgs.srv import GetStateValidityRequest
#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from moveit_msgs.msg import RobotTrajectory
#import moveit_msgs_boost as mmb
#from pr2_python.world_interface import WorldInterface
#from pr2_python.hand_description import HandDescription
#from pr2_python.controller_manager_client import ControllerManagerClient
#from pr2_python.cartesian_controller_interface import CartesianControllerInterface
#from moveit_python_interface.arm_planner import ArmPlanner
#from moveit_python_interface import conversions
#from moveit_python_interface import trajectory_tools
#from pr2_python.exceptions import ArmNavError, ActionFailedError

roslib.load_manifest('move_group_interface')
from move_group_interface import MoveGroup

#DEFAULT_PLANNER_SERVICE_NAME = 'ompl_planning/plan_kinematic_path'

class ArmMover:
    def __init__(self, name):
        self._g = MoveGroup(name)

    def move_arm(self, goal):
        """ goal (geometry_msgs.msg.PoseStamped or sensor_msgs.msg.JointState) """
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
        return self._g.move()

    def get_current_joint_values(self):
        """should return JointState.msg"""
        msg = JointState()
        msg.position = self._g.get_current_joint_values()
        return msg

    def get_current_pose(self):
        if self.has_end_effector_link():
            msg = PoseStamped()
            msg = self._g.get_current_pose()
            return msg
        else:
            raise "There is no end effector to get the pose of"

    def cancel_move(self):
        """ Stop the current execution, if any """
        self._g.stop()

    def plan(self, goal):
        if type(goal) == JointState:
            self._g.set_joint_value_target(goal.position);

        elif type(goal) == PoseStamped:
            if self.has_end_effector_link():
                self._g.set_pose_target(pose)
            else:
                raise "There is no end effector to get the pose of"
        plan = self._g.get_plan()
        plan_msg = JointTrajectory()
        plan_msg.joint_names = plan["joint_names"]
        for point in plan["points"]:
            plan_msg.points.append(JointTrajectoryPoint(
                positions = point["positions"],
                velocities = point["velocities"],
                accelerations = point["accelerations"]))
        return plan_msg

        
        
       
