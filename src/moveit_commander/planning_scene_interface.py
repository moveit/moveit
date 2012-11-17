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
#from moveit_python_interface.arm_planner import ArmPlanner
#from moveit_python_interface import conversions
#from moveit_python_interface import trajectory_tools
#from pr2_python.exceptions import ArmNavError, ActionFailedError

roslib.load_manifest('move_group_interface')
from _planning_scene_interface import PlanningSceneInterface

#DEFAULT_PLANNER_SERVICE_NAME = 'ompl_planning/plan_kinematic_path'

class PlanningScene:

    def __init__(self):
        self._g = PlanningSceneInterface()

    def add_simple_object(self, id_name, frame_id, type_name, pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w, dimensions):
        self._g.add_simple_object(id_name, frame_id, type_name, pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w, dimensions)

    def remove_simple_object(self, id_name, frame_id):
        self._g.remove_simple_object(id_name, frame_id)

    def attach_simple_collision_object(self, id_name, frame_id, type_name, link_name, touch_links, pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w, dimensions):
        self._g.attach_simple_collision_object(id_name, frame_id, type_name, link_name, touch_links, pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w, dimensions)

    def remove_simple_attached_object(self, id_name, frame_id, link_name):
        self._g.remove_simple_attached_object(id_name, frame_id, link_name)


    #def add_simple_object(self, filepath):
    #    self._g.add_simple_object(filepath)
