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
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from _moveit_planning_scene_interface import PlanningSceneInterface

#DEFAULT_PLANNER_SERVICE_NAME = 'ompl_planning/plan_kinematic_path'

class PlanningScene:

    def __init__(self):
        self._g = PlanningSceneInterface()

    def add_sphere(self, id_name, frame_id, position, orientation, radius):
        """
        Add sphere to the planning scene in specified frame
        position: [x,y,z]
        orientation: [x,y,z,w]
        
        """
        self._g.add_sphere(id_name, frame_id, position, orientation, radius)

    def add_cylinder(self, id_name, frame_id, position, orientation, height, radius):
        
        self._g.add_cylinder(id_name, frame_id, position, orientation, height, radius)

    def add_box(self, id_name, frame_id, position, orientation, length, width, height):
        self._g.add_box(id_name, frame_id, position, orientation, length, width, height)

    def add_cone(self, id_name, frame_id, position, orientation, height, radius):
        self._g.add_cone(id_name, frame_id, position, orientation, height, radius)

    def remove_simple_object(self, id_name, frame_id):
        """
        Remove object from planning scene with id_name and frame_id
        
        """
        self._g.remove_simple_object(id_name, frame_id)

    def attach_sphere(self, id_name, frame_id, link_name, touch_links, position, orientation, radius):
        """
        Add sphere to the planning scene in attached to specified link
        touch_links: [link1, link2, link3, etc] <- links that object can touch
        position: [x,y,z]
        orientation: [x,y,z,w]
        
        """

        self._g.attach_sphere(id_name, frame_id, link_name, touch_links, position, orientation, radius)

    def attach_cylinder(self, id_name, frame_id, link_name, touch_links, position, orientation, height, radius):
        self._g.attach_cylinder(id_name, frame_id, link_name, touch_links, position, orientation, height, radius)

    def attach_box(self, id_name, frame_id, link_name, touch_links, position, orientation, length, width, height):
        self._g.attach_box(id_name, frame_id, link_name, touch_links, position, orientation, length, width, height)

    def attach_cone(self, id_name, frame_id, link_name, touch_links, position, orientation, height, radius):
        self._g.attach_cone(id_name, frame_id, link_name, touch_links, position, orientation, height, radius)

    def remove_simple_attached_object(self, id_name, frame_id, link_name):
        """
        Detach object from link, add it back into planning scene
        
        """

        self._g.remove_simple_attached_object(id_name, frame_id, link_name)


