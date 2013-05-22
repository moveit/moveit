# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
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

from moveit_commander import MoveGroupCommander, MoveItCommanderException
from moveit_ros_planning_interface import _moveit_robot_interface
import conversions

class RobotCommander(object):

    class Joint(object):
        def __init__(self, robot, name):
            self._robot = robot
            self._name = name
        def name(self):
            return self._name
        def variable_count(self):
            return len(self.__get_joint_limits())
        def bounds(self):
            l = self.__get_joint_limits()
            if len(l) == 1:
                return l[0]
            else:
                return l
        def min_bound(self):
            limits = self.__get_joint_limits()
            if len(limits) == 1:
                return limits[0][0]
            else:
                return [l[0] for l in limits]
        def max_bound(self):
            limits = self.__get_joint_limits()
            if len(limits) == 1:
                return limits[0][1]
            else:
                return [l[1] for l in limits]
        def value(self):
            vals = self._robot._r.get_current_joint_values(self._name)
            if len(vals) == 1:
                return vals[0]
            else:
                return vals
        def move(self, position, wait = True):
            group = self._robot._r.find_min_containing_group(self._name)
            if group is None:
                raise MoveItCommanderException("There is no known group containing joint %s. Cannot move." % self._name)
            gc = self._robot.get_group(group)
            if gc is not None:
                gc.set_joint_value_target(gc.get_current_joint_values())
                gc.set_joint_value_target(self._name, position)
                return gc.go(wait)
            return False
        def __get_joint_limits(self):
            return self._robot._r.get_joint_limits(self._name)

    class Link(object):
        def __init__(self, robot, name):
            self._robot = robot
            self._name = name
        def name(self):
            return self._name
        def pose(self):
            return conversions.list_to_pose_stamped(self._robot._r.get_link_pose(self._name), self._robot.get_planning_frame())

    def __init__(self):
        self._r = _moveit_robot_interface.RobotInterface("robot_description")
        self._groups = {}

    def get_planning_frame(self):
        """Get the frame of reference in which planning is done (and environment is maintained)"""
        return self._r.get_planning_frame()

    def get_joint_names(self):
        """ Get the names of all the joints that make up the robot model. This includes fixed joints and virtual joints"""
        return self._r.get_joint_names()

    def get_link_names(self):
        """Get the links that make up the robot"""
        return self._r.get_link_names()

    def get_group_names(self):
        """Get the names of the groups defined for the robot"""
        return self._r.get_group_names()

    def get_current_variable_values(self):
        """Get a dictionary mapping variable names to values. Note that a joint may consist of one or more variables """
        return self._r.get_current_variable_values()

    def get_joint(self, name):
        return self.Joint(self, name)

    def get_link(self, name):
        return self.Link(self, name)

    def get_group(self, name):
        if not self._groups.has_key(name):
            self._groups[name] = MoveGroupCommander(name)
        return self._groups[name]

    def __getattr__(self, name):
        """ We catch the names of groups, joints and links to allow easy access to their properties """
        if name in self.get_group_names():
            return self.get_group(name)
        elif name in self.get_joint_names():
            return self.get_joint(name)
        elif name in self.get_link_names():
            return self.get_link(name)
        else:
            return object.__getattribute__(self, name)
