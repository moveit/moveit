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

# Functions to control the gripper

import roslib
roslib.load_manifest('moveit_commander')
import rospy
import actionlib
import moveit_commander.exceptions as ex
import pr2_controllers_msgs.msg as pr2c
import actionlib_msgs.msg as am


class Gripper(object):
    """
    Represents a gripper of the PR2
    """

    def __init__(self, side):
        """
        :param side: A string, either 'left_arm' or 'right_arm'
        """
        assert side in ['left_arm', 'right_arm']
        self._side = side
        #action_name = '{0}_gripper_controller/gripper_action'.\
        #              format('l' if side=='left_arm' else 'r')
        action_name = 'low_cost_gripper_controller/gripper_action'
        self._ac = actionlib.SimpleActionClient(action_name,
                                                pr2c.Pr2GripperCommandAction)
        rospy.loginfo("Waiting for action server {0}...".format(action_name))
        self._ac.wait_for_server()
        rospy.loginfo("Connected to action server {0}".format(action_name))

    def open(self, max_effort=5):
        """
        Open this gripper
        """
        self.move(0.135, max_effort)

    def close(self, max_effort=5):
        """
        Close this gripper
        """
        self.move(0.0, max_effort)

    def move(self, position, max_effort=5):
        goal = pr2c.Pr2GripperCommandGoal(
            pr2c.Pr2GripperCommand(position=position, max_effort=max_effort))
        self._ac.send_goal(goal)
        rospy.loginfo("Sending goal to gripper and waiting for result...")
        self._ac.wait_for_result()
        rospy.loginfo("Gripper action returned")
        if self._ac.get_state() != am.GoalStatus.SUCCEEDED:
            raise ex.ActionFailedError()

