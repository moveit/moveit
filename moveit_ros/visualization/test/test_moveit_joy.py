#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, TORK (Tokyo Opensource Robotics Kyokai Association)
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
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association
#    nor the names of its contributors may be used to endorse or promote 
#    products derived from this software without specific prior written 
#    permission.
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


import unittest

from geometry_msgs.msg import Pose
from moveit_ros_visualization.moveit_joy import MoveitJoy
import rospy

import math
from tf.transformations import quaternion_from_euler

_PKGNAME = 'moveit_ros_visualization'
_NODENAME = 'test_moveit_joy'

class TestMoveitJoy(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rospy.init_node(_NODENAME)
        self.moveit_joy = MoveitJoy()

    @classmethod
    def tearDownClass(self):
        True  # TODO impl something meaningful

    def test_updatePlanningGroup_exception(self):
        '''Test MoveitJoy.updatePlanningGroup'''
        exception_raised = False
        try:
            # Passng 0 to MoveitJoy.updatePlanningGroup should raise an exception.
            self.moveit_joy.updatePlanningGroup(0)
        except rospy.ROSInitException:
            exception_raised = True
        self.assertTrue(exception_raised)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(_PKGNAME, _NODENAME, TestMoveitJoy) 
