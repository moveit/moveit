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
# Author: Bhaskara Marthi

# Functions for moving the torso

import roslib
roslib.load_manifest('moveit_commander')
import rospy
#import actionlib

#import moveit_commander.exceptions as ex
#import pr2_controllers_msgs.msg as pr2c
from std_msgs.msg import Float64
#import actionlib_msgs.msg as am
#import actionlib as al

class Torso(object):

    def __init__(self, topic_name):
        """
        Represents the torso of the pr2.
        """
        self._pub = rospy.Publisher(topic_name, Float64)
        rospy.sleep(2.0)

        #pr2_controllers_msgs not catkinized

        #rospy.init_node('torso_lift')

        #name = 'torso_controller/position_joint_action'
        #self._ac = al.SimpleActionClient(name,pr2c.SingleJointPositionAction)
        #rospy.loginfo("Waiting for torso controller action server")
        #self._ac.wait_for_server()
        #rospy.loginfo("Torso control action client ready")

    def move(self, pos):
        """
        Move torso to a given height

        :param position: Desired height (m)
        """
        self._pub.publish(Float64(pos))

        #pr2_controllers_msgs not catkinized

        #goal = pr2c.SingleJointPositionGoal(position=pos,
        #                                    min_duration=rospy.Duration(2),
        #                                    max_velocity=1)
        #rospy.loginfo("Sending torso goal and waiting for result")
        #self._ac.send_goal_and_wait(goal)
        #res = self._ac.get_result()
        #if self._ac.get_state() != am.GoalStatus.SUCCEEDED:
        #    raise ex.ActionFailedError()

    def up(self):
        """
        Move torso to max height (0.3)
        """
        self.move(0.3)

    def down(self):
        """
        Move torso to min height (0)
        """
        self.move(0.01)
        
