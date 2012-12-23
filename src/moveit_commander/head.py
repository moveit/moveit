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

# Functions to control the head

import roslib; roslib.load_manifest('moveit_commander')
import rospy
#import actionlib
import geometry_msgs.msg as gm
#from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

class Head(object):
    """
    Represents the pr2 head
    """

    def __init__(self, pan_topic, tilt_topic):
        #pr2_controllers_msgs not catkinized
        #name = '/head_traj_controller/point_head_action'
        #self._ac = actionlib.SimpleActionClient(name, PointHeadAction)
        #rospy.loginfo("Waiting for head control action server")
        #self._ac.wait_for_server()
        #rospy.loginfo("Connected to head control action server")
        #self.pointing_frame = rospy.get_param("head_pointing_frame", "/openni_rgb_optical_frame")

        self._pan_pub = rospy.Publisher(pan_topic, Float64)

        self._tilt_pub = rospy.Publisher(tilt_topic, Float64)
        rospy.sleep(2.0)

    def move(self, pan, tilt):
        """
        Hack because pr2_controllers_msgs is not catkinized
        """
        self._pan_pub.publish(Float64(pan))
        self._tilt_pub.publish(Float64(tilt))

    """
    def look_at_map_point(self, x, y, z):
        "Look at a point (x, y, z) specified in the map frame"
        self._point_head((x, y, z), "/map")

    def look_at_relative_point(self, x, y, z):
        "Look at a point (x, y, z) specified in the base_footprint frame"
        self._point_head((x, y, z), "/base_footprint")

    def _point_head(self, p, frame, execute_timeout = rospy.Duration(3.0), preempt_timeout = rospy.Duration(3.0)):
        p = gm.Point(*p)
        ps = gm.PointStamped(point=p)
        ps.header.frame_id = frame
        ps.header.stamp = rospy.Time.now()

        goal = PointHeadGoal(target=ps)
        goal.pointing_axis = gm.Vector3(0.0, 0.0, 1.0)
        goal.pointing_frame = self.pointing_frame
        goal.max_velocity = 1.0
        rospy.loginfo('Sending goal to head and waiting for result.')
        self._ac.send_goal_and_wait(goal,execute_timeout=execute_timeout,preempt_timeout=preempt_timeout)
    """
