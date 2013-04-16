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
# Author: Ioan Sucan

import rospy
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

# This is going to have more functionality; (feel free to add some!)
# This class will include simple Python code for publishing messages for a planning scene

class PlanningSceneInterface:
    """ Simple interface to making updates to a planning scene """

    def __init__(self):
        self._pub_co = rospy.Publisher('/collision_object', CollisionObject)
        self._pub_aco = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)

    def add_sphere(self, name, pose, radius = 1):
        """
        Add sphere to the planning scene 
        """
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [radius]
        co.primitives = [sphere]
        co.primitive_poses = [pose.pose]
        self._pub_co.publish(co)
        
    def remove_object(self, name):
        """
        Remove object from planning scene         
        """
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.id = name
        self._pub_co.publish(co)
