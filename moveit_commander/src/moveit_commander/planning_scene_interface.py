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
# Author: Ioan Sucan, Felix Messmer

import rospy
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import PoseStamped, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle

from pyassimp import pyassimp

# This is going to have more functionality; (feel free to add some!)
# This class will include simple Python code for publishing messages for a planning scene

class PlanningSceneInterface:
    """ Simple interface to making updates to a planning scene """

    def __init__(self):
        self._pub_co = rospy.Publisher('/collision_object', CollisionObject)
        self._pub_aco = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)

    def make_sphere(self, name, pose, radius):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [radius]
        co.primitives = [sphere]
        co.primitive_poses = [pose.pose]
        return co

    def add_sphere(self, name, pose, radius = 1):
        """
        Add a sphere to the planning scene 
        """
        self._pub_co.publish(self.make_sphere(name, pose, radius))
        rospy.sleep(1.0)

    def make_box(self, name, pose, size):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]
        return co
    
    def make_mesh(self, name, pose, filename):
        co = CollisionObject()
        scene = pyassimp.load(filename)
        if not scene.meshes:
            raise "There are no meshes in the file"
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        
        mesh = Mesh()
        for face in scene.meshes[0].faces:
            triangle = MeshTriangle()
            if len(face.indices) == 3:
                triangle.vertex_indices = [face.indices[0], face.indices[1], face.indices[2]]
            mesh.triangles.append(triangle)
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0]
            point.y = vertex[1]
            point.z = vertex[2]
            mesh.vertices.append(point)
        co.meshes = [mesh]
        co.mesh_poses = [pose.pose]
        pyassimp.release(scene)
        return co
    
    def add_mesh(self, name, pose, filename):
        """
        Add a mesh to the planning scene
        """
        self._pub_co.publish(self.make_mesh(name, pose, filename))
        rospy.sleep(1.0)

    def add_box(self, name, pose, size = (1, 1, 1)):
        """
        Add a box to the planning scene 
        """
        self._pub_co.publish(self.make_box(name, pose, size))
        rospy.sleep(1.0)

    def add_plane(self, name, pose, normal = (0, 0, 1), offset = 0):
        """ Add a plane to the planning scene """
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        p = Plane()
        p.coef = list(normal)
        p.coef.append(offset)
        co.planes = [p]
        co.plane_poses = [pose.pose]
        self._pub_co.publish(co)
        rospy.sleep(1.0)
        
    def attach_mesh(self, link, name, pose, filename, touch_links = []):
        aco = AttachedCollisionObject()
        aco.object = self.make_mesh(name, pose, filename)
        aco.link_name = link
        aco.touch_links = [link]
        if len(touch_links) > 0:
            aco.touch_links = touch_links
        self._pub_aco.publish(aco)
        rospy.sleep(1.0)

    def attach_box(self, link, name, pose, size = (1, 1, 1), touch_links = []):
        aco = AttachedCollisionObject()
        aco.object = self.make_box(name, pose, size)
        aco.link_name = link
        aco.touch_links = [link]
        if len(touch_links) > 0:
            aco.touch_links = touch_links
        self._pub_aco.publish(aco)
        rospy.sleep(1.0)

    def remove_world_object(self, name, world_link = '/odom_combined'):
        """
        Remove object from planning scene         
        """
        co = CollisionObject()
        co.header.frame_id = world_link
        co.operation = CollisionObject.REMOVE
        co.id = name
        self._pub_co.publish(co)
        rospy.sleep(1.0)

    def remove_attached_object(self, link, name = ''):
        """
        Remove object from planning scene         
        """
        aco = AttachedCollisionObject()
        aco.object.operation = CollisionObject.REMOVE
        aco.link_name = link
        aco.object.id = name
        self._pub_aco.publish(aco)
        rospy.sleep(1.0)
