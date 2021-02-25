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
from rosgraph.names import ns_join
from . import conversions

from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_ros_planning_interface import _moveit_planning_scene_interface
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from .exception import MoveItCommanderException
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print("Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info")


class PlanningSceneInterface(object):
    """ Simple interface to making updates to a planning scene """

    def __init__(self, ns='', synchronous=False, service_timeout=5.0):
        """ Create a planning scene interface; it uses both C++ wrapped methods and scene manipulation topics. """
        self._psi = _moveit_planning_scene_interface.PlanningSceneInterface(ns)

        self._pub_co = rospy.Publisher(ns_join(ns, 'collision_object'), CollisionObject, queue_size=100)
        self._pub_aco = rospy.Publisher(ns_join(ns, 'attached_collision_object'), AttachedCollisionObject, queue_size=100)
        self.__synchronous = synchronous
        if self.__synchronous:
            self._apply_planning_scene_diff = rospy.ServiceProxy(ns_join(ns, 'apply_planning_scene'), ApplyPlanningScene)
            self._apply_planning_scene_diff.wait_for_service(service_timeout)

    def __submit(self, collision_object):
        if self.__synchronous:
            self._apply_planning_scene_diff.call(self.__make_planning_scene_diff_req(collision_object))
        else:
            if isinstance(collision_object, AttachedCollisionObject):
                self._pub_aco.publish(collision_object)
            else:
                self._pub_co.publish(collision_object)

    def remove_world_object(self, name=None):
        """
        Remove an object from planning scene, or all if no name is provided
        """
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        if name is not None:
            co.id = name
        self.__submit(co)

    def remove_attached_object(self, link, name=None):
        """
        Remove an attached object from planning scene, or all objects attached to this link if no name is provided
        """
        aco = AttachedCollisionObject()
        aco.object.operation = CollisionObject.REMOVE
        aco.link_name = link
        if name is not None:
            aco.object.id = name
        self.__submit(aco)

    def get_known_object_names(self, with_type=False):
        """
        Get the names of all known objects in the world. If with_type is set to true, only return objects that have a known type.
        """
        return self._psi.get_known_object_names(with_type)

    def get_known_object_names_in_roi(self, minx, miny, minz, maxx, maxy, maxz, with_type=False):
        """
        Get the names of known objects in the world that are located within a bounding region (specified in the frame reported by
        get_planning_frame()). If with_type is set to true, only return objects that have a known type.
        """
        return self._psi.get_known_object_names_in_roi(minx, miny, minz, maxx, maxy, maxz, with_type)

    def get_object_poses(self, object_ids):
        """
        Get the poses from the objects identified by the given object ids list.
        """
        ser_ops = self._psi.get_object_poses(object_ids)
        ops = dict()
        for key in ser_ops:
            msg = Pose()
            conversions.msg_from_string(msg, ser_ops[key])
            ops[key] = msg
        return ops

    def get_objects(self, object_ids=[]):
        """
        Get the objects identified by the given object ids list. If no ids are provided, return all the known objects.
        """
        ser_objs = self._psi.get_objects(object_ids)
        objs = dict()
        for key in ser_objs:
            msg = CollisionObject()
            conversions.msg_from_string(msg, ser_objs[key])
            objs[key] = msg
        return objs

    def get_attached_objects(self, object_ids=[]):
        """
        Get the attached objects identified by the given object ids list. If no ids are provided, return all the attached objects.
        """
        ser_aobjs = self._psi.get_attached_objects(object_ids)
        aobjs = dict()
        for key in ser_aobjs:
            msg = AttachedCollisionObject()
            conversions.msg_from_string(msg, ser_aobjs[key])
            aobjs[key] = msg
        return aobjs

    @staticmethod
    def __make_existing(name):
        """
        Create an empty Collision Object, used when the object already exists
        """
        co = CollisionObject()
        co.id = name
        return co

    @staticmethod
    def __make_box(name, pose, size):
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

    @staticmethod
    def __make_mesh(name, pose, filename, scale=(1, 1, 1)):
        co = CollisionObject()
        if pyassimp is False:
            raise MoveItCommanderException(
                "Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt")
        scene = pyassimp.load(filename)
        if not scene.meshes or len(scene.meshes) == 0:
            raise MoveItCommanderException("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise MoveItCommanderException("There are no faces in the mesh")
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header

        mesh = Mesh()
        first_face = scene.meshes[0].faces[0]
        if hasattr(first_face, '__len__'):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face[0], face[1], face[2]]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, 'indices'):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face.indices[0],
                                               face.indices[1],
                                               face.indices[2]]
                    mesh.triangles.append(triangle)
        else:
            raise MoveItCommanderException("Unable to build triangles from mesh due to mesh object structure")
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
        co.meshes = [mesh]
        co.mesh_poses = [pose.pose]
        pyassimp.release(scene)
        return co

    @staticmethod
    def __make_sphere(name, pose, radius=1):
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

    @staticmethod
    def __make_cylinder(name, pose, height, radius):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]
        co.primitives = [cylinder]
        co.primitive_poses = [pose.pose]
        return co

    @staticmethod
    def __make_plane(name, pose, normal=(0, 0, 1), offset=0):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        p = Plane()
        p.coef = list(normal)
        p.coef.append(offset)
        co.planes = [p]
        co.plane_poses = [pose.pose]
        return co

    @staticmethod
    def __make_planning_scene_diff_req(collision_object):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if isinstance(collision_object, AttachedCollisionObject):
            scene.robot_state.attached_collision_objects = [collision_object]
        else:
            scene.world.collision_objects = [collision_object]
        planning_scene_diff_req = ApplyPlanningSceneRequest()
        planning_scene_diff_req.scene = scene
        return planning_scene_diff_req


def __create_add_attach_methods(shape):
    """Add methods add_xxx() and attach_xxx() to class PlanningSceneInterface for a given shape"""
    make_method = getattr(PlanningSceneInterface, '_PlanningSceneInterface__make_' + shape)

    def add(self, name, pose, *args, **kwargs):
        co = make_method(name, pose, *args, **kwargs)
        self._PlanningSceneInterface__submit(co)

    def attach(self, link, name, pose=None, *args, **kwargs):
        touch_links = kwargs.pop('touch_links', [link])
        aco = AttachedCollisionObject()
        if pose is not None:
            try:
                aco.object = make_method(name, pose, *args, **kwargs)
            except TypeError:
                # If touch_links was specified as a an unnamed, last argument, the call to make_method
                # will throw a TypeError (because the additional argument doesn't match its signature).
                # In this case, grab the last argument as touch_links and retry.
                args = list(args)  # convert args into a (mutable) list
                touch_links = args.pop()  # pop last argument as touch_links
                aco.object = make_method(name, pose, *args, **kwargs)  # retry
        else:
            aco.object = self._PlanningSceneInterface__make_existing(name)
        aco.link_name = link
        aco.touch_links = touch_links
        self._PlanningSceneInterface__submit(aco)

    # Provide the correct name and doc string
    add.__name__ = 'add_' + shape
    add.__doc__ = 'Add a {} to the planning scene'.format(shape)
    attach.__name__ = 'attach_' + shape
    attach.__doc__ = 'Attach a {} to the given robot link'.format(shape)

    # Set the methods
    setattr(PlanningSceneInterface, add.__name__, add)
    setattr(PlanningSceneInterface, attach.__name__, attach)


for shape in ['box', 'sphere', 'cylinder', 'mesh', 'plane']:
    __create_add_attach_methods(shape)
