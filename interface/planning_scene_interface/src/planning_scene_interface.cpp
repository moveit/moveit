/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sarah Elliott */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <moveit/py_bindings_tools/py_conversions.h>

namespace bp = boost::python;

namespace planning_scene_interface

{

  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  PlanningSceneInterface::PlanningSceneInterface() : moveit_py_bindings_tools::ROScppInitializer()
  {
    collision_object_pub_ = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    attached_collision_object_pub_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
    // This delay is so critical, otherwise the first published object may not be added in the collision_space by the environment_server
    ros::Duration(2.0).sleep();
  }


  void PlanningSceneInterface::addSphere(const std::string &id, const std::string &frame_id, bp::list &position, 
                                         bp::list &orientation, double radius)
  {
    bp::list dimensions;
    dimensions.append(radius);
    addSimpleObjectToPlanningScenePython(id, frame_id, shape_msgs::SolidPrimitive::SPHERE, position, orientation, 
                                         dimensions);
  }

  void PlanningSceneInterface::addCylinder(const std::string &id, const std::string &frame_id, bp::list  &position, 
                                           bp::list  &orientation, double height, double radius)
  {
    bp::list dimensions;
    dimensions.append(height);
    dimensions.append(radius);
    addSimpleObjectToPlanningScenePython(id, frame_id, shape_msgs::SolidPrimitive::CYLINDER, position, orientation, 
                                        dimensions);

  }

  void PlanningSceneInterface::addBox(const std::string &id, const std::string &frame_id, bp::list &position, 
                                      bp::list &orientation, double length, double width, double height)
  {
    bp::list  dimensions;
    dimensions.append(length);
    dimensions.append(width);
    dimensions.append(height);
    addSimpleObjectToPlanningScenePython(id, frame_id, shape_msgs::SolidPrimitive::BOX, position, orientation, dimensions);
  }

  void PlanningSceneInterface::addCone(const std::string &id, const std::string &frame_id, bp::list &position, 
                                       bp::list &orientation, double height, double radius)
  {
    bp::list dimensions;
    dimensions.append(height);
    dimensions.append(radius);
    addSimpleObjectToPlanningScenePython(id, frame_id, shape_msgs::SolidPrimitive::CONE, position, orientation, dimensions);
  }

  void PlanningSceneInterface::addSimpleObjectToPlanningScene(const std::string &id, const std::string &frame_id, int type,
                                                              const std::vector<double> &position, 
                                                              const std::vector<double> &orientation, 
                                                              const std::vector<double> &dimensions)
  {
    //add the cylinder into the collision space
    moveit_msgs::CollisionObject cylinder_object;
    cylinder_object.id = id;
    cylinder_object.operation = moveit_msgs::CollisionObject::ADD;
    cylinder_object.header.frame_id = frame_id;
    cylinder_object.header.stamp = ros::Time::now();
    shape_msgs::SolidPrimitive object;
    object.type = type;
    object.dimensions.resize(dimensions.size());
    for (int i=0; i < dimensions.size(); i++)
    {
      object.dimensions[i] = dimensions.at(i);
    }

    //object.dimensions = dimensions;
    cylinder_object.primitives.push_back(object);

    geometry_msgs::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.x = orientation[0];
    pose.orientation.y = orientation[1];
    pose.orientation.z = orientation[2];
    pose.orientation.w = orientation[3];
    cylinder_object.primitive_poses.push_back(pose);
    collision_object_pub_.publish(cylinder_object);

    ROS_INFO("Should have published");
  }

  void PlanningSceneInterface::addSimpleObjectToPlanningScenePython(const std::string &id, const std::string &frame_id, 
                                                                    int type, bp::list &position,
                                                                    bp::list &orientation, bp::list &dimensions)
  {
    addSimpleObjectToPlanningScene(id, frame_id, type,
                                   moveit_py_bindings_tools::doubleFromList(position),
                                   moveit_py_bindings_tools::doubleFromList(orientation), 
                                   moveit_py_bindings_tools::doubleFromList(dimensions));
  }

  void PlanningSceneInterface::removeSimpleObjectFromPlanningScene(const std::string &id, const std::string &frame_id) 
  {
    moveit_msgs::CollisionObject object_to_remove;
    object_to_remove.id = id;
    object_to_remove.operation = moveit_msgs::CollisionObject::REMOVE;
    object_to_remove.header.frame_id = frame_id;
    object_to_remove.header.stamp = ros::Time::now();

    collision_object_pub_.publish(object_to_remove);

    ROS_INFO("Should have published");
  }
  
  void PlanningSceneInterface::attachSimpleCollisionObject(const std::string &id, const std::string &frame_id, int type,  
                                   const std::string &link_name, const std::vector<std::string> &touch_links, 
                                   const std::vector<double> &position, const std::vector<double> &orientation, 
                                   const std::vector<double> &dimensions)
  {
    //add the cylinder into the collision space
    moveit_msgs::AttachedCollisionObject gripper_object;
    gripper_object.object.id = id;
    gripper_object.link_name = link_name;

    for (int i=0; i < touch_links.size(); i++)
    {
      gripper_object.touch_links.push_back(touch_links.at(i));    
    }

    gripper_object.object.operation = moveit_msgs::CollisionObject::ADD;
    gripper_object.object.header.frame_id = frame_id;
    gripper_object.object.header.stamp = ros::Time::now();
    shape_msgs::SolidPrimitive object;
    object.type = type;
    object.dimensions.resize(dimensions.size());
    for (int i=0; i < dimensions.size(); i++)
    {
      object.dimensions[i] = dimensions.at(i);
    }

    gripper_object.object.primitives.push_back(object);

    geometry_msgs::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.x = orientation[0];
    pose.orientation.y = orientation[1];
    pose.orientation.z = orientation[2];
    pose.orientation.w = orientation[3];
    gripper_object.object.primitive_poses.push_back(pose);
    attached_collision_object_pub_.publish(gripper_object);

    ROS_INFO("Should have published");
  }

  void PlanningSceneInterface::attachSphere(const std::string &id, const std::string &frame_id, const std::string &link_name,
                                         bp::list &touch_links, bp::list &position,                      
                                         bp::list &orientation, double radius)
  {
    bp::list dimensions;
    dimensions.append(radius);
    attachSimpleCollisionObjectPython(id, frame_id, shape_msgs::SolidPrimitive::SPHERE, link_name, touch_links, 
                                      position, orientation, dimensions);
  }

  void PlanningSceneInterface::attachCylinder(const std::string &id, const std::string &frame_id, const std::string &link_name,
                                         bp::list &touch_links, bp::list &position,
                                         bp::list &orientation, double height, double radius)
  {
    bp::list dimensions;
    dimensions.append(height);
    dimensions.append(radius);
    attachSimpleCollisionObjectPython(id, frame_id, shape_msgs::SolidPrimitive::CYLINDER, link_name, touch_links, 
                                      position, orientation, dimensions);
  }

  void PlanningSceneInterface::attachBox(const std::string &id, const std::string &frame_id, const std::string &link_name,
                                         bp::list &touch_links, bp::list &position,
                                         bp::list &orientation, double length, double width, double height)
  {
    bp::list dimensions;
    dimensions.append(length);
    dimensions.append(width);
    dimensions.append(height);
    attachSimpleCollisionObjectPython(id, frame_id, shape_msgs::SolidPrimitive::BOX, link_name, touch_links, 
                                      position, orientation, dimensions);
  }
 
  void PlanningSceneInterface::attachCone(const std::string &id, const std::string &frame_id, const std::string &link_name,
                                         bp::list &touch_links, bp::list &position,
                                         bp::list &orientation, double height, double radius)
  {
    bp::list dimensions;
    dimensions.append(height);
    dimensions.append(radius);
    attachSimpleCollisionObjectPython(id, frame_id, shape_msgs::SolidPrimitive::CONE, link_name, touch_links, 
                                      position, orientation, dimensions);
  }

  void PlanningSceneInterface::attachSimpleCollisionObjectPython(const std::string &id, const std::string &frame_id, 
                                                                 int type, const std::string &link_name, 
                                                                 bp::list &touch_links, bp::list &position, 
                                                                 bp::list &orientation, bp::list &dimensions)
  {
    attachSimpleCollisionObject(id, frame_id, type, link_name, 
                                moveit_py_bindings_tools::stringFromList(touch_links), 
                                moveit_py_bindings_tools::doubleFromList(position), 
                                moveit_py_bindings_tools::doubleFromList(orientation),
                                moveit_py_bindings_tools::doubleFromList(dimensions));
  }

  void PlanningSceneInterface::removeSimpleAttachedObject(const std::string &id, const std::string &frame_id, const std::string &link_name)
  {
    moveit_msgs::AttachedCollisionObject object_to_remove;
    object_to_remove.link_name = link_name;
    object_to_remove.object.id = id;
    object_to_remove.object.operation = moveit_msgs::CollisionObject::REMOVE;
    object_to_remove.object.header.frame_id = frame_id;
    object_to_remove.object.header.stamp = ros::Time::now();

    attached_collision_object_pub_.publish(object_to_remove);

    ROS_INFO("Should have published");
  }

void wrapPlanningSceneInterface()
{
  bp::class_<PlanningSceneInterface> PlanningSceneInterfaceClass("PlanningSceneInterface");
  
  PlanningSceneInterfaceClass.def("add_sphere", &PlanningSceneInterface::addSphere);
  PlanningSceneInterfaceClass.def("add_cylinder", &PlanningSceneInterface::addCylinder);
  PlanningSceneInterfaceClass.def("add_box", &PlanningSceneInterface::addBox);
  PlanningSceneInterfaceClass.def("add_cone", &PlanningSceneInterface::addCone);
  
  PlanningSceneInterfaceClass.def("remove_simple_object", 
                                  &PlanningSceneInterface::removeSimpleObjectFromPlanningScene);
  
  PlanningSceneInterfaceClass.def("attach_sphere", &PlanningSceneInterface::attachSphere);
  PlanningSceneInterfaceClass.def("attach_cylinder", &PlanningSceneInterface::attachCylinder);
  PlanningSceneInterfaceClass.def("attach_box", &PlanningSceneInterface::attachBox);
  PlanningSceneInterfaceClass.def("attach_cone", &PlanningSceneInterface::attachCone);
  
  PlanningSceneInterfaceClass.def("remove_simple_attached_object", 
                                  &PlanningSceneInterface::removeSimpleAttachedObject);
}

}

BOOST_PYTHON_MODULE(_moveit_planning_scene_interface)
{
  using namespace planning_scene_interface;
  wrapPlanningSceneInterface();  
}
