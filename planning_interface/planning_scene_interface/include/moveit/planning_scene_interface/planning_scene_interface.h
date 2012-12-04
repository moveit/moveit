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

#ifndef MOVEIT_PLANNING_INTERFACE_PLANNING_SCENE_INTERFACE_
#define MOVEIT_PLANNING_INTERFACE_PLANNING_SCENE_INTERFACE__

#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <boost/function.hpp>
#include <boost/python.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

#include <boost/shared_ptr.hpp>
#include <Python.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

namespace planning_scene_interface
{

class PlanningSceneInterface : protected moveit_py_bindings_tools::ROScppInitializer
{
  
public:
  
  PlanningSceneInterface();
  
  
  void addSimpleObjectToPlanningScene(const std::string &id, const std::string &frame_id, int type, 
                                      const std::vector<double> &position, const std::vector<double> &orientation,
                                      const std::vector<double> &dimensions);
  
  void addSimpleObjectToPlanningScenePython(const std::string &id, const std::string &frame_id, int type,
                                            boost::python::list &position, boost::python::list &orientation, boost::python::list &dimensions);
  
  void removeSimpleObjectFromPlanningScene(const std::string &id, const std::string &frame_id);
  
  void attachSimpleCollisionObject(const std::string &id, const std::string &frame_id, int type,
                                   const std::string &link_name, const std::vector<std::string> &touch_links,
                                   const std::vector<double> &position, const std::vector<double> &orientation, 
                                   const std::vector<double> &dimensions);
  
  void attachSimpleCollisionObjectPython(const std::string &id, const std::string &frame_id, int type, 
                                         const std::string &link_name, boost::python::list &touch_links, boost::python::list &position,
                                         boost::python::list &orientation, boost::python::list &dimensions);
  
  void removeSimpleAttachedObject(const std::string &id, const std::string &frame_id, const std::string &link_name);
  
  void addSphere(const std::string &id, const std::string &frame_id, boost::python::list &position,
                 boost::python::list &orientation, double radius);
  
  void addCylinder(const std::string &id, const std::string &frame_id, boost::python::list &position,
                   boost::python::list &orientation, double height, double radius);
  
  void addBox(const std::string &id, const std::string &frame_id, boost::python::list &position, 
              boost::python::list &orientation, double length, double width, double height);
  
  void addCone(const std::string &id, const std::string &frame_id, boost::python::list &position,
               boost::python::list &orientation, double height, double radius);
  
  void attachSphere(const std::string &id, const std::string &frame_id, const std::string &link_name,
                    boost::python::list &touch_links, boost::python::list &position, boost::python::list &orientation, double radius);
  
  void attachCylinder(const std::string &id, const std::string &frame_id, const std::string &link_name,
                      boost::python::list &touch_links, boost::python::list &position, boost::python::list &orientation, double height, double radius);
  
  void attachBox(const std::string &id, const std::string &frame_id, const std::string &link_name,
                 boost::python::list &touch_links, boost::python::list &position, boost::python::list &orientation, double length, double width, double height);
  
  void attachCone(const std::string &id, const std::string &frame_id, const std::string &link_name,
                  boost::python::list &touch_links, boost::python::list &position, boost::python::list &orientation, double height, double radius);
  
private:
  
  ros::Publisher collision_object_pub_, attached_collision_object_pub_;  
  ros::NodeHandle nh_;
};

}

#endif 
