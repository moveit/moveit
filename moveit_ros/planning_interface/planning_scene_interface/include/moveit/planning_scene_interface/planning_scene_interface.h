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

#ifndef PLANNING_SCENE_INTERFACE_
#define PLANNING_SCENE_INTERFACE__

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

namespace bp = boost::python;

namespace planning_scene_interface
{

class PlanningSceneInterface : protected moveit_py_bindings_tools::ROScppInitializer
{

public:

  PlanningSceneInterface();


  void addSimpleObjectToPlanningScene(std::string id, std::string frame_id, int type, double pos_x,
                                      double pos_y, double pos_z, double or_x, double or_y, double or_z,
                                      double or_w, std::vector<double> dimensions);

  void addSimpleObjectToPlanningScenePython(std::string id, std::string frame_id, int type, double pos_x,
                                            double pos_y, double pos_z, double or_x, double or_y, double or_z,
                                            double or_w, bp::list &values);

  void removeSimpleObjectFromPlanningScene(std::string id, std::string frame_id);
  
  void attachSimpleCollisionObject(std::string id, std::string frame_id, int type, std::string link_name,
                                   std::vector<std::string> touch_links, double pos_x, double pos_y,
                                   double pos_z, double or_x, double or_y, double or_z, double or_w,
                                   std::vector<double> dimensions);

  void attachSimpleCollisionObjectPython(std::string id, std::string frame_id, int type, std::string link_name,
                                         bp::list &touch_links, double pos_x, double pos_y, double pos_z,
                                         double or_x, double or_y, double or_z, double or_w, bp::list &values);

  void removeSimpleAttachedObject(std::string id, std::string frame_id, std::string link_name);

private:

  std::vector<double> doubleFromList(bp::list &values);

  std::vector<std::string> stringFromList(bp::list &values);

  ros::Publisher collision_object_pub_, attached_collision_object_pub_;

  ros::NodeHandle nh_;
};

}

#endif 
