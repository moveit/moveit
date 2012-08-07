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

/* Author: Ioan Sucan */

#include "move_group_interface/move_group.h"

#include <boost/function.hpp>
#include <boost/python.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/shared_ptr.hpp>
#include <Python.h>
#include <ros/ros.h>

namespace bp = boost::python;

namespace move_group_interface
{

class MoveGroupWrapper : public MoveGroup
{
public:
  
  MoveGroupWrapper(const std::string &group_name) : MoveGroup(group_name)
  {
  }
  
  const char* getEndEffectorLinkCStr(void) const
  {
    return getEndEffectorLink().c_str();
  }  

  const char* getPoseReferenceFrameCStr(void) const
  {
    return getPoseReferenceFrame().c_str();
  }
  
  bool moveSimple(void)
  {
    return move();
  }
  
};  
  
void wrap_move_group_interface()
{
  bp::class_<MoveGroupWrapper> MoveGroupClass("MoveGroup", bp::init<std::string>());

  MoveGroupClass.def("async_move", &MoveGroupWrapper::asyncMove);
  MoveGroupClass.def("move", &MoveGroupWrapper::moveSimple);
  MoveGroupClass.def("plan", &MoveGroupWrapper::plan);

  MoveGroupClass.def("set_pose_reference_frame", &MoveGroupWrapper::setPoseReferenceFrame);
  
  MoveGroupClass.def("set_pose_reference_frame", &MoveGroupWrapper::setPoseReferenceFrame);
  MoveGroupClass.def("set_end_effector_link", &MoveGroupWrapper::setEndEffectorLink);
  MoveGroupClass.def("get_end_effector_link", &MoveGroupWrapper::getEndEffectorLinkCStr);
  MoveGroupClass.def("get_pose_reference_frame", &MoveGroupWrapper::getPoseReferenceFrameCStr);
  
  void (MoveGroupWrapper::*setPoseTarget_1)(const geometry_msgs::PoseStamped &) = &MoveGroupWrapper::setPoseTarget;
  MoveGroupClass.def("set_pose_target", setPoseTarget_1);

  void (MoveGroupWrapper::*setPoseTarget_2)(const geometry_msgs::Pose &) = &MoveGroupWrapper::setPoseTarget;
  MoveGroupClass.def("set_pose_target", setPoseTarget_2);


  void (MoveGroupWrapper::*setJointValueTarget_1)(const std::vector<double> &) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_1);

  void (MoveGroupWrapper::*setJointValueTarget_2)(const std::map<std::string, double> &) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_2);

  void (MoveGroupWrapper::*setJointValueTarget_3)(const std::string&, const std::vector<double> &) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_3);

  void (MoveGroupWrapper::*setJointValueTarget_4)(const std::string&, double) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_4);

  void (MoveGroupWrapper::*setJointValueTarget_5)(const sensor_msgs::JointState &) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_5);

  MoveGroupClass.def("set_named_target", &MoveGroupWrapper::setNamedTarget); 

  void (MoveGroupWrapper::*rememberJointValues_1)(const std::string&, const std::vector<double> &) = &MoveGroupWrapper::rememberJointValues;
  MoveGroupClass.def("remember_joint_values", rememberJointValues_1);

  void (MoveGroupWrapper::*rememberJointValues_2)(const std::string&) = &MoveGroupWrapper::rememberJointValues;
  MoveGroupClass.def("remember_joint_values", rememberJointValues_2);

  MoveGroupClass.def("forget_joint_values", &MoveGroupWrapper::forgetJointValues); 
}

}

BOOST_PYTHON_MODULE(move_group)
{
  using namespace move_group_interface;
  wrap_move_group_interface();
  
  char **fake_argv = new char*[1];
  fake_argv[0] = strdup("move_group_python_wrappers");
  int fake_argc = 1;
  ros::init(fake_argc, fake_argv, "move_group_python_wrappers", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  delete[] fake_argv[0];
  delete[] fake_argv;
  
  static ros::AsyncSpinner spinner(1);
  spinner.start();
}
