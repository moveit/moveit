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
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

#include <boost/shared_ptr.hpp>
#include <Python.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

namespace bp = boost::python;

namespace move_group_interface
{

class ROSInitializer
{
public:
  
  ROSInitializer(void)
  {
    // ensure we do not accidentally initialize ROS multiple times per process
    static boost::mutex lock;
    boost::mutex::scoped_lock slock(lock);
    
    // once per process, we start a spinner
    static bool once = true;
    if (once)
    {
      once = false;
      static boost::shared_ptr<InitProxy> proxy;
      
      // if ROS (cpp) is not initialized, we initialize it
      if (!ros::isInitialized())
        proxy.reset(new InitProxy());
      
      static ros::AsyncSpinner spinner(1);
      spinner.start();
    }
  }
  
private:
  
  struct InitProxy
  {
    InitProxy(void)
    {
      char **fake_argv = new char*[1];
      fake_argv[0] = strdup("move_group_python_wrappers");
      int fake_argc = 1;
      ros::init(fake_argc, fake_argv, "move_group_python_wrappers", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
      delete[] fake_argv[0];
      delete[] fake_argv;
    }
    
    ~InitProxy(void)
    { 
      if (ros::isInitialized() && !ros::isShuttingDown())
        ros::shutdown();
    }
  };
};

class MoveGroupWrapper : protected ROSInitializer,
                         public MoveGroup
{
public:

  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  MoveGroupWrapper(const std::string &group_name) : ROSInitializer(),
                                                    MoveGroup(group_name)
  {
  }
  
  void setJointValueTargetPerJointPythonList(const std::string &joint, bp::list &values)
  {
    setJointValueTarget(joint, doubleFromList(values));
  }
  
  void setJointValueTargetPythonList(bp::list &values)
  {
    setJointValueTarget(doubleFromList(values));
  }

  void setJointValueTargetPythonDict(bp::dict &values)
  {
    bp::list k = values.keys(); 
    int l = bp::len(k);
    std::map<std::string, double> v;
    for (int i = 0; i < l ; ++i)
      v[bp::extract<std::string>(k[i])] = bp::extract<double>(values[k[i]]);
    setJointValueTarget(v);
  }
  
  void rememberJointValuesFromPythonList(const std::string &string, bp::list &values)
  {
    rememberJointValues(string, doubleFromList(values));
  }

  bp::list getJointsList(void)
  {
    return listFromString(getJoints());
  }

  bp::list getCurrentJointValuesList(void)
  {
    return listFromDouble(getCurrentJointValues());
  }
  
  bp::list getRandomJointValuesList(void)
  {
    return listFromDouble(getRandomJointValues());
  }
  
  bp::dict getRememberedJointValuesPython(void) const
  {
    const std::map<std::string, std::vector<double> > &rv = getRememberedJointValues();
    bp::dict d;
    for (std::map<std::string, std::vector<double> >::const_iterator it = rv.begin() ; it != rv.end() ; ++it)
      d[it->first] = listFromDouble(it->second);
    return d;
  }
  
  bp::list getCurrentPosePython(void)
  {
    Eigen::Affine3d pose = getCurrentPose();
    std::vector<double> v(6);
    v[0] = pose.translation().x();
    v[1] = pose.translation().y();
    v[2] = pose.translation().z();
    
    Eigen::Vector3d r = pose.rotation().eulerAngles(0, 1, 2); // XYZ
    v[3] = r(0);
    v[4] = r(1);
    v[5] = r(2);
    return listFromDouble(v);
  }

  void setPoseTargetPython(bp::list &pose)
  {
    std::vector<double> v = doubleFromList(pose);
    if (v.size() != 6)
      ROS_ERROR("Pose description expected to consist of 6 values");
    else
    {
      setPositionTarget(v[0], v[1], v[2]);
      setOrientationTarget(v[3], v[4], v[5]);
    }
  }
  
  const char* getEndEffectorLinkCStr(void) const
  {
    return getEndEffectorLink().c_str();
  }  
  
  const char* getPoseReferenceFrameCStr(void) const
  {
    return getPoseReferenceFrame().c_str();
  }
  
  const char* getNameCStr(void) const
  {
    return getName().c_str();
  }
  
private:

  std::vector<double> doubleFromList(bp::list &values) const
  {   
    int l = bp::len(values);
    std::vector<double> v(l);
    for (int i = 0; i < l ; ++i)
      v[i] = bp::extract<double>(values[i]);
    return v;
  }

  template<typename T>
  bp::list listFromType(const std::vector<T>& v) const
  {
    bp::list l;
    for (std::size_t i = 0 ; i < v.size() ; ++i)
      l.append(v[i]);
    return l;
  }
  
  bp::list listFromDouble(const std::vector<double>& v) const
  {
    return listFromType<double>(v);
  }

  bp::list listFromString(const std::vector<std::string>& v) const
  {
    return listFromType<std::string>(v);
  }

};  
  
void wrap_move_group_interface()
{
  bp::class_<MoveGroupWrapper> MoveGroupClass("MoveGroup", bp::init<std::string>());

  MoveGroupClass.def("async_move", &MoveGroupWrapper::asyncMove);
  MoveGroupClass.def("move", &MoveGroupWrapper::move);
  MoveGroupClass.def("plan", &MoveGroupWrapper::plan);
  MoveGroupClass.def("stop", &MoveGroupWrapper::stop);

  MoveGroupClass.def("get_name", &MoveGroupWrapper::getNameCStr);
  MoveGroupClass.def("get_joints", &MoveGroupWrapper::getJointsList);
  MoveGroupClass.def("allow_looking", &MoveGroupWrapper::allowLooking);
  
  MoveGroupClass.def("set_pose_reference_frame", &MoveGroupWrapper::setPoseReferenceFrame);
  
  MoveGroupClass.def("set_pose_reference_frame", &MoveGroupWrapper::setPoseReferenceFrame);
  MoveGroupClass.def("set_end_effector_link", &MoveGroupWrapper::setEndEffectorLink);
  MoveGroupClass.def("get_end_effector_link", &MoveGroupWrapper::getEndEffectorLinkCStr);
  MoveGroupClass.def("get_pose_reference_frame", &MoveGroupWrapper::getPoseReferenceFrameCStr);
  
  void (MoveGroupWrapper::*setPoseTarget_1)(const geometry_msgs::PoseStamped &) = &MoveGroupWrapper::setPoseTarget;
  MoveGroupClass.def("set_pose_target", setPoseTarget_1);

  void (MoveGroupWrapper::*setPoseTarget_2)(const geometry_msgs::Pose &) = &MoveGroupWrapper::setPoseTarget;
  MoveGroupClass.def("set_pose_target", setPoseTarget_2);

  MoveGroupClass.def("set_position_target", &MoveGroupWrapper::setPositionTarget);
  MoveGroupClass.def("set_orientation_target", &MoveGroupWrapper::setOrientationTarget);
  MoveGroupClass.def("set_pose_target", &MoveGroupWrapper::setPoseTargetPython);

  MoveGroupClass.def("get_current_pose", &MoveGroupWrapper::getCurrentPosePython);

  MoveGroupClass.def("set_joint_value_target", &MoveGroupWrapper::setJointValueTargetPythonList);
  MoveGroupClass.def("set_joint_value_target", &MoveGroupWrapper::setJointValueTargetPythonDict);
  MoveGroupClass.def("set_joint_value_target", &MoveGroupWrapper::setJointValueTargetPerJointPythonList);
  void (MoveGroupWrapper::*setJointValueTarget_4)(const std::string&, double) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_4);

  void (MoveGroupWrapper::*setJointValueTarget_5)(const sensor_msgs::JointState &) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_5);

  MoveGroupClass.def("set_named_target", &MoveGroupWrapper::setNamedTarget); 
  MoveGroupClass.def("set_random_target", &MoveGroupWrapper::setRandomTarget); 

  void (MoveGroupWrapper::*rememberJointValues_2)(const std::string&) = &MoveGroupWrapper::rememberJointValues;
  MoveGroupClass.def("remember_joint_values", rememberJointValues_2);
  
  MoveGroupClass.def("remember_joint_values",  &MoveGroupWrapper::rememberJointValuesFromPythonList);

  MoveGroupClass.def("get_current_joint_values",  &MoveGroupWrapper::getCurrentJointValuesList);
  MoveGroupClass.def("get_random_joint_values",  &MoveGroupWrapper::getRandomJointValuesList);
  MoveGroupClass.def("get_remembered_joint_values",  &MoveGroupWrapper::getRememberedJointValuesPython);

  MoveGroupClass.def("forget_joint_values", &MoveGroupWrapper::forgetJointValues); 

  MoveGroupClass.def("get_goal_tolerance", &MoveGroupWrapper::getGoalTolerance); 
  MoveGroupClass.def("set_goal_tolerance", &MoveGroupWrapper::setGoalTolerance); 
}

}

BOOST_PYTHON_MODULE(_move_group)
{
  using namespace move_group_interface;
  wrap_move_group_interface();
}
