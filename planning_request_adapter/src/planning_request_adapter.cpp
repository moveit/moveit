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

#include <planning_request_adapter/planning_request_adapter.h>
#include <planning_models/conversions.h>
#include <boost/bind.hpp>
#include <ros/console.h>

namespace planning_request_adapter
{
static bool callPlannerInterfaceSolve(const planning_interface::Planner *planner,
                                      const planning_scene::PlanningSceneConstPtr& planning_scene,
                                      const moveit_msgs::GetMotionPlan::Request &req, 
                                      moveit_msgs::GetMotionPlan::Response &res)
{
  return planner->solve(planning_scene, req, res);
}
}

bool planning_request_adapter::PlanningRequestAdapter::adaptAndPlan(const planning_interface::PlannerPtr &planner,
                                                                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                    const moveit_msgs::GetMotionPlan::Request &req, 
                                                                    moveit_msgs::GetMotionPlan::Response &res) const
{
  return adaptAndPlan(boost::bind(&callPlannerInterfaceSolve, planner.get(), _1, _2, _3), planning_scene, req, res);
}

void planning_request_adapter::PlanningRequestAdapter::addPrefixState(const planning_models::KinematicState &prefix, moveit_msgs::GetMotionPlan::Response &res,
                                                                      double max_dt_offset, const planning_models::TransformsConstPtr &transforms) const
{
  planning_models::kinematicStateToRobotState(prefix, res.trajectory_start);
  
  // heuristically decide a duration offset for the trajectory (induced by the additional point added as a prefix to the computed trajectory)
  double d = max_dt_offset;
  if (res.trajectory.joint_trajectory.points.size() > 1 || res.trajectory.multi_dof_joint_trajectory.points.size() > 1)
  {
    double temp = (res.trajectory.joint_trajectory.points.size() > res.trajectory.multi_dof_joint_trajectory.points.size()) ? 
      res.trajectory.joint_trajectory.points.back().time_from_start.toSec() / (double)(res.trajectory.joint_trajectory.points.size() - 1) :
      res.trajectory.multi_dof_joint_trajectory.points.back().time_from_start.toSec() / (double)(res.trajectory.multi_dof_joint_trajectory.points.size() - 1);
    if (temp < d)
      d = temp;
  }
  ros::Duration dt(d);
  
  if (!res.trajectory.joint_trajectory.points.empty() && !res.trajectory.joint_trajectory.joint_names.empty())
  {
    trajectory_msgs::JointTrajectoryPoint new_start = res.trajectory.joint_trajectory.points.front();
    std::vector<double> vals;
    for (std::size_t i = 0 ; i < res.trajectory.joint_trajectory.joint_names.size() ; ++i)
    {
      const planning_models::KinematicState::JointState *js = prefix.getJointState(res.trajectory.joint_trajectory.joint_names[i]);
      if (!js)
        break;
      if (js->getVariableValues().size() != 1)
      {
        ROS_ERROR("Unexpected number of joint values. Got %u when 1 should have been found", (unsigned int)js->getVariableValues().size());
        break;
      }
      vals.push_back(js->getVariableValues()[0]);
    }
    
    if (vals.size() == new_start.positions.size())
    {
      new_start.positions = vals;
      
      //insert extra point at the start of trajectory
      res.trajectory.joint_trajectory.points.insert(res.trajectory.joint_trajectory.points.begin(), new_start);
      
      // add duration offset for the trajectory
      for (std::size_t j = 1 ; j < res.trajectory.joint_trajectory.points.size() ; ++j)
        res.trajectory.joint_trajectory.points[j].time_from_start += dt;
    }
    else
      ROS_ERROR("The number of joints in the solution reported by the planner does not match with the known set of joints");
  }
  
  if (!res.trajectory.multi_dof_joint_trajectory.points.empty() && !res.trajectory.multi_dof_joint_trajectory.joint_names.empty())
  {
    moveit_msgs::MultiDOFJointTrajectoryPoint new_start = res.trajectory.multi_dof_joint_trajectory.points.front();
    std::vector<Eigen::Affine3d> poses;
    for (std::size_t i = 0 ; i < res.trajectory.multi_dof_joint_trajectory.joint_names.size() ; ++i)
    {
      const planning_models::KinematicState::JointState *js = prefix.getJointState(res.trajectory.multi_dof_joint_trajectory.joint_names[i]);
      if (!js)
        break;
      if (res.trajectory.multi_dof_joint_trajectory.child_frame_ids.size() <= i ||
          js->getJointModel()->getChildLinkModel()->getName() != res.trajectory.multi_dof_joint_trajectory.child_frame_ids[i])
      {
        ROS_ERROR("Unmatched multi-dof joint: '%s'", js->getJointModel()->getChildLinkModel()->getName().c_str());
        break;
      }
      const Eigen::Affine3d& t = js->getVariableTransform();
      if (res.trajectory.multi_dof_joint_trajectory.frame_ids.size() >= i &&
          res.trajectory.multi_dof_joint_trajectory.frame_ids[i] != prefix.getKinematicModel()->getModelFrame())
      {
        if (transforms)
          poses.push_back(transforms->getTransform(prefix, res.trajectory.multi_dof_joint_trajectory.frame_ids[i]) * t);
        else
        {
          ROS_ERROR("Transform to frame '%s' is not known", res.trajectory.multi_dof_joint_trajectory.frame_ids[i].c_str());
          break;
        }
      }
      else
        poses.push_back(t);
    }
    if (poses.size() == new_start.poses.size())
    {
      for (std::size_t i = 0 ; i < poses.size() ; ++i)
        planning_models::msgFromPose(poses[i], new_start.poses[i]);
      
      //insert extra point at the start of trajectory
      res.trajectory.multi_dof_joint_trajectory.points.insert(res.trajectory.multi_dof_joint_trajectory.points.begin(), new_start);
      
      // add duration offset for the trajectory
      for (std::size_t j = 1 ; j < res.trajectory.joint_trajectory.points.size() ; ++j)
        res.trajectory.multi_dof_joint_trajectory.points[j].time_from_start += dt;
    }      
    else
      ROS_ERROR("The number of mulit-dof joints in the solution reported by the planner does not match with the known set of joints");
  }
}

namespace planning_request_adapter
{

// boost bind is not happy with overloading, so we add intermediate function objects

static bool callAdapter1(const PlanningRequestAdapter *adapter,
                         const planning_interface::PlannerPtr &planner,
                         const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const moveit_msgs::GetMotionPlan::Request &req, 
                         moveit_msgs::GetMotionPlan::Response &res)
{
  return adapter->adaptAndPlan(planner, planning_scene, req, res);
}

static bool callAdapter2(const PlanningRequestAdapter *adapter,
                         const PlannerFn &planner,
                         const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const moveit_msgs::GetMotionPlan::Request &req, 
                         moveit_msgs::GetMotionPlan::Response &res)
{
  return adapter->adaptAndPlan(planner, planning_scene, req, res);
}

}

bool planning_request_adapter::PlanningRequestAdapterChain::adaptAndPlan(const planning_interface::PlannerPtr &planner,
                                                                         const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                         const moveit_msgs::GetMotionPlan::Request &req, 
                                                                         moveit_msgs::GetMotionPlan::Response &res) const
{
  if (adapters_.empty())
    return planner->solve(planning_scene, req, res);
  else
  {
    PlannerFn fn = boost::bind(&callAdapter1, adapters_.back().get(), planner, _1, _2, _3);
    for (int i = adapters_.size() - 2 ; i >= 0 ; --i)
      fn = boost::bind(&callAdapter2, adapters_[i].get(), fn, _1, _2, _3);
    return fn(planning_scene, req, res);
  }
}
