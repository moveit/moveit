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
  // if there are no adapters, run the planner directly 
  if (adapters_.empty())
    return planner->solve(planning_scene, req, res);
  else
  {
    // if there are adapters, construct a function pointer for each, in order,
    // so that in the end we have a nested sequence of function pointers that call the adapters in the correct order.
    PlannerFn fn = boost::bind(&callAdapter1, adapters_.back().get(), planner, _1, _2, _3);
    for (int i = adapters_.size() - 2 ; i >= 0 ; --i)
      fn = boost::bind(&callAdapter2, adapters_[i].get(), fn, _1, _2, _3);
    return fn(planning_scene, req, res);
  }
}
