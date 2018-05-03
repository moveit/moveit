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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <boost/bind.hpp>
#include <algorithm>

// we could really use some c++11 lambda functions here :)

namespace planning_request_adapter
{
namespace
{
bool callPlannerInterfaceSolve(const planning_interface::PlannerManager* planner,
                               const planning_scene::PlanningSceneConstPtr& planning_scene,
                               const planning_interface::MotionPlanRequest& req,
                               planning_interface::MotionPlanResponse& res)
{
  planning_interface::PlanningContextPtr context = planner->getPlanningContext(planning_scene, req, res.error_code_);
  if (context)
    return context->solve(res);
  else
    return false;
}
}

bool PlanningRequestAdapter::adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                                          const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const planning_interface::MotionPlanRequest& req,
                                          planning_interface::MotionPlanResponse& res,
                                          std::vector<std::size_t>& added_path_index) const
{
  return adaptAndPlan(boost::bind(&callPlannerInterfaceSolve, planner.get(), _1, _2, _3), planning_scene, req, res,
                      added_path_index);
}

bool PlanningRequestAdapter::adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                                          const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const planning_interface::MotionPlanRequest& req,
                                          planning_interface::MotionPlanResponse& res) const
{
  std::vector<std::size_t> dummy;
  return adaptAndPlan(planner, planning_scene, req, res, dummy);
}

namespace
{
// boost bind is not happy with overloading, so we add intermediate function objects

bool callAdapter1(const PlanningRequestAdapter* adapter, const planning_interface::PlannerManagerPtr& planner,
                  const planning_scene::PlanningSceneConstPtr& planning_scene,
                  const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                  std::vector<std::size_t>& added_path_index)
{
  try
  {
    return adapter->adaptAndPlan(planner, planning_scene, req, res, added_path_index);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED("planning_request_adapter", "Exception caught executing *final* adapter '%s': %s",
                    adapter->getDescription().c_str(), ex.what());
    added_path_index.clear();
    return callPlannerInterfaceSolve(planner.get(), planning_scene, req, res);
  }
}

bool callAdapter2(const PlanningRequestAdapter* adapter, const PlanningRequestAdapter::PlannerFn& planner,
                  const planning_scene::PlanningSceneConstPtr& planning_scene,
                  const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                  std::vector<std::size_t>& added_path_index)
{
  try
  {
    return adapter->adaptAndPlan(planner, planning_scene, req, res, added_path_index);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED("planning_request_adapter", "Exception caught executing *next* adapter '%s': %s",
                    adapter->getDescription().c_str(), ex.what());
    added_path_index.clear();
    return planner(planning_scene, req, res);
  }
}
}

bool PlanningRequestAdapterChain::adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                                               const planning_scene::PlanningSceneConstPtr& planning_scene,
                                               const planning_interface::MotionPlanRequest& req,
                                               planning_interface::MotionPlanResponse& res) const
{
  std::vector<std::size_t> dummy;
  return adaptAndPlan(planner, planning_scene, req, res, dummy);
}

bool PlanningRequestAdapterChain::adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                                               const planning_scene::PlanningSceneConstPtr& planning_scene,
                                               const planning_interface::MotionPlanRequest& req,
                                               planning_interface::MotionPlanResponse& res,
                                               std::vector<std::size_t>& added_path_index) const
{
  // if there are no adapters, run the planner directly
  if (adapters_.empty())
  {
    added_path_index.clear();
    return callPlannerInterfaceSolve(planner.get(), planning_scene, req, res);
  }
  else
  {
    // the index values added by each adapter
    std::vector<std::vector<std::size_t> > added_path_index_each(adapters_.size());

    // if there are adapters, construct a function pointer for each, in order,
    // so that in the end we have a nested sequence of function pointers that call the adapters in the correct order.
    PlanningRequestAdapter::PlannerFn fn = boost::bind(&callAdapter1, adapters_.back().get(), planner, _1, _2, _3,
                                                       boost::ref(added_path_index_each.back()));
    for (int i = adapters_.size() - 2; i >= 0; --i)
      fn = boost::bind(&callAdapter2, adapters_[i].get(), fn, _1, _2, _3, boost::ref(added_path_index_each[i]));
    bool result = fn(planning_scene, req, res);
    added_path_index.clear();

    // merge the index values from each adapter
    for (std::size_t i = 0; i < added_path_index_each.size(); ++i)
      for (std::size_t j = 0; j < added_path_index_each[i].size(); ++j)
      {
        for (std::size_t k = 0; k < added_path_index.size(); ++k)
          if (added_path_index_each[i][j] <= added_path_index[k])
            added_path_index[k]++;
        added_path_index.push_back(added_path_index_each[i][j]);
      }
    std::sort(added_path_index.begin(), added_path_index.end());
    return result;
  }
}

}  // end of namespace planning_request_adapter