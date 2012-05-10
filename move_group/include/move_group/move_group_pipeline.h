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

#include <planning_interface/planning_interface.h>
#include <planning_request_adapter/planning_request_adapter.h>
#include <trajectory_processing/iterative_smoother.h>
#include <pluginlib/class_loader.h>
#include <planning_scene_monitor/planning_scene_monitor.h>

namespace move_group {

class MoveGroupPipeline {

public:
  
  MoveGroupPipeline(const planning_scene_monitor::PlanningSceneMonitorConstPtr& psm);

  bool generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const moveit_msgs::GetMotionPlan::Request& req,
                    moveit_msgs::GetMotionPlan::Response& res) const;

  const std::string& getPlanningPluginName() {
    return planning_plugin_name_;
  }

protected:

  planning_scene_monitor::PlanningSceneMonitorConstPtr psm_;

  std::string planning_plugin_name_;
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::Planner> > planner_plugin_loader_;
  planning_interface::PlannerPtr planner_instance_;

  trajectory_processing::IterativeParabolicSmoother smoother_;

  boost::scoped_ptr<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter> > adapter_plugin_loader_;
  boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> adapter_chain_;

};

}
