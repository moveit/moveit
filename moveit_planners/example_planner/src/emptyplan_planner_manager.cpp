/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example planning plugin template
*/

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <class_loader/class_loader.hpp>

namespace emptyplan_interface
{

class EmptyPlanPlannerManager : public planning_interface::PlannerManager
{
public:
  EmptyPlanPlannerManager() : planning_interface::PlannerManager(), nh_("~")
  {
  }

  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override
  {
    if (!ns.empty())
      nh_ = ros::NodeHandle(ns);
    //emptyplan_interface_.reset(new EmptyPlanInterface(model, nh_));
    std::string emptyplan_ns = ns.empty() ? "emptyplan" : ns + "/emptyplan";
    //config_settings_ = emptyplan_interface_->getPlannerConfigurations();
    return true;
  }

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  std::string getDescription() const override
  {
    return "EmptyPlan";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    // const planning_interface::PlannerConfigurationMap& pconfig = emptyplan_interface_->getPlannerConfigurations();
    // algs.clear();
    // algs.reserve(pconfig.size());
    // for (const std::pair<const std::string, planning_interface::PlannerConfigurationSettings>& config : pconfig)
    //   algs.push_back(config.first);


    algs.clear();
    algs.push_back("trajopt");
  }

  // void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig) override
  // {
  //   // this call can add a few more configs than we pass in (adds defaults)
  //   emptyplan_interface_->setPlannerConfigurations(pconfig);
  //   // so we read the configs instead of just setting pconfig
  //   PlannerManager::setPlannerConfigurations(emptyplan_interface_->getPlannerConfigurations());
  // }

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const override
  {
    // TODO(ommmid): this is the key thing to implement
    return planning_interface::PlanningContextPtr();
    //return emptyplan_interface_->getPlanningContext(planning_scene, req, error_code);
  }

private:


  ros::NodeHandle nh_;
  // std::unique_ptr<EmptyPlanInterface> emptyplan_interface_;
  // std::unique_ptr<std::thread> pub_valid_states_thread_;
  // bool display_random_valid_states_{ false };
  // ros::Publisher pub_markers_;
  // ros::Publisher pub_valid_states_;
  // ros::Publisher pub_valid_traj_;
  // std::string planner_data_link_name_;
  // std::shared_ptr<emptyplan::msg::OutputHandler> output_handler_;
};

}  // namespace emptyplan_interface

CLASS_LOADER_REGISTER_CLASS(emptyplan_interface::EmptyPlanPlannerManager, planning_interface::PlannerManager);
