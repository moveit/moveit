/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Zachary Kingston */

#ifndef MOVEIT_OMPL_INTERFACE_OMPL_PLANNER_MANAGER_
#define MOVEIT_OMPL_INTERFACE_OMPL_PLANNER_MANAGER_

namespace ompl_interface
{
using namespace moveit_planners_ompl;

class OMPLPlannerManager : public planning_interface::PlannerManager
{
public:
  OMPLPlannerManager();

  virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns);

  virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const;

  virtual std::string getDescription() const;

  virtual void getPlanningAlgorithms(std::vector<std::string>& algs) const;

  virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig);

  virtual planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req, moveit_msgs::MoveItErrorCodes& error_code) const;

  /** @brief Get the configurations for the planners that are already loaded
      @param pconfig Configurations for the different planners */
  const planning_interface::PlannerConfigurationMap& getPlannerConfigurations() const
  {
    return context_manager_->getPlannerConfigurations();
  }

  const std::unique_ptr<PlanningContextManager>& getPlanningContextManager() const
  {
    return context_manager_;
  }

  std::unique_ptr<PlanningContextManager>& getPlanningContextManager()
  {
    return context_manager_;
  }

  constraint_samplers::ConstraintSamplerManager& getConstraintSamplerManager()
  {
    return *constraint_sampler_manager_;
  }

  const constraint_samplers::ConstraintSamplerManager& getConstraintSamplerManager() const
  {
    return *constraint_sampler_manager_;
  }

private:
  void dynamicReconfigureCallback(OMPLDynamicReconfigureConfig& config, uint32_t level)
  {
    config_ = config;
  }

  /** @brief Load planner configurations for specified group into planner_config */
  bool loadPlannerConfiguration(const std::string& group_name, const std::string& planner_id,
                                const std::map<std::string, std::string>& group_params,
                                planning_interface::PlannerConfigurationSettings& planner_config);

  /** @brief Configure the planners*/
  void loadPlannerConfigurations();

  /** @brief Load the additional plugins for sampling constraints */
  void loadConstraintSamplers();

  ros::NodeHandle nh_;
  std::unique_ptr<dynamic_reconfigure::Server<OMPLDynamicReconfigureConfig> > dynamic_reconfigure_server_;

  robot_model::RobotModelConstPtr kmodel_;
  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;
  constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;

  std::unique_ptr<PlanningContextManager> context_manager_;
  OMPLDynamicReconfigureConfig config_;
  std::shared_ptr<ompl::msg::OutputHandler> output_handler_;
};

}  // namespace ompl_interface

#endif
