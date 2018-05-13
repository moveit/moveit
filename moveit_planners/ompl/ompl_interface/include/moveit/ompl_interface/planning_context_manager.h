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

#ifndef MOVEIT_OMPL_INTERFACE_PLANNING_CONTEXT_MANAGER_
#define MOVEIT_OMPL_INTERFACE_PLANNING_CONTEXT_MANAGER_

#include <pluginlib/class_loader.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space_factory.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/macros/class_forward.h>

#include <vector>
#include <string>
#include <map>

namespace ompl_interface
{
const static std::string DEFAULT_OMPL_PLANNER = "geometric::RRTConnect";
const static std::string DEFAULT_OMPL_PLUGIN = "ompl_interface/ModelBasedPlanningContext";

class PlanningContextManager
{
public:
  PlanningContextManager(const robot_model::RobotModelConstPtr& kmodel,
                         const constraint_samplers::ConstraintSamplerManagerPtr& csm);

  bool initialize();

  /** @brief Specify configurations for the planners.
      @param pconfig Configurations for the different planners */
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig);

  /** @brief Return the previously set planner configurations */
  const planning_interface::PlannerConfigurationMap& getPlannerConfigurations() const
  {
    return planner_configs_;
  }

  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return kmodel_;
  }

  // ModelBasedPlanningContextPtr getLastPlanningContext() const;

  OMPLPlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                            const planning_interface::MotionPlanRequest& req,
                                            moveit_msgs::MoveItErrorCodes& error_code) const;

protected:
  /** \brief The kinematic model for which motion plans are computed */
  robot_model::RobotModelConstPtr kmodel_;

  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;

  /** \brief All the existing planning configurations. The name
      of the configuration is the key of the map. This name can
      be of the form "group_name[config_name]" if there are
      particular configurations specified for a group, or of the
      form "group_name" if default settings are to be used. */
  planning_interface::PlannerConfigurationMap planner_configs_;

  std::shared_ptr<pluginlib::ClassLoader<OMPLPlanningContext>> ompl_context_loader_;

private:
  // MOVEIT_CLASS_FORWARD(LastPlanningContext);
  // LastPlanningContextPtr last_planning_context_;

  // MOVEIT_CLASS_FORWARD(CachedContexts);
  // CachedContextsPtr cached_contexts_;
};
}

#endif
