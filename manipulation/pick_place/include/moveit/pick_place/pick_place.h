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

/* Author: Ioan Sucan, Sachin Chitta */

#ifndef MOVEIT_PICK_PLACE_PICK_PLACE_
#define MOVEIT_PICK_PLACE_PICK_PLACE_

#include <moveit/pick_place/manipulation_stage.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>

namespace pick_place
{

class PickPlace
{
public: 

  PickPlace(const planning_pipeline::PlanningPipelinePtr &planning_pipeline);
  
  const constraint_samplers::ConstraintSamplerManagerPtr& getConstraintsSamplerManager(void) const
  {
    return constraint_sampler_manager_loader_->getConstraintSamplerManager();
  }
  
  const planning_pipeline::PlanningPipelinePtr& getPlanningPipeline(void) const
  {
    return planning_pipeline_;
  }
  
  /** \brief Plan the sequence of motions that perform a pickup action */
  ManipulationPlanPtr planPick(const planning_scene::PlanningScenePtr &planning_scene, const moveit_msgs::PickupGoal &goal) const;

  /** \brief Plan the sequence of motions that perform a placement action */
  ManipulationPlanPtr planPlace(const planning_scene::PlanningScenePtr &planning_scene, const moveit_msgs::PlaceGoal &goal) const;

  void displayPlan(const ManipulationPlanPtr &plan) const;
  
private:
  
  ros::NodeHandle nh_;
  ros::Publisher display_path_publisher_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;  
  constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
};

typedef boost::shared_ptr<PickPlace> PickPlacePtr;
typedef boost::shared_ptr<const PickPlace> PickPlaceConstPtr;

}

#endif
