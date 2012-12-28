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

#ifndef MOVEIT_PICK_PLACE_REACHABLE_VALID_GRASP_FILTER_
#define MOVEIT_PICK_PLACE_REACHABLE_VALID_GRASP_FILTER_

#include <moveit/pick_place/manipulation_stage.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_scene/planning_scene.h>

namespace pick_place
{

class ReachableAndValidGraspFilter : public ManipulationStage
{
public:
  
  struct Options
  {
    Options(const std::string &planning_group, const std::string &ik_link) :
      planning_group_(planning_group),
      ik_link_(ik_link),
      tolerance_position_xyz_(3, 1e-3), // 1mm tolerance
      tolerance_rotation_xyz_(3, 1e-2) // approx 0.573 degrees tolerance
    {
    };
    
    std::string planning_group_;
    std::string ik_link_;
    std::vector<double> tolerance_position_xyz_;
    std::vector<double> tolerance_rotation_xyz_;
  };
  
  ReachableAndValidGraspFilter(const Options &opt,
                               const planning_scene::PlanningSceneConstPtr &scene,
                               const constraint_samplers::ConstraintSamplerManagerPtr &constraints_sampler_manager,
                               unsigned int nthreads = 4);
  
  virtual bool evaluate(unsigned int thread_id, const ManipulationPlanPtr &plan) const;
  
private:

  bool isStateCollisionFree(kinematic_state::JointStateGroup *joint_state_group,
                            const std::vector<double> &joint_group_variable_values) const;
  
  Options opt_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  const constraint_samplers::ConstraintSamplerManagerPtr constraints_sampler_manager_;
  std::vector<kinematic_state::KinematicStatePtr> states_;
  std::vector<kinematic_state::JointStateGroup*> joint_state_groups_;
  unsigned int sampling_attempts_;
};

}

#endif

