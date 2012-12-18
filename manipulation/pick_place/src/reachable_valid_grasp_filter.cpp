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

#include <moveit/pick_place/reachable_valid_grasp_filter.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/kinematic_constraints/utils.h>

namespace pick_place
{

ReachableAndValidGraspFilter::ReachableAndValidGraspFilter(const Options &opt,
                                                           const planning_scene::PlanningSceneConstPtr &scene,
                                                           unsigned int nthreads) :
  GraspFilter(scene, nthreads),
  opt_(opt)
{
  states_.resize(nthreads_);
  for (std::size_t i = 0 ; i < states_.size() ; ++i)
  {
    states_[i].reset(new kinematic_state::KinematicState(scene->getCurrentState()));
    joint_state_groups_[i] = states_[i]->getJointStateGroup(opt_.planning_group_);
  }
}

bool ReachableAndValidGraspFilter::evaluate(unsigned int thread_id, const Grasp &grasp) const
{
  geometry_msgs::Pose pose;
  
  if (joint_state_groups_[thread_id]->setFromIK(pose, opt_.ik_link_))
    if (next_)
      next_->push(grasp);
}

}
