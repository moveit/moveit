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

#ifndef MOVEIT_CONSTRAINT_SAMPLERS_CONSTRAINT_SAMPLER_MANAGER_
#define MOVEIT_CONSTRAINT_SAMPLERS_CONSTRAINT_SAMPLER_MANAGER_

#include "constraint_samplers/constraint_sampler_allocator.h"

namespace constraint_samplers
{

class ConstraintSamplerManager
{
public:
  ConstraintSamplerManager(void)
  {
  }

  void registerSamplerAllocator(const ConstraintSamplerAllocatorPtr &sa)
  {
    sampler_alloc_.push_back(sa);
  }
  
  ConstraintSamplerPtr selectSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr) const;
  static ConstraintSamplerPtr selectDefaultSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr);
  
private:

  std::vector<ConstraintSamplerAllocatorPtr> sampler_alloc_;
};

}


#endif
