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

#include <moveit/constraint_samplers/constraint_sampler_allocator.h>
#include <boost/shared_ptr.hpp>

namespace constraint_samplers
{

/**
 * \brief This class assists in the generation of a ConstraintSampler for a
 * particular group from a moveit_msgs::Constraints.  
 * 
 * It contains logic that will generate either a
 * JointConstraintSampler, an IKConstraintSampler, or a
 * UnionConstraintSampler depending on the contents of the Constraints
 * message and the group in question.
 * 
 */
class ConstraintSamplerManager
{
public:
  /** 
   * \brief Empty constructor
   * 
   */
  ConstraintSamplerManager(void)
  {
  }
  /** 
   * \brief Allows the user to specify an alternate ConstraintSamplerAllocation
   * 
   * @param sa The constraint sampler allocator that will be used
   */
  void registerSamplerAllocator(const ConstraintSamplerAllocatorPtr &sa)
  {
    sampler_alloc_.push_back(sa);
  }
  /** 
   * \brief Selects among the potential sampler allocators.  
   * 
   * This function will iterate through the constraint sampler
   * allocators, trying to find one that can service the constraints.
   * The first one that can service the request will be called.  If no
   * allocators can service the Constraints, or there are no
   * allocators, the selectDefaultSampler will be called.
   * 
   * @param scene The planning scene that will be passed into the constraint sampler
   * @param group_name The group name for which to allocate the constraint sampler
   * @param constr The constraints
   * 
   * @return A boost::shared_ptr to the ConstraintSampler that is
   * allocated, or an empty pointer if none could be allocated
   */
  ConstraintSamplerPtr selectSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr) const;

  /** 
   * \brief Default logic to select a ConstraintSampler given a
   * constraints message.
   *
   * 
   * 
   * @param scene 
   * @param group_name 
   * @param constr 
   * 
   * @return 
   */
  static ConstraintSamplerPtr selectDefaultSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr);
  
private:

  std::vector<ConstraintSamplerAllocatorPtr> sampler_alloc_; /**< \brief Holds the constraint sampler allocators, which will be tested in order  */
};

typedef boost::shared_ptr<ConstraintSamplerManager> ConstraintSamplerManagerPtr; /**< \brief boost::shared_ptr to a ConstraintSamplerManager */
typedef boost::shared_ptr<const ConstraintSamplerManager> ConstraintSamplerManagerConstPtr; /**< \brief boost::shared_ptr to a const ConstraintSamplerManager */

}


#endif
