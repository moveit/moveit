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

#ifndef MOVEIT_CONSTRAINT_SAMPLERS_CONSTRAINT_SAMPLER_
#define MOVEIT_CONSTRAINT_SAMPLERS_CONSTRAINT_SAMPLER_

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace constraint_samplers
{
/**
 * \brief ConstraintSampler is an abstract base class that allows the
 * sampling of a kinematic state for a particular group of a robot.
 */
class ConstraintSampler
{
public:
  
  static const unsigned int DEFAULT_MAX_SAMPLING_ATTEMPTS = 2; /**< \brief The default value associated with a sampling request.  By default if a valid sample cannot be produced in this many attempts, it returns with no sample */
  
  /** 
   * Constructor
   * 
   * @param [in] scene The planning scene that will be used for constraint checking 
   * @param [in] group_name The group name of the associated group.  Will be invalid if no group name is passed in
   */
  ConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name);

  virtual ~ConstraintSampler(void)
  {
  }
  
  virtual bool configure(const moveit_msgs::Constraints &constr) = 0;

  const std::string& getGroupName(void) const
  {
    return getJointModelGroup()->getName();
  }
  
  const kinematic_model::JointModelGroup* getJointModelGroup(void) const
  {
    return jmg_;
  }
  
  const planning_scene::PlanningSceneConstPtr& getPlanningScene(void) const
  {
    return scene_;
  }
  
  /// Return the names of the mobile frames (correspond to robot links) whose pose is needed when sample() is called.
  const std::vector<std::string>& getFrameDependency(void) const
  {
    return frame_depends_;
  }
  
  bool sample(kinematic_state::JointStateGroup *jsg, const kinematic_state::KinematicState &reference_state)
  {
    return sample(jsg, reference_state, DEFAULT_MAX_SAMPLING_ATTEMPTS);
  }
  
  virtual bool sample(kinematic_state::JointStateGroup *jsg, const kinematic_state::KinematicState &reference_state, unsigned int max_attempts) = 0;
  
protected:

  planning_scene::PlanningSceneConstPtr                   scene_;
  const kinematic_model::JointModelGroup *jmg_;
  std::vector<std::string>                                frame_depends_;
};

typedef boost::shared_ptr<ConstraintSampler> ConstraintSamplerPtr;
typedef boost::shared_ptr<const ConstraintSampler> ConstraintSamplerConstPtr;


}


#endif
