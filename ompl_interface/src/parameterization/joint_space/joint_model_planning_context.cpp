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

#include "ompl_interface/parameterization/joint_space/joint_model_planning_context.h"
#include "ompl_interface/detail/projection_evaluators.h"
#include "ompl_interface/detail/constrained_sampler.h"

ompl::base::ProjectionEvaluatorPtr ompl_interface::JointModelPlanningContext::getProjectionEvaluator(const std::string &peval) const
{
  if (peval.find_first_of("link(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string link_name = peval.substr(5, peval.length() - 6);
    if (getKinematicModel()->hasLinkModel(link_name))
      return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorLinkPose(this, link_name));
    else
      ROS_ERROR("Attempted to set projection evaluator with respect to position of link '%s', but that link is not known to the kinematic model.", link_name.c_str());
  }
  else
    if (peval.find_first_of("joints(") == 0 && peval[peval.length() - 1] == ')')
    {
      std::string joints = peval.substr(7, peval.length() - 8);
      boost::replace_all(joints, ",", " ");
      std::vector<std::pair<std::string, unsigned int> > j;
      std::stringstream ss(joints);
      while (ss.good() && !ss.eof())
      {
	std::string v; ss >> v >> std::ws;
	if (getKinematicModel()->hasJointModel(v))
	{
	  unsigned int vc = getKinematicModel()->getJointModel(v)->getVariableCount();
	  if (vc > 0)
	    j.push_back(std::make_pair(v, vc));
	  else
	    ROS_WARN("%s: Ignoring joint '%s' in projection since it has 0 DOF", name_.c_str(), v.c_str());
	}
	else
	  ROS_ERROR("%s: Attempted to set projection evaluator with respect to value of joint '%s', but that joint is not known to the kinematic model.",
		    name_.c_str(), v.c_str());
      }
      if (j.empty())
	ROS_ERROR("%s: No valid joints specified for joint projection", name_.c_str());
      else
	return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorJointValue(this, j));
    }
    else
      ROS_ERROR("Unable to allocate projection evaluator based on description: '%s'", peval.c_str());  
  return ob::ProjectionEvaluatorPtr();
}

ompl::base::StateSamplerPtr ompl_interface::JointModelPlanningContext::allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const
{
  if (ompl_state_space_.get() != ss)
    ROS_FATAL("%s: Attempted to allocate a state sampler for an unknown state space", name_.c_str());
  ROS_DEBUG("%s: Allocating a new state sampler (attempts to use path constraints)", name_.c_str());
  
  if (path_constraints_)
  {
    kc::ConstraintSamplerPtr cs = kc::constructConstraintsSampler(getJointModelGroup(), path_constraints_->getAllConstraints(), getKinematicModel(),
                                                                  getPlanningScene()->getTransforms(), ompl_state_space_->getIKAllocator(),
                                                                  ompl_state_space_->getIKSubgroupAllocators());
    if (cs)
    {
      ROS_DEBUG("%s: Allocating specialized state sampler for state space", name_.c_str());
      return ob::StateSamplerPtr(new ConstrainedSampler(this, cs));
    }
  }
  ROS_DEBUG("%s: Allocating default state sampler for state space", name_.c_str());
  return ss->allocDefaultStateSampler();
}
