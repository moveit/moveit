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

#include "ompl_interface/detail/projection_evaluators.h"

ompl_interface::ProjectionEvaluatorLinkPose::ProjectionEvaluatorLinkPose(const PlanningGroup *pg, const std::string &link) :
  ompl::base::ProjectionEvaluator(&pg->getKMStateSpace()), planning_group_(pg),
  group_name_(planning_group_->getJointModelGroup()->getName()), link_name_(link), tss_(pg->getStartState())
{
}

unsigned int ompl_interface::ProjectionEvaluatorLinkPose::getDimension(void) const
{
  return 3;
}

void ompl_interface::ProjectionEvaluatorLinkPose::defaultCellSizes(void)
{
  cellSizes_.resize(3);
  cellSizes_[0] = 0.01;
  cellSizes_[1] = 0.01;
  cellSizes_[2] = 0.01;
}

void ompl_interface::ProjectionEvaluatorLinkPose::project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
{
  planning_models::KinematicState *s = tss_.getStateStorage();
  planning_group_->getKMStateSpace().copyToKinematicState(*s, state);
  s->getJointStateGroup(group_name_)->updateLinkTransforms();
  
  const planning_models::KinematicState::LinkState *ls = s->getLinkState(link_name_);
  const Eigen::Vector3d &o = ls->getGlobalLinkTransform().translation();
  projection(0) = o.x();
  projection(1) = o.y();
  projection(2) = o.z();
}

ompl_interface::ProjectionEvaluatorJointValue::ProjectionEvaluatorJointValue(const PlanningGroup *pg, const std::vector<std::pair<std::string, unsigned int> > &joints) :
  ompl::base::ProjectionEvaluator(&pg->getKMStateSpace()), planning_group_(pg), joints_(joints)
{
  dimension_ = 0;
  for (std::size_t i = 0 ; i < joints_.size() ; ++i)
    dimension_ += joints_[i].second;
}

unsigned int ompl_interface::ProjectionEvaluatorJointValue::getDimension(void) const
{
  return dimension_;
}

void ompl_interface::ProjectionEvaluatorJointValue::defaultCellSizes(void)
{
  cellSizes_.clear();
  cellSizes_.resize(dimension_, 0.1);
}

void ompl_interface::ProjectionEvaluatorJointValue::project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
{
  unsigned int k = 0;
  for (std::size_t i = 0 ; i < joints_.size() ; ++i)
  {
    const double *v = planning_group_->getKMStateSpace().getValueAddressAtName(state, joints_[i].first);
    for (unsigned int j = 0 ; j < joints_[i].second ; ++j)
      projection(k++) = v[j];
  }
}
