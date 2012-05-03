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

#ifndef MOVEIT_CONSTRAINT_SAMPLERS_DEFAULT_CONSTRAINT_SAMPLERS_
#define MOVEIT_CONSTRAINT_SAMPLERS_DEFAULT_CONSTRAINT_SAMPLERS_

#include "constraint_samplers/constraint_sampler.h"
#include <random_numbers/random_numbers.h>

namespace constraint_samplers
{

class JointConstraintSampler : public ConstraintSampler
{
public:
  JointConstraintSampler(void) : ConstraintSampler()
  {
  }
  
  JointConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const planning_models::KinematicModel::JointModelGroup *jmg, const std::vector<kinematic_constraints::JointConstraint> &jc) : ConstraintSampler()
  {
    scene_ = scene;
    jmg_ = jmg;
    setup(jc);
  }
  
  virtual bool configure(const moveit_msgs::Constraints &constr);
  
  bool setup(const std::vector<kinematic_constraints::JointConstraint> &jc);

  virtual bool canService(const moveit_msgs::Constraints &constr) const;
  
  virtual bool sample(std::vector<double> &values, const planning_models::KinematicState &ks,  unsigned int max_attempts = 100);
  
  std::size_t getConstrainedJointCount(void) const
  {
    return bounds_.size();
  }
  
  std::size_t getUnconstrainedJointCount(void) const
  {
    return unbounded_.size();
  }
  
protected:

  random_numbers::RandomNumberGenerator                           random_number_generator_;  
  std::vector<std::pair<double, double> >                         bounds_;
  std::vector<unsigned int>                                       index_;
  
  std::vector<const planning_models::KinematicModel::JointModel*> unbounded_;
  std::vector<unsigned int>                                       uindex_;
};


struct IKSamplingPose
{
  IKSamplingPose(void);
  IKSamplingPose(const kinematic_constraints::PositionConstraint &pc);
  IKSamplingPose(const kinematic_constraints::OrientationConstraint &oc);
  IKSamplingPose(const kinematic_constraints::PositionConstraint &pc, const kinematic_constraints::OrientationConstraint &oc);
  
  IKSamplingPose(const boost::shared_ptr<kinematic_constraints::PositionConstraint> &pc);
  IKSamplingPose(const boost::shared_ptr<kinematic_constraints::OrientationConstraint> &oc);
  IKSamplingPose(const boost::shared_ptr<kinematic_constraints::PositionConstraint> &pc, const boost::shared_ptr<kinematic_constraints::OrientationConstraint> &oc);
  
  boost::shared_ptr<kinematic_constraints::PositionConstraint>    pc_;
  boost::shared_ptr<kinematic_constraints::OrientationConstraint> oc_;
};

class IKConstraintSampler : public ConstraintSampler
{
public:
  
  IKConstraintSampler(void) : ConstraintSampler()
  {
  }
  
  IKConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const planning_models::KinematicModel::JointModelGroup *jmg, const IKSamplingPose &sp) : ConstraintSampler()
  {
    jmg_ = jmg;
    scene_ = scene;
    setup(sp);
  }
  
  virtual bool configure(const moveit_msgs::Constraints &constr);

  virtual bool canService(const moveit_msgs::Constraints &constr) const;

  bool setup(const IKSamplingPose &sp);
  
  double getIKTimeout(void) const
  {
    return ik_timeout_;
  }
  
  void setIKTimeout(double timeout)
  {
    ik_timeout_ = timeout;
  }
  
  const boost::shared_ptr<kinematic_constraints::PositionConstraint>& getPositionConstraint(void) const
  {
    return sp_.pc_;
  }
  
  const boost::shared_ptr<kinematic_constraints::OrientationConstraint>& getOrientationConstraint(void) const
  {
    return sp_.oc_;
  }

  bool loadIKSolver(void);
  
  double getSamplingVolume(void) const;
  const std::string& getLinkName(void) const;
  
  bool samplePose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat, const planning_models::KinematicState &ks, unsigned int max_attempts = 100);
  virtual bool sample(std::vector<double> &values, const planning_models::KinematicState &ks, unsigned int max_attempts = 100);
  
protected:
  
  bool callIK(const geometry_msgs::Pose &ik_query, double timeout, std::vector<double> &solution);

  random_numbers::RandomNumberGenerator         random_number_generator_;
  planning_scene::KinematicsAllocatorFn         ik_alloc_;
  IKSamplingPose                                sp_;
  boost::shared_ptr<kinematics::KinematicsBase> kb_;
  double                                        ik_timeout_;
  std::vector<unsigned int>                     ik_joint_bijection_;
  std::string                                   ik_frame_;
  bool                                          transform_ik_;
};


}


#endif
