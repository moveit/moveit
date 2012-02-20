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

#ifndef KINEMATIC_CONSTRAINTS_CONSTRAINT_SAMPLERS_
#define KINEMATIC_CONSTRAINTS_CONSTRAINT_SAMPLERS_

#include "kinematic_constraints/kinematic_constraint.h"
#include <random_numbers/random_numbers.h>
#include <kinematics_base/kinematics_base.h>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace kinematic_constraints
{

class ConstraintSampler
{
public:
  ConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg);
  
  virtual ~ConstraintSampler(void);
  
  const planning_models::KinematicModel::JointModelGroup* getJointModelGroup(void)
  {
    return jmg_;
  }

  /// Return the names of the mobile frames (correspond to robot links) whose pose is needed when sample() is called.
  const std::vector<std::string>& getFrameDependency(void)
  {
    return frame_depends_;
  }
  
  virtual bool sample(std::vector<double> &values, const planning_models::KinematicState &ks, unsigned int max_attempts = 100) = 0;
  
protected:
  
  const planning_models::KinematicModel::JointModelGroup *jmg_;
  std::vector<std::string>                                frame_depends_;
  random_numbers::RandomNumberGenerator                   random_number_generator_;
};

typedef boost::shared_ptr<ConstraintSampler> ConstraintSamplerPtr;
typedef boost::shared_ptr<const ConstraintSampler> ConstraintSamplerConstPtr;

/// function type that allocates an IK solver for a particular group
typedef boost::function<boost::shared_ptr<kinematics::KinematicsBase>(const planning_models::KinematicModel::JointModelGroup*)> KinematicsAllocator;

/// some groups do not have IK solver associated to them, but may contain disjoint groups that do have IK solvers associated to them
typedef std::map<const planning_models::KinematicModel::JointModelGroup*, KinematicsAllocator> KinematicsSubgroupAllocator;

class JointConstraintSampler : public ConstraintSampler
{
public:
  
  JointConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg, const std::vector<JointConstraint> &jc);
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
  
  std::vector<std::pair<double, double> >                         bounds_;
  std::vector<unsigned int>                                       index_;
  
  std::vector<const planning_models::KinematicModel::JointModel*> unbounded_;
  std::vector<unsigned int>                                       uindex_;
};


struct IKSamplingPose
{
  IKSamplingPose();
  IKSamplingPose(const PositionConstraint &pc);
  IKSamplingPose(const OrientationConstraint &oc);
  IKSamplingPose(const PositionConstraint &pc, const OrientationConstraint &oc);
  
  IKSamplingPose(const boost::shared_ptr<PositionConstraint> &pc);
  IKSamplingPose(const boost::shared_ptr<OrientationConstraint> &oc);
  IKSamplingPose(const boost::shared_ptr<PositionConstraint> &pc, const boost::shared_ptr<OrientationConstraint> &oc);
  
  boost::shared_ptr<PositionConstraint>    pc_;
  boost::shared_ptr<OrientationConstraint> oc_;
};

class IKConstraintSampler : public ConstraintSampler
{
public:
  
  IKConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg, const KinematicsAllocator &ik_alloc, const IKSamplingPose &sp);
  
  double getIKTimeout(void) const
  {
    return ik_timeout_;
  }
  
  void setIKTimeout(double timeout)
  {
    ik_timeout_ = timeout;
  }
  
  bool initialize(void);
  
  const boost::shared_ptr<PositionConstraint>& getPositionConstraint(void) const
  {
    return sp_.pc_;
  }
  
  const boost::shared_ptr<OrientationConstraint>& getOrientationConstraint(void) const
  {
    return sp_.oc_;
  }
  
  double getSamplingVolume(void) const;
  const std::string& getLinkName(void) const;
  
  bool samplePose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat, const planning_models::KinematicState &ks, unsigned int max_attempts = 100);
  virtual bool sample(std::vector<double> &values, const planning_models::KinematicState &ks, unsigned int max_attempts = 100);
  
protected:
  
  bool callIK(const geometry_msgs::Pose &ik_query, double timeout, std::vector<double> &solution);
  bool loadIKSolver(void);
  
  KinematicsAllocator                           ik_alloc_;
  IKSamplingPose                                sp_;
  boost::shared_ptr<kinematics::KinematicsBase> kb_;
  double                                        ik_timeout_;
  std::vector<unsigned int>                     ik_joint_bijection_;
  std::string                                   ik_frame_;
  bool                                          transform_ik_;
};

class UnionConstraintSampler : public ConstraintSampler
{
public:
  
  UnionConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg, std::vector<ConstraintSamplerPtr> &samplers);
  
  const std::vector<ConstraintSamplerPtr>& getSamplers(void) const
  {
    return samplers_;
  }
  
  virtual bool sample(std::vector<double> &values, const planning_models::KinematicState &ks, unsigned int max_attempts = 100);
  
protected:
  
  std::vector<ConstraintSamplerPtr>       samplers_;
  std::vector<std::vector<unsigned int> > bijection_;
};

ConstraintSamplerPtr constructConstraintsSampler(const planning_models::KinematicModel::JointModelGroup *jmg, const moveit_msgs::Constraints &constr,
                                                 const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf,
                                                 const KinematicsAllocator &ik_alloc = KinematicsAllocator(), const KinematicsSubgroupAllocator &ik_subgroup_alloc = KinematicsSubgroupAllocator());
}


#endif
