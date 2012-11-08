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

#include <moveit/constraint_samplers/constraint_sampler.h>
#include <random_numbers/random_numbers.h>

namespace constraint_samplers
{

struct JointInfo
{
  JointInfo() 
  {
    min_bound_ = -std::numeric_limits<double>::max();
    max_bound_ = std::numeric_limits<double>::max();
  }

  void potentiallyAdjustMinMaxBounds(double min, double max) 
  {
    min_bound_ = std::max(min, min_bound_);
    max_bound_ = std::min(max, max_bound_);
  }

  double min_bound_;
  double max_bound_;
  std::size_t index_;
};

/**
 * \brief JointConstraintSampler is a class that allows the sampling
 * of joints in a particular group of the robot.  It further allows a
 * set of constraints to be associated with the group that will reduce
 * the allowable bounds used in the sampling.  Unconstrained values
 * will be sampled within their limits.
 *
 */
class JointConstraintSampler : public ConstraintSampler
{
public:

  /** 
   * Constructor
   * 
   * @param scene The planning scene used to check the constraint 
   * @param group_name The group name associated with the constraint
   * 
   * @return 
   */
  JointConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, 
                         const std::string &group_name) :
    ConstraintSampler(scene, group_name),
    is_valid_(false)
  {
  }
  
  virtual bool configure(const moveit_msgs::Constraints &constr);
  virtual bool sample(kinematic_state::JointStateGroup *jsg, const kinematic_state::KinematicState &ks,  unsigned int max_attempts);
  
  bool configure(const std::vector<kinematic_constraints::JointConstraint> &jc);

  std::size_t getConstrainedJointCount(void) const
  {
    return bounds_.size();
  }
  
  std::size_t getUnconstrainedJointCount(void) const
  {
    return unbounded_.size();
  }
  
protected:
  
  void clear();
  bool is_valid_;
  std::string joint_state_group_name_;

  random_numbers::RandomNumberGenerator           random_number_generator_;  
  std::vector<JointInfo> bounds_;

  std::vector<const kinematic_model::JointModel*> unbounded_;
  std::vector<unsigned int>                       uindex_;
  std::vector<double>                             values_;
};


struct IKSamplingPose
{
  IKSamplingPose(void);
  IKSamplingPose(const kinematic_constraints::PositionConstraint &pc);
  IKSamplingPose(const kinematic_constraints::OrientationConstraint &oc);
  IKSamplingPose(const kinematic_constraints::PositionConstraint &pc, 
                 const kinematic_constraints::OrientationConstraint &oc);
  
  IKSamplingPose(const boost::shared_ptr<kinematic_constraints::PositionConstraint> &pc);
  IKSamplingPose(const boost::shared_ptr<kinematic_constraints::OrientationConstraint> &oc);
  IKSamplingPose(const boost::shared_ptr<kinematic_constraints::PositionConstraint> &pc, 
                 const boost::shared_ptr<kinematic_constraints::OrientationConstraint> &oc);
  
  boost::shared_ptr<kinematic_constraints::PositionConstraint>    position_constraint_;
  boost::shared_ptr<kinematic_constraints::OrientationConstraint> orientation_constraint_;
};

class IKConstraintSampler : public ConstraintSampler
{
public:

  IKConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, 
                      const std::string &group_name, 
                      const IKSamplingPose &sp) :
    ConstraintSampler(scene, group_name)
  {
    setup(sp);
  }
  
  virtual bool configure(const moveit_msgs::Constraints &constr);

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
    return sampling_pose_.position_constraint_;
  }
  
  const boost::shared_ptr<kinematic_constraints::OrientationConstraint>& getOrientationConstraint(void) const
  {
    return sampling_pose_.orientation_constraint_;
  }

  bool loadIKSolver(void);
  
  double getSamplingVolume(void) const;
  const std::string& getLinkName(void) const;
  
  bool samplePose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat, const kinematic_state::KinematicState &ks, unsigned int max_attempts);
  virtual bool sample(kinematic_state::JointStateGroup *jsg, const kinematic_state::KinematicState &ks, unsigned int max_attempts);
  
protected:
  
  bool callIK(const geometry_msgs::Pose &ik_query, double timeout, kinematic_state::JointStateGroup *jsg);

  random_numbers::RandomNumberGenerator random_number_generator_;
  kinematic_model::SolverAllocatorFn    ik_alloc_;
  IKSamplingPose                        sampling_pose_;
  kinematics::KinematicsBasePtr         kb_;
  double                                ik_timeout_;
  std::vector<unsigned int>             ik_joint_bijection_;
  std::string                           ik_frame_;
  bool                                  transform_ik_;
};


}


#endif
