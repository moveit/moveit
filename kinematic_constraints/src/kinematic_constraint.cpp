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

#include <kinematic_constraints/kinematic_constraint.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <planning_models/conversions.h>
#include <collision_detection_fcl/collision_robot.h>
#include <collision_detection_fcl/collision_world.h>
#include <boost/scoped_ptr.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/bind.hpp>
#include <limits>
#include <ros/console.h>

namespace kinematic_constraints
{
static double normalizeAngle(double angle)
{   
  double v = fmod(angle, 2.0 * boost::math::constants::pi<double>());
  if (v < -boost::math::constants::pi<double>())
    v += 2.0 * boost::math::constants::pi<double>();
  else
    if (v > boost::math::constants::pi<double>())
      v -= 2.0 * boost::math::constants::pi<double>();
  return v;
}
}

kinematic_constraints::KinematicConstraint::KinematicConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
  type_(UNKNOWN_CONSTRAINT), kmodel_(model), tf_(tf), constraint_weight_(std::numeric_limits<double>::epsilon())
{
}

kinematic_constraints::KinematicConstraint::~KinematicConstraint(void)
{
}

bool kinematic_constraints::JointConstraint::configure(const moveit_msgs::JointConstraint &jc)
{
  joint_variable_name_ = jc.joint_name;
  local_variable_name_.clear();
  if (kmodel_->hasJointModel(joint_variable_name_))
    joint_model_ = kmodel_->getJointModel(joint_variable_name_);
  else
  {
    std::size_t pos = jc.joint_name.find_last_of("/");
    if (pos != std::string::npos)
    {
      joint_model_ = kmodel_->getJointModel(jc.joint_name.substr(0, pos));
      if (pos + 1 < jc.joint_name.length())
        local_variable_name_ = jc.joint_name.substr(pos + 1);
    }
    else
      joint_model_ = kmodel_->getJointModel(jc.joint_name);
  }
  
  if (joint_model_)
  {
    if (local_variable_name_.empty())
    {
      // check if the joint has 1 DOF (the only kind we can handle)
      if (joint_model_->getVariableCount() == 0)
      {
        ROS_ERROR_STREAM("Joint '" << jc.joint_name << "' has no parameters to constrain");
        joint_model_ = NULL;
      }
      else
        if (joint_model_->getVariableCount() > 1)
        {
          ROS_ERROR_STREAM("Joint '" << jc.joint_name << "' has more than one parameter to constrain. This type of constraint is not supported.");
          joint_model_ = NULL;
        }
    }
    else
    {
      int found = -1;
      const std::vector<std::string> &local_var_names = joint_model_->getLocalVariableNames();
      for (std::size_t i = 0 ; i < local_var_names.size() ; ++i)
        if (local_var_names[i] == local_variable_name_)
        {
          found = i;
          break;
        }
      if (found < 0)
      {
        ROS_ERROR("Local variable name '%s' is not known to joint '%s'", local_variable_name_.c_str(), joint_model_->getName().c_str());
        joint_model_ = NULL;
      }
    }
  }
  

  if (joint_model_)
  {
    joint_is_continuous_ = false;
    joint_tolerance_above_ = jc.tolerance_above;
    joint_tolerance_below_ = jc.tolerance_below;
    
    // check if we have to wrap angles when computing distances
    joint_is_continuous_ = false;
    if (joint_model_->getType() == planning_models::KinematicModel::JointModel::REVOLUTE)
    {
      const planning_models::KinematicModel::RevoluteJointModel *rjoint = static_cast<const planning_models::KinematicModel::RevoluteJointModel*>(joint_model_);
      if (rjoint->isContinuous())
        joint_is_continuous_ = true;
    }
    else
      if (joint_model_->getType() == planning_models::KinematicModel::JointModel::PLANAR)
      {
        if (local_variable_name_ == "theta") 
          joint_is_continuous_ = true;
      }
    
    if (joint_is_continuous_)
    {
      joint_position_ = normalizeAngle(jc.position);
    }
    else
    {
      joint_position_ = jc.position;
      
      std::pair<double, double> bounds;
      joint_model_->getVariableBounds(joint_variable_name_, bounds);

      if (bounds.first > joint_position_ + joint_tolerance_above_)
      {
	joint_position_ = bounds.first;
	joint_tolerance_above_ = std::numeric_limits<double>::epsilon();
	ROS_WARN("Joint %s is constrained to be below the minimum bounds. Assuming minimum bounds instead.", jc.joint_name.c_str());
      }
      else
	if (bounds.second < joint_position_ - joint_tolerance_below_)
	{
	  joint_position_ = bounds.second;
	  joint_tolerance_below_ = std::numeric_limits<double>::epsilon();
	  ROS_WARN("Joint %s is constrained to be above the maximum bounds. Assuming maximum bounds instead.", jc.joint_name.c_str());
	}
    }
    
    if (jc.weight <= std::numeric_limits<double>::epsilon())
      ROS_WARN_STREAM("The weight on constraint for joint '" << jc.joint_name << "' should be positive");
    else
      constraint_weight_ = jc.weight;
  }
  return joint_model_ != NULL;
}

bool kinematic_constraints::JointConstraint::equal(const KinematicConstraint &other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const JointConstraint &o = static_cast<const JointConstraint&>(other);
  if (o.joint_model_ == joint_model_ && o.local_variable_name_ == local_variable_name_)
    return fabs(joint_position_ - o.joint_position_) <= margin &&
      fabs(joint_tolerance_above_ - o.joint_tolerance_above_) <= margin &&
      fabs(joint_tolerance_below_ - o.joint_tolerance_below_) <= margin;
  return false;
}

kinematic_constraints::ConstraintEvaluationResult kinematic_constraints::JointConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
  if (!joint_model_)
    return ConstraintEvaluationResult(true, 0.0);
  
  const planning_models::KinematicState::JointState *joint = state.getJointState(joint_model_->getName());
  
  if (!joint)
  {
    ROS_WARN_STREAM("No joint in state with name '" << joint_model_->getName() << "'"); 
    return ConstraintEvaluationResult(true, 0.0);
  }
  
  double current_joint_position = joint->getVariableValues()[0];
  if (!local_variable_name_.empty())
  {
    const std::map<std::string, unsigned int> &index_map = joint->getVariableIndexMap();
    std::map<std::string, unsigned int>::const_iterator it = index_map.find(joint_variable_name_);
    if (it == index_map.end())
    {   
      ROS_WARN_STREAM("Local name '" << local_variable_name_ << "' is not known to joint state with name '" << joint_model_->getName() << "'");
      return ConstraintEvaluationResult(true, 0.0);
    }
    else
      current_joint_position = joint->getVariableValues()[it->second];
  }
  
  double dif = 0.0;
  
  // compute signed shortest distance for continuous joints
  if (joint_is_continuous_)
  {
    dif = normalizeAngle(current_joint_position) - joint_position_;
    
    if (dif > boost::math::constants::pi<double>())
      dif = 2.0*boost::math::constants::pi<double>() - dif;
    else
      if (dif < -boost::math::constants::pi<double>())
        dif += 2.0*boost::math::constants::pi<double>(); // we include a sign change to have dif > 0
    // however, we want to include proper sign for diff, as the tol below is may be different from tol above
    if (current_joint_position < joint_position_)
      dif = -dif;
  }
  else
    dif = current_joint_position - joint_position_;
  
  // check bounds
  bool result = dif <= joint_tolerance_above_ && dif >= -joint_tolerance_below_;
  if (verbose)
    ROS_INFO("Constraint %s:: Joint name: '%s', actual value: %f, desired value: %f, tolerance_above: %f, tolerance_below: %f",
             result ? "satisfied" : "violated", joint_variable_name_.c_str(),
             current_joint_position, joint_position_, joint_tolerance_above_, joint_tolerance_below_);
  return ConstraintEvaluationResult(result, constraint_weight_ * fabs(dif));
}

bool kinematic_constraints::JointConstraint::enabled(void) const
{
  return joint_model_;
}

void kinematic_constraints::JointConstraint::clear(void)
{
  joint_model_ = NULL;
}

void kinematic_constraints::JointConstraint::print(std::ostream &out) const
{
  if (joint_model_)
  {
    out << "Joint constraint for joint " << joint_variable_name_ << ": " << std::endl;
    out << "  value = ";
    out << joint_position_ << "; ";
    out << "  tolerance below = ";
    out << joint_tolerance_below_ << "; ";
    out << "  tolerance above = ";
    out << joint_tolerance_above_ << "; ";
    out << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}

bool kinematic_constraints::PositionConstraint::configure(const moveit_msgs::PositionConstraint &pc)
{
  link_model_ = kmodel_->getLinkModel(pc.link_name);
  offset_ = Eigen::Vector3d(pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z);
  has_offset_ = offset_.squaredNorm() > std::numeric_limits<double>::epsilon();
  
  if (pc.header.frame_id.empty())
    ROS_WARN("No frame specified for position constraint on link '%s'!", pc.link_name.c_str());
  
  if (tf_->isFixedFrame(pc.header.frame_id))
  {
    constraint_frame_id_ = tf_->getTargetFrame();
    mobile_frame_ = false;
  }
  else
  {
    constraint_frame_id_ = pc.header.frame_id;
    mobile_frame_ = true;
  }
  
  // load primitive shapes
  for (std::size_t i = 0 ; i < pc.constraint_region.primitives.size() ; ++i)
  {
    boost::scoped_ptr<shapes::Shape> shape(shapes::constructShapeFromMsg(pc.constraint_region.primitives[i]));
    if (shape)
    {
      constraint_region_.push_back(bodies::BodyPtr(bodies::createBodyFromShape(shape.get())));
      Eigen::Affine3d t;
      if (!planning_models::poseFromMsg(pc.constraint_region.primitive_poses[i], t))
        ROS_WARN("Incorrect specification of orientation in pose for link '%s'. Assuming identity quaternion.", pc.link_name.c_str());
      constraint_region_pose_.push_back(t);
      if (mobile_frame_)
	constraint_region_.back()->setPose(constraint_region_pose_.back());
      else
      {
        tf_->transformPose(pc.header.frame_id, constraint_region_pose_.back(), constraint_region_pose_.back());
        constraint_region_.back()->setPose(constraint_region_pose_.back());
      }
    }
  }
  
  // load meshes
  for (std::size_t i = 0 ; i < pc.constraint_region.meshes.size() ; ++i)
  {
    boost::scoped_ptr<shapes::Shape> shape(shapes::constructShapeFromMsg(pc.constraint_region.meshes[i]));
    if (shape)
    {
      constraint_region_.push_back(bodies::BodyPtr(bodies::createBodyFromShape(shape.get())));
      Eigen::Affine3d t;
      if (!planning_models::poseFromMsg(pc.constraint_region.mesh_poses[i], t))
        ROS_WARN("Incorrect specification of orientation in pose for link '%s'. Assuming identity quaternion.", pc.link_name.c_str());
      constraint_region_pose_.push_back(t);
      if (mobile_frame_)
	constraint_region_.back()->setPose(constraint_region_pose_.back());
      else
      {
        tf_->transformPose(pc.header.frame_id, constraint_region_pose_.back(), constraint_region_pose_.back());
        constraint_region_.back()->setPose(constraint_region_pose_.back());
      }
    }
  }
  
  if (pc.weight <= std::numeric_limits<double>::epsilon())
    ROS_WARN_STREAM("The weight on position constraint for link '" << pc.link_name << "' should be positive");
  else
    constraint_weight_ = pc.weight;
  
  return !constraint_region_.empty();
}

bool kinematic_constraints::PositionConstraint::equal(const KinematicConstraint &other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const PositionConstraint &o = static_cast<const PositionConstraint&>(other);
  
  if (link_model_ == o.link_model_ && constraint_frame_id_ == o.constraint_frame_id_)
  {
    if ((offset_ - o.offset_).norm() > margin)
      return false;
    if (constraint_region_.size() != o.constraint_region_.size())
      return false;
    for (std::size_t i = 0 ; i < constraint_region_.size() ; ++i)
    {
      Eigen::Affine3d diff = constraint_region_pose_[i].inverse() * o.constraint_region_pose_[i];
      if (diff.translation().norm() > margin)
        return false;
      if (!diff.rotation().isIdentity(margin))
        return false;
      if (fabs(constraint_region_[i]->computeVolume() - o.constraint_region_[i]->computeVolume()) >= margin)
        return false;
    }
    return true;
  }
  return false;
}

namespace kinematic_constraints
{
// helper function to avoid code duplication
static inline kinematic_constraints::ConstraintEvaluationResult finishPositionConstraintDecision(const Eigen::Vector3d &pt, const Eigen::Vector3d &desired, const std::string &name,
                                                                                                 double weight, bool result, bool verbose)
{
  if (verbose)
    ROS_INFO("Position constraint %s on link '%s'. Desired: %f, %f, %f, current: %f, %f, %f",
             result ? "satisfied" : "violated", name.c_str(), desired.x(), desired.y(), desired.z(), pt.x(), pt.y(), pt.z());
  double dx = desired.x() - pt.x();
  double dy = desired.y() - pt.y();
  double dz = desired.z() - pt.z(); 
  return ConstraintEvaluationResult(result, weight * sqrt(dx * dx + dy * dy + dz * dz));
}
}

kinematic_constraints::ConstraintEvaluationResult kinematic_constraints::PositionConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
  if (!link_model_ || constraint_region_.empty())
    return ConstraintEvaluationResult(true, 0.0);
  
  const planning_models::KinematicState::LinkState *link_state = state.getLinkState(link_model_->getName());
  
  if (!link_state)
  {
    ROS_WARN_STREAM("No link in state with name '" << link_model_->getName() << "'");
    return ConstraintEvaluationResult(false, 0.0);
  }
  
  Eigen::Vector3d pt = link_state->getGlobalLinkTransform() * offset_;
  if (mobile_frame_)
  {
    Eigen::Affine3d tmp;
    for (std::size_t i = 0 ; i < constraint_region_.size() ; ++i)
    {
      tf_->transformPose(state, constraint_frame_id_, constraint_region_pose_[i], tmp);
      bool result = constraint_region_[i]->cloneAt(tmp)->containsPoint(pt);
      if (result || (i + 1 == constraint_region_pose_.size()))
        return finishPositionConstraintDecision(pt, tmp.translation(), link_model_->getName(), constraint_weight_, result, verbose);
    }
  }
  else
  {   
    for (std::size_t i = 0 ; i < constraint_region_.size() ; ++i)
    {
      bool result = constraint_region_[i]->containsPoint(pt);
      if (result || (i + 1 == constraint_region_.size()))
        return finishPositionConstraintDecision(pt, constraint_region_[i]->getPose().translation(), link_model_->getName(), constraint_weight_, result, verbose);
    }
  }
  return ConstraintEvaluationResult(false, 0.0);
}

void kinematic_constraints::PositionConstraint::print(std::ostream &out) const
{
  if (enabled())
  {
    out << "Position constraint on link '" << link_model_->getName() << "'" << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}

void kinematic_constraints::PositionConstraint::clear(void)
{
  link_model_ = NULL;
  constraint_region_.clear();
  constraint_region_pose_.clear();
}

bool kinematic_constraints::PositionConstraint::enabled(void) const
{
  return link_model_ && !constraint_region_.empty();
}

bool kinematic_constraints::OrientationConstraint::configure(const moveit_msgs::OrientationConstraint &oc)
{
  link_model_ = kmodel_->getLinkModel(oc.link_name);
  Eigen::Quaterniond q;
  if (!planning_models::quatFromMsg(oc.orientation, q))
    ROS_WARN("Orientation constraint for link '%s' is probably incorrect: %f, %f, %f, %f. Assuming identity instead.", oc.link_name.c_str(),
             oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w);
  
  if (oc.header.frame_id.empty())
    ROS_WARN("No frame specified for position constraint on link '%s'!", oc.link_name.c_str());
  
  if (tf_->isFixedFrame(oc.header.frame_id))
  {
    tf_->transformQuaternion(oc.header.frame_id, q, q);
    desired_rotation_frame_id_ = tf_->getTargetFrame();
    desired_rotation_matrix_ = Eigen::Matrix3d(q);
    desired_rotation_matrix_inv_ = desired_rotation_matrix_.inverse();
    mobile_frame_ = false;
  }
  else
  {
    desired_rotation_frame_id_ = oc.header.frame_id;
    desired_rotation_matrix_ = Eigen::Matrix3d(q);
    mobile_frame_ = true;
  }
  ROS_DEBUG_STREAM("The desired rotation matrix for link "  << oc.link_name << " in frame " << desired_rotation_frame_id_ << " is:\n" << desired_rotation_matrix_);
  
  if (oc.weight <= std::numeric_limits<double>::epsilon())
    ROS_WARN_STREAM("The weight on orientation constraint for link '" << oc.link_name << "' should be positive");
  else
    constraint_weight_ = oc.weight;
  absolute_x_axis_tolerance_ = fabs(oc.absolute_x_axis_tolerance);
  absolute_y_axis_tolerance_ = fabs(oc.absolute_y_axis_tolerance);
  absolute_z_axis_tolerance_ = fabs(oc.absolute_z_axis_tolerance);
  
  return link_model_ != NULL;
}

bool kinematic_constraints::OrientationConstraint::equal(const KinematicConstraint &other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const OrientationConstraint &o = static_cast<const OrientationConstraint&>(other);
  
  if (o.link_model_ == link_model_ && desired_rotation_frame_id_ == o.desired_rotation_frame_id_)
  {
    Eigen::Matrix3d diff = desired_rotation_matrix_.inverse() * o.desired_rotation_matrix_;
    if (!diff.isIdentity(margin))
      return false;
    return fabs(absolute_x_axis_tolerance_ - o.absolute_x_axis_tolerance_) <= margin &&
      fabs(absolute_y_axis_tolerance_ - o.absolute_y_axis_tolerance_) <= margin &&
      fabs(absolute_z_axis_tolerance_ - o.absolute_z_axis_tolerance_) <= margin;
  }
  return false;
}

void kinematic_constraints::OrientationConstraint::clear(void)
{
  link_model_ = NULL;
}

bool kinematic_constraints::OrientationConstraint::enabled(void) const
{
  return link_model_;
}

kinematic_constraints::ConstraintEvaluationResult kinematic_constraints::OrientationConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
  if (!link_model_) 
    return ConstraintEvaluationResult(true, 0.0);

  const planning_models::KinematicState::LinkState *link_state = state.getLinkState(link_model_->getName());
  
  if (!link_state)
  {
    ROS_WARN_STREAM("No link in state with name '" << link_model_->getName() << "'");
    return ConstraintEvaluationResult(false, 0.0);
  }
  
  Eigen::Vector3d xyz;
  if (mobile_frame_)
  {
    Eigen::Matrix3d tmp;
    tf_->transformRotationMatrix(state, desired_rotation_frame_id_, desired_rotation_matrix_, tmp);
    Eigen::Affine3d diff(tmp.inverse() * link_state->getGlobalLinkTransform().rotation());
    xyz = diff.rotation().eulerAngles(0, 1, 2); // XYZ
  }
  else
  {
    Eigen::Affine3d diff(desired_rotation_matrix_inv_ * link_state->getGlobalLinkTransform().rotation());
    xyz = diff.rotation().eulerAngles(0, 1, 2); // 0,1,2 corresponds to ZXZ, the convention used in sampling constraints
  }
  
  xyz(0) = std::min(fabs(xyz(0)), boost::math::constants::pi<double>() - fabs(xyz(0)));
  xyz(1) = std::min(fabs(xyz(1)), boost::math::constants::pi<double>() - fabs(xyz(1)));
  xyz(2) = std::min(fabs(xyz(2)), boost::math::constants::pi<double>() - fabs(xyz(2)));
  bool result = xyz(2) < absolute_z_axis_tolerance_ && xyz(1) < absolute_y_axis_tolerance_ && xyz(0) < absolute_x_axis_tolerance_;
  
  if (verbose)
  {
    Eigen::Quaterniond q_act(link_state->getGlobalLinkTransform().rotation());
    Eigen::Quaterniond q_des(desired_rotation_matrix_);
    ROS_INFO("Orientation constraint %s for link '%s'. Quaternion desired: %f %f %f %f, quaternion actual: %f %f %f %f, error: x=%f, y=%f, z=%f, tolerance: x=%f, y=%f, z=%f",
             result ? "satisfied" : "violated", link_model_->getName().c_str(),
             q_des.x(), q_des.y(), q_des.z(), q_des.w(),
             q_act.x(), q_act.y(), q_act.z(), q_act.w(), xyz(0), xyz(1), xyz(2),
             absolute_x_axis_tolerance_, absolute_y_axis_tolerance_, absolute_z_axis_tolerance_);
  }
  
  return ConstraintEvaluationResult(result, constraint_weight_ * (xyz(0) + xyz(1) + xyz(2)));
}

void kinematic_constraints::OrientationConstraint::print(std::ostream &out) const
{
  if (link_model_)
  {
    out << "Orientation constraint on link '" << link_model_->getName() << "'" << std::endl;
    Eigen::Quaterniond q_des(desired_rotation_matrix_);
    out << "Desired orientation:" << q_des.x() << "," <<  q_des.y() << ","  <<  q_des.z() << "," << q_des.w() << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}

kinematic_constraints::VisibilityConstraint::VisibilityConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
  KinematicConstraint(model, tf), collision_robot_(new collision_detection::CollisionRobotFCL(model))
{
  type_ = VISIBILITY_CONSTRAINT;
}

void kinematic_constraints::VisibilityConstraint::clear(void)
{
  target_radius_ = -1.0;
}

bool kinematic_constraints::VisibilityConstraint::configure(const moveit_msgs::VisibilityConstraint &vc)
{
  target_radius_ = fabs(vc.target_radius);
  
  if (vc.target_radius <= std::numeric_limits<double>::epsilon())
    ROS_WARN("The radius of the target disc that must be visible should be positive");
  
  if (vc.cone_sides < 3)
  {
    ROS_WARN("The number of sides for the visibility region must be 3 or more. Assuming 3 sides instead of the specified %d", vc.cone_sides);
    cone_sides_ = 3;
  }
  else
    cone_sides_ = vc.cone_sides;
  
  // compute the points on the base circle of the cone that make up the cone sides
  points_.clear();
  double delta = 2.0 * boost::math::constants::pi<double>() / (double)cone_sides_;
  double a = 0.0;
  for (unsigned int i = 0 ; i < cone_sides_ ; ++i, a += delta)
  {
    double x = sin(a) * target_radius_;
    double y = cos(a) * target_radius_;
    points_.push_back(Eigen::Vector3d(x, y, 0.0));
  }
  
  if (!planning_models::poseFromMsg(vc.target_pose.pose, target_pose_))
    ROS_WARN("Incorrect specification of orientation in target pose for visibility constraint. Assuming identity quaternion.");
  
  if (tf_->isFixedFrame(vc.target_pose.header.frame_id))
  {
    tf_->transformPose(vc.target_pose.header.frame_id, target_pose_, target_pose_);
    target_frame_id_ = tf_->getTargetFrame();
    mobile_target_frame_ = false;
    // transform won't change, so apply it now
    for (std::size_t i = 0 ; i < points_.size() ; ++i)
      points_[i] = target_pose_*points_[i];
  }
  else
  {
    target_frame_id_ = vc.target_pose.header.frame_id;
    mobile_target_frame_ = true;
  }
  
  if (!planning_models::poseFromMsg(vc.sensor_pose.pose, sensor_pose_))
    ROS_WARN("Incorrect specification of orientation in sensor pose for visibility constraint. Assuming identity quaternion.");
  
  if (tf_->isFixedFrame(vc.sensor_pose.header.frame_id))
  {
    tf_->transformPose(vc.sensor_pose.header.frame_id, sensor_pose_, sensor_pose_);
    sensor_frame_id_ = tf_->getTargetFrame();
    mobile_sensor_frame_ = false;
  }
  else
  {
    sensor_frame_id_ = vc.sensor_pose.header.frame_id;
    mobile_sensor_frame_ = true;
  }
  
  if (vc.weight <= std::numeric_limits<double>::epsilon())
    ROS_WARN_STREAM("The weight of visibility constraints should be positive");
  else
    constraint_weight_ = vc.weight;
  
  max_view_angle_ = vc.max_view_angle;
  max_range_angle_ = vc.max_range_angle;
  sensor_view_direction_ = vc.sensor_view_direction;
  
  return target_radius_ > std::numeric_limits<double>::epsilon();
}

bool kinematic_constraints::VisibilityConstraint::equal(const KinematicConstraint &other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const VisibilityConstraint &o = static_cast<const VisibilityConstraint&>(other);
  
  if (target_frame_id_ == o.target_frame_id_ && sensor_frame_id_ == o.sensor_frame_id_ &&
      cone_sides_ == o.cone_sides_ && sensor_view_direction_ == o.sensor_view_direction_)
  {
    if (fabs(max_view_angle_ - o.max_view_angle_) > margin ||
        fabs(target_radius_ - o.target_radius_) > margin)
      return false;
    Eigen::Affine3d diff = sensor_pose_.inverse() * o.sensor_pose_;
    if (diff.translation().norm() > margin)
      return false;
    if (!diff.rotation().isIdentity(margin))
      return false;
    diff = target_pose_.inverse() * o.target_pose_;
    if (diff.translation().norm() > margin)
      return false;
    if (!diff.rotation().isIdentity(margin))
      return false;
    return true;
  }
  return false;
}

bool kinematic_constraints::VisibilityConstraint::enabled(void) const
{
  return target_radius_ > std::numeric_limits<double>::epsilon();
}

shapes::Mesh* kinematic_constraints::VisibilityConstraint::getVisibilityCone(const planning_models::KinematicState &state) const
{
  // the current pose of the sensor
  const Eigen::Affine3d &sp = mobile_sensor_frame_ ? tf_->getTransform(state, sensor_frame_id_) * sensor_pose_ : sensor_pose_;
  const Eigen::Affine3d &tp = mobile_target_frame_ ? tf_->getTransform(state, target_frame_id_) * target_pose_ : target_pose_;
  
  // transform the points on the disc to the desired target frame
  const EigenSTL::vector_Vector3d *points = &points_;
  boost::scoped_ptr<EigenSTL::vector_Vector3d> tempPoints;
  if (mobile_target_frame_)
  {
    tempPoints.reset(new EigenSTL::vector_Vector3d(points_.size()));
    for (std::size_t i = 0 ; i < points_.size() ; ++i)
      tempPoints->at(i) = tp*points_[i];
    points = tempPoints.get();
  }
  
  // allocate memory for a mesh to represent the visibility cone
  shapes::Mesh *m = new shapes::Mesh();
  m->vertex_count = cone_sides_ + 2;
  m->vertices = new double[m->vertex_count * 3];
  m->triangle_count = cone_sides_ * 2;
  m->triangles = new unsigned int[m->triangle_count * 3];
  // we do NOT allocate normals because we do not compute them
  
  // the sensor origin
  m->vertices[0] = sp.translation().x();
  m->vertices[1] = sp.translation().y();
  m->vertices[2] = sp.translation().z();
  
  // the center of the base of the cone approximation
  m->vertices[3] = tp.translation().x();
  m->vertices[4] = tp.translation().y();
  m->vertices[5] = tp.translation().z();
  
  // the points that approximate the base disc
  for (std::size_t i = 0 ; i < points->size() ; ++i)
  {
    m->vertices[i*3 + 6] = points->at(i).x();
    m->vertices[i*3 + 7] = points->at(i).y();
    m->vertices[i*3 + 8] = points->at(i).z();
  }
  
  // add the triangles
  std::size_t p3 = points->size() * 3;
  for (std::size_t i = 1 ; i < points->size() ; ++i)
  {
    // triangle forming a side of the cone, using the sensor origin
    std::size_t i3 = (i - 1) * 3;
    m->triangles[i3] = i + 1;
    m->triangles[i3 + 1] = 0;
    m->triangles[i3 + 2] = i + 2;
    // triangle forming a part of the base of the cone, using the center of the base
    std::size_t i6 = p3 + i3;
    m->triangles[i6] = i + 1;
    m->triangles[i6 + 1] = 1;
    m->triangles[i6 + 2] = i + 2;
  }
  
  // last triangles
  m->triangles[p3 - 3] = points->size() + 1;
  m->triangles[p3 - 2] = 0;
  m->triangles[p3 - 1] = 2;
  p3 *= 2;
  m->triangles[p3 - 3] = points->size() + 1;
  m->triangles[p3 - 2] = 1;
  m->triangles[p3 - 1] = 2;
  
  return m;
}

void kinematic_constraints::VisibilityConstraint::getMarkers(const planning_models::KinematicState &state, visualization_msgs::MarkerArray &markers) const
{
  shapes::Mesh *m = getVisibilityCone(state);
  visualization_msgs::Marker mk;
  shapes::constructMarkerFromShape(m, mk);
  delete m;
  mk.header.frame_id = kmodel_->getModelFrame();
  mk.header.stamp = ros::Time::now();
  mk.ns = "constraints";
  mk.id = 1;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.position.x = 0;
  mk.pose.position.y = 0;
  mk.pose.position.z = 0;
  mk.pose.orientation.x = 0;
  mk.pose.orientation.y = 0;
  mk.pose.orientation.z = 0;
  mk.pose.orientation.w = 1;
  mk.lifetime = ros::Duration(60);
  mk.color.a = 0.5;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  markers.markers.push_back(mk);
  
  const Eigen::Affine3d &sp = mobile_sensor_frame_ ? tf_->getTransform(state, sensor_frame_id_) * sensor_pose_ : sensor_pose_;
  const Eigen::Affine3d &tp = mobile_target_frame_ ? tf_->getTransform(state, target_frame_id_) * target_pose_ : target_pose_;
  
  visualization_msgs::Marker mka;
  mka.type = visualization_msgs::Marker::LINE_STRIP;
  mka.action = visualization_msgs::Marker::ADD;
  mka.color = mk.color;
  mka.pose = mk.pose;
  
  mka.header = mk.header;
  mka.ns = mk.ns;
  mka.id = 2;
  mka.lifetime = mk.lifetime;
  mka.scale.x = mka.scale.y = mka.scale.z = 0.05;
  mka.colors.resize(2, mk.color);
  mka.colors[1].g = 1.0;
  mka.colors[1].r = 0.0;
  mka.points.resize(2);  
  Eigen::Vector3d d = tp.translation() - tp.rotation().col(2) * 0.5;
  mka.points[0].x = tp.translation().x();
  mka.points[0].y = tp.translation().y();
  mka.points[0].z = tp.translation().z();
  mka.points[1].x = d.x();
  mka.points[1].y = d.y();
  mka.points[1].z = d.z();
  markers.markers.push_back(mka);
  
  mka.id = 3;  
  d = sp.translation() + sp.rotation().col(0) * 0.5; 
  mka.points[0].x = sp.translation().x();
  mka.points[0].y = sp.translation().y();
  mka.points[0].z = sp.translation().z();
  mka.points[1].x = d.x();
  mka.points[1].y = d.y();
  mka.points[1].z = d.z();
  
  markers.markers.push_back(mka);
}

kinematic_constraints::ConstraintEvaluationResult kinematic_constraints::VisibilityConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
  if (target_radius_ <= std::numeric_limits<double>::epsilon())
    return ConstraintEvaluationResult(true, 0.0);
  
  if (max_view_angle_ > 0.0 || max_range_angle_ > 0.0)
  {
    const Eigen::Affine3d &sp = mobile_sensor_frame_ ? tf_->getTransform(state, sensor_frame_id_) * sensor_pose_ : sensor_pose_;
    const Eigen::Affine3d &tp = mobile_target_frame_ ? tf_->getTransform(state, target_frame_id_) * target_pose_ : target_pose_;
    const Eigen::Vector3d &normal2 = sp.rotation().col(sensor_view_direction_);

    if (max_view_angle_ > 0.0)
    {
      const Eigen::Vector3d &normal1 = tp.rotation().col(2); // along Z axis
      double dp = normal2.dot(normal1);
      if (dp < 0.0)
      { 
        if (verbose)
	    ROS_INFO("Visibility constraint is violated because the sensor is looking at the wrong side");
        return ConstraintEvaluationResult(false, 0.0);
      }
      double ang = acos(dp);
      if (max_view_angle_ < ang)
      {
	if (verbose)
	  ROS_INFO("Visibility constraint is violated because the view angle is %lf (above the maximum allowed of %lf)", ang, max_view_angle_);
        return ConstraintEvaluationResult(false, 0.0);
      }
    }
    if (max_range_angle_ > 0.0)
    {
      const Eigen::Vector3d &dir = (tp.translation() - sp.translation()).normalized();
      double dp = normal2.dot(dir); 
      if (dp < 0.0)
      { 
        if (verbose)
	    ROS_INFO("Visibility constraint is violated because the sensor is looking at the wrong side");
        return ConstraintEvaluationResult(false, 0.0);
      }
      
      double ang = acos(dp);
      if (max_range_angle_ < ang)
      {
	if (verbose)
	  ROS_INFO("Visibility constraint is violated because the range angle is %lf (above the maximum allowed of %lf)", ang, max_view_angle_);
        return ConstraintEvaluationResult(false, 0.0);
      }
    }
  }
  
  shapes::Mesh *m = getVisibilityCone(state);
  if (!m)
    return ConstraintEvaluationResult(false, 0.0);

  // add the visibility cone as an object
  collision_detection::CollisionWorldFCL collision_world;
  collision_world.addToObject("cone", shapes::ShapeConstPtr(m), Eigen::Affine3d::Identity());

  // check for collisions between the robot and the cone
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  collision_detection::AllowedCollisionMatrix acm;
  acm.setDefaultEntry("cone", boost::bind(&VisibilityConstraint::decideContact, this, _1));
  req.contacts = true;
  req.verbose = verbose;
  req.max_contacts = 1; 
  collision_world.checkRobotCollision(req, res, *collision_robot_, state, acm);

  if (verbose)
  {
    std::stringstream ss;
    m->print(ss);
    ROS_INFO("Visibility constraint %ssatisfied. Visibility cone approximation:\n %s", res.collision ? "not " : "", ss.str().c_str());
  }
  
  return ConstraintEvaluationResult(!res.collision, res.collision ? res.contacts.begin()->second.front().depth : 0.0);
}

bool kinematic_constraints::VisibilityConstraint::decideContact(collision_detection::Contact &contact) const
{
    if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_ATTACHED ||
	contact.body_type_2 == collision_detection::BodyTypes::ROBOT_ATTACHED)
	return true;
    if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_LINK &&
	contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT &&
	(contact.body_name_1 == sensor_frame_id_ || contact.body_name_1 == target_frame_id_))
	return true;
    if (contact.body_type_2 == collision_detection::BodyTypes::ROBOT_LINK &&
	contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT &&
	(contact.body_name_2 == sensor_frame_id_ || contact.body_name_2 == target_frame_id_))
	return true;    
    return false;
}

void kinematic_constraints::VisibilityConstraint::print(std::ostream &out) const
{
  if (enabled())
  {
    out << "Visibility constraint for sensor in frame '" << sensor_frame_id_ << "' using target in frame '" << target_frame_id_ << "'" << std::endl;
    out << "Target radius: " << target_radius_ << ", using " << cone_sides_ << " sides." << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}

void kinematic_constraints::KinematicConstraintSet::clear(void)
{
  all_constraints_ = moveit_msgs::Constraints();
  kinematic_constraints_.clear();
  joint_constraints_.clear();
  position_constraints_.clear();
  orientation_constraints_.clear();
  visibility_constraints_.clear();
}

bool kinematic_constraints::KinematicConstraintSet::add(const std::vector<moveit_msgs::JointConstraint> &jc)
{
  bool result = true;
  for (unsigned int i = 0 ; i < jc.size() ; ++i)
  {
    JointConstraint *ev = new JointConstraint(kmodel_, tf_);
    bool u = ev->configure(jc[i]);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    joint_constraints_.push_back(jc[i]);
    all_constraints_.joint_constraints.push_back(jc[i]);
  }
  return result;
}

bool kinematic_constraints::KinematicConstraintSet::add(const std::vector<moveit_msgs::PositionConstraint> &pc)
{
  bool result = true;
  for (unsigned int i = 0 ; i < pc.size() ; ++i)
  {
    PositionConstraint *ev = new PositionConstraint(kmodel_, tf_);
    bool u = ev->configure(pc[i]);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    position_constraints_.push_back(pc[i]);
    all_constraints_.position_constraints.push_back(pc[i]);
  }
  return result;
}

bool kinematic_constraints::KinematicConstraintSet::add(const std::vector<moveit_msgs::OrientationConstraint> &oc)
{
  bool result = true;
  for (unsigned int i = 0 ; i < oc.size() ; ++i)
  {
    OrientationConstraint *ev = new OrientationConstraint(kmodel_, tf_);
    bool u = ev->configure(oc[i]);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    orientation_constraints_.push_back(oc[i]);
    all_constraints_.orientation_constraints.push_back(oc[i]);
  }
  return result;
}

bool kinematic_constraints::KinematicConstraintSet::add(const std::vector<moveit_msgs::VisibilityConstraint> &vc)
{
  bool result = true;
  for (unsigned int i = 0 ; i < vc.size() ; ++i)
  {
    VisibilityConstraint *ev = new VisibilityConstraint(kmodel_, tf_);
    bool u = ev->configure(vc[i]);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    visibility_constraints_.push_back(vc[i]);
    all_constraints_.visibility_constraints.push_back(vc[i]);
  }
  return result;
}

bool kinematic_constraints::KinematicConstraintSet::add(const moveit_msgs::Constraints &c)
{
  bool j = add(c.joint_constraints);
  bool p = add(c.position_constraints);
  bool o = add(c.orientation_constraints);
  bool v = add(c.visibility_constraints);
  return j && p && o && v;
}

kinematic_constraints::ConstraintEvaluationResult kinematic_constraints::KinematicConstraintSet::decide(const planning_models::KinematicState &state, bool verbose) const
{
  ConstraintEvaluationResult res(true, 0.0);
  for (unsigned int i = 0 ; i < kinematic_constraints_.size() ; ++i)
  {
    ConstraintEvaluationResult r = kinematic_constraints_[i]->decide(state, verbose);
    if (!r.satisfied)
      res.satisfied = false;
    res.distance += r.distance;
  }
  return res;
}

kinematic_constraints::ConstraintEvaluationResult kinematic_constraints::KinematicConstraintSet::decide(const planning_models::KinematicState &state,
                                                                                                        std::vector<ConstraintEvaluationResult> &results, 
                                                                                                        bool verbose) const
{
  ConstraintEvaluationResult result(true, 0.0);
  results.resize(joint_constraints_.size());
  for (std::size_t i = 0 ; i < joint_constraints_.size() ; ++i)
  {
    results[i] = kinematic_constraints_[i]->decide(state, verbose);
    result.satisfied = result.satisfied && results[i].satisfied;
    result.distance += results[i].distance;
  }
  
  return result;
}

void kinematic_constraints::KinematicConstraintSet::print(std::ostream &out) const
{
  out << kinematic_constraints_.size() << " kinematic constraints" << std::endl;
  for (unsigned int i = 0 ; i < kinematic_constraints_.size() ; ++i)
    kinematic_constraints_[i]->print(out);
}

bool kinematic_constraints::KinematicConstraintSet::equal(const KinematicConstraintSet &other, double margin) const
{
  for (unsigned int i = 0 ; i < kinematic_constraints_.size() ; ++i)
  {
    bool found = false;
    for (unsigned int j = 0 ; !found && j < other.kinematic_constraints_.size() ; ++j)
      found = kinematic_constraints_[i]->equal(*other.kinematic_constraints_[j], margin);
    if (!found)
      return false;
  }
  return true;
}

