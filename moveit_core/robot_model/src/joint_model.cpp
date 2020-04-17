/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Ioan A. Sucan
*  Copyright (c) 2008-2013, Willow Garage, Inc.
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

#include <moveit/exceptions/exceptions.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/link_model.h>
#include <algorithm>

namespace moveit
{
namespace core
{
JointModel::JointModel(const std::string& name)
  : name_(name)
  , type_(UNKNOWN)
  , parent_link_model_(nullptr)
  , child_link_model_(nullptr)
  , mimic_(nullptr)
  , mimic_factor_(1.0)
  , mimic_offset_(0.0)
  , passive_(false)
  , distance_factor_(1.0)
  , first_variable_index_(-1)
  , joint_index_(-1)
{
}

JointModel::~JointModel() = default;

std::string JointModel::getTypeName() const
{
  switch (type_)
  {
    case UNKNOWN:
      return "Unkown";
    case REVOLUTE:
      return "Revolute";
    case PRISMATIC:
      return "Prismatic";
    case PLANAR:
      return "Planar";
    case FLOATING:
      return "Floating";
    case FIXED:
      return "Fixed";
    default:
      return "[Unkown]";
  }
}

int JointModel::getLocalVariableIndex(const std::string& variable) const
{
  VariableIndexMap::const_iterator it = variable_index_map_.find(variable);
  if (it == variable_index_map_.end())
    throw Exception("Could not find variable '" + variable + "' to get bounds for within joint '" + name_ + "'");
  return it->second;
}

bool JointModel::harmonizePosition(double* values, const Bounds& other_bounds) const
{
  return false;
}

bool JointModel::enforceVelocityBounds(double* values, const Bounds& other_bounds) const
{
  bool change = false;
  for (std::size_t i = 0; i < other_bounds.size(); ++i)
    if (other_bounds[i].max_velocity_ < values[i])
    {
      values[i] = other_bounds[i].max_velocity_;
      change = true;
    }
    else if (other_bounds[i].min_velocity_ > values[i])
    {
      values[i] = other_bounds[i].min_velocity_;
      change = true;
    }
  return change;
}

bool JointModel::satisfiesVelocityBounds(const double* values, const Bounds& other_bounds, double margin) const
{
  for (std::size_t i = 0; i < other_bounds.size(); ++i)
    if (other_bounds[i].max_velocity_ + margin < values[i])
      return false;
    else if (other_bounds[i].min_velocity_ - margin > values[i])
      return false;
  return true;
}

const VariableBounds& JointModel::getVariableBounds(const std::string& variable) const
{
  return variable_bounds_[getLocalVariableIndex(variable)];
}

void JointModel::setVariableBounds(const std::string& variable, const VariableBounds& bounds)
{
  variable_bounds_[getLocalVariableIndex(variable)] = bounds;
  computeVariableBoundsMsg();
}

void JointModel::setVariableBounds(const std::vector<moveit_msgs::JointLimits>& jlim)
{
  for (std::size_t j = 0; j < variable_names_.size(); ++j)
    for (const moveit_msgs::JointLimits& joint_limit : jlim)
      if (joint_limit.joint_name == variable_names_[j])
      {
        variable_bounds_[j].position_bounded_ = joint_limit.has_position_limits;
        if (joint_limit.has_position_limits)
        {
          variable_bounds_[j].min_position_ = joint_limit.min_position;
          variable_bounds_[j].max_position_ = joint_limit.max_position;
        }
        variable_bounds_[j].velocity_bounded_ = joint_limit.has_velocity_limits;
        if (joint_limit.has_velocity_limits)
        {
          variable_bounds_[j].min_velocity_ = -joint_limit.max_velocity;
          variable_bounds_[j].max_velocity_ = joint_limit.max_velocity;
        }
        variable_bounds_[j].acceleration_bounded_ = joint_limit.has_acceleration_limits;
        if (joint_limit.has_acceleration_limits)
        {
          variable_bounds_[j].min_acceleration_ = -joint_limit.max_acceleration;
          variable_bounds_[j].max_acceleration_ = joint_limit.max_acceleration;
        }
        break;
      }
  computeVariableBoundsMsg();
}

void JointModel::computeVariableBoundsMsg()
{
  variable_bounds_msg_.clear();
  for (std::size_t i = 0; i < variable_bounds_.size(); ++i)
  {
    moveit_msgs::JointLimits lim;
    lim.joint_name = variable_names_[i];
    lim.has_position_limits = variable_bounds_[i].position_bounded_;
    lim.min_position = variable_bounds_[i].min_position_;
    lim.max_position = variable_bounds_[i].max_position_;
    lim.has_velocity_limits = variable_bounds_[i].velocity_bounded_;
    lim.max_velocity = std::min(fabs(variable_bounds_[i].min_velocity_), fabs(variable_bounds_[i].max_velocity_));
    lim.has_acceleration_limits = variable_bounds_[i].acceleration_bounded_;
    lim.max_acceleration =
        std::min(fabs(variable_bounds_[i].min_acceleration_), fabs(variable_bounds_[i].max_acceleration_));
    variable_bounds_msg_.push_back(lim);
  }
}

void JointModel::setMimic(const JointModel* mimic, double factor, double offset)
{
  mimic_ = mimic;
  mimic_factor_ = factor;
  mimic_offset_ = offset;
}

void JointModel::addMimicRequest(const JointModel* joint)
{
  mimic_requests_.push_back(joint);
}

void JointModel::addDescendantJointModel(const JointModel* joint)
{
  descendant_joint_models_.push_back(joint);
  if (joint->getType() != FIXED)
    non_fixed_descendant_joint_models_.push_back(joint);
}

void JointModel::addDescendantLinkModel(const LinkModel* link)
{
  descendant_link_models_.push_back(link);
}

namespace
{
inline void printBoundHelper(std::ostream& out, double v)
{
  if (v <= -std::numeric_limits<double>::infinity())
    out << "-inf";
  else if (v >= std::numeric_limits<double>::infinity())
    out << "inf";
  else
    out << v;
}
}  // namespace

std::ostream& operator<<(std::ostream& out, const VariableBounds& b)
{
  out << "P." << (b.position_bounded_ ? "bounded" : "unbounded") << " [";
  printBoundHelper(out, b.min_position_);
  out << ", ";
  printBoundHelper(out, b.max_position_);
  out << "]; "
      << "V." << (b.velocity_bounded_ ? "bounded" : "unbounded") << " [";
  printBoundHelper(out, b.min_velocity_);
  out << ", ";
  printBoundHelper(out, b.max_velocity_);
  out << "]; "
      << "A." << (b.acceleration_bounded_ ? "bounded" : "unbounded") << " [";
  printBoundHelper(out, b.min_acceleration_);
  out << ", ";
  printBoundHelper(out, b.max_acceleration_);
  out << "];";
  return out;
}


// Calculating hash for JointModel
namespace std
{
	template<> struct hash<JointModel>
	{
		std::size_t operator()(JointModel const& joint) const noexcept
		{
			std::size_t h =0;
      // use Joint name_
			boost::hash_combine(h, joint.name_);
      // use Joint type_: unknown/revolute/prismatic/planner/floating/fixed
			boost::hash_combine(h, std::hash<JointType>{}(joint.type_));

      // local_var_names: Do we want this for hash? Yep. This makes Index map in RobotModel
      // See RobotModel::buildJointInfo
			for (auto &local_var : local_variable_names_) {
				boost::hash_combine(h, std::hash<std::string>{}(local_var));
			}
      // Ignoring variable_names_

      // use Joint variable bounds 
      // //This is a vector of VariableBounds. They should be really Map<std::string, Bounds> where the key 
      // should be local_var_names_. So instead of initializing two variables (variable_names_ and 
      // variable_bounds_) I would just initialize this map. So no need for two loop to associate 
      // variable_bounds_ from the JointLimit to variable_names_ and then assign limits to VariableBounds. 
      // ANYHOW we don’t need to worry about the order of those, since the vector corresponds to the 
      // local_variable_names_ index!
			for (auto &var : variable_bounds_) {
				boost::hash_combine(h, std::hash<VariableBounds>{}(var));
			}

      // Ignoring JointLimits.msg
      // Ignoring variable_index_map_
_
      // use parent and child model links model
			boost::hash_combine(h, std::hash<LinkModel>{}(parent_link_model_->hash()));
			boost::hash_combine(h, std::hash<LinkModel>{}(child_link_model_->hash()));
      boost::hash_combine(h, std::hash<JointModel>{})(mimic_->hash());
      boost::hash_combine(h, mimic_factor_);
      boost::hash_combine(h, mimic_offset_);
      // Ignore mimic mimic_requests_
    
      // All the links/joins which would move/not move with this Joint
      for (auto &desc_link : descendant_link_models_) {
        boost::hash_combine(h, std::hash<LinkModel>{}(desc_link->hash()));
      }
      for (auto &desc_joint : descendant_joint_models_) {
        boost::hash_combine(h, std::hash<JointModel>{}(desc_joint->hash()));
      }
      for (auto &non_fixed : non_fixed_descendant_joint_models_) {
        boost::hash_combine(h, std::hash<JointModel>{}(non_fixed->hash()));
      }

      boost::hash_combine(h, joint.passive_);
      boost::hash_combine(h, joint.distance_factor_);
      boost::hash_combine(h, joint.first_variable_index_);
      boost::hash_combine(h, joint.joint_index_);
      return h;
		}
	}

// Calculating Hash for VariableBounds
  template<> struct hash<VariableBounds> {
		std::size_t operator()(VariableBounds const& varBound) const noexcept
    {
			std::size_t h = 0;
			boost::hash_combine(h, varBound.min_position_);
			boost::hash_combine(h, varBound.max_position_);
			boost::hash_combine(h, varBound.acceleration_bounded_);
			boost::hash_combine(h, varBound.min_acceleration_);
			boost::hash_combine(h, varBound.max_acceleration_);
			boost::hash_combine(h, varBound.position_bounded_);
			boost::hash_combine(h, varBound.min_velocity_);
			boost::hash_combine(h, varBound.max_velocity_);

      return h;
    }
} // end of namespace std

}  // end of namespace core
}  // end of namespace moveit
