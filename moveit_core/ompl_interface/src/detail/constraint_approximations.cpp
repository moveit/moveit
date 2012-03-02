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

#include "ompl_interface/detail/constraint_approximations.h"
#include "ompl_interface/parameterization/model_based_state_space.h"
#include <random_numbers/random_numbers.h>

namespace ompl_interface
{
template<typename T>
void msgToHex(const T& msg, std::string &hex)
{
  static const char symbol[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  const size_t serial_size_arg = ros::serialization::serializationLength(msg);
  
  boost::shared_array<uint8_t> buffer_arg(new uint8_t[serial_size_arg]);
  ros::serialization::OStream stream_arg(buffer_arg.get(), serial_size_arg);
  ros::serialization::serialize(stream_arg, msg);
  hex.resize(serial_size_arg * 2);
  for (std::size_t i = 0 ; i < serial_size_arg ; ++i)
  {
    hex[i * 2] = symbol[buffer_arg[i]/16];
    hex[i * 2 + 1] = symbol[buffer_arg[i]%16];
  }
}

template<typename T>
void hexToMsg(const std::string &hex, T& msg)
{
  const size_t serial_size_arg = hex.length() / 2;
  boost::shared_array<uint8_t> buffer_arg(new uint8_t[serial_size_arg]);
  for (std::size_t i = 0 ; i < serial_size_arg ; ++i)
  {
    buffer_arg[i] =
      (hex[i * 2] <= '9' ? (hex[i * 2] - '0') : (hex[i * 2] - 'A' + 10)) * 16 +
      (hex[i * 2 + 1] <= '9' ? (hex[i * 2 + 1] - '0') : (hex[i * 2 + 1] - 'A' + 10));
  }
  ros::serialization::IStream stream_arg(buffer_arg.get(), serial_size_arg);
  ros::serialization::deserialize(stream_arg, msg);
}
}

ompl_interface::ConstraintApproximation::ConstraintApproximation(const planning_models::KinematicModelConstPtr &kinematic_model, const std::string &group,
                                                                 const std::string &factory, const std::string &serialization, const std::string &filename,
								 const ompl::base::StateStoragePtr &storage) :
  group_(group), factory_(factory), serialization_(serialization), kmodel_(kinematic_model), ompldb_filename_(filename), state_storage_ptr_(storage)
{
  hexToMsg(serialization, constraint_msg_);
  state_storage_ = static_cast<ConstraintApproximationStateStorage*>(state_storage_ptr_.get());
  state_storage_->getStateSpace()->computeSignature(space_signature_);
  kconstraints_set_.reset(new kinematic_constraints::KinematicConstraintSet(kinematic_model,
									    planning_models::TransformsConstPtr(new planning_models::Transforms(kinematic_model->getModelFrame()))));
  kconstraints_set_->add(constraint_msg_);
}

ompl_interface::ConstraintApproximation::ConstraintApproximation(const planning_models::KinematicModelConstPtr &kinematic_model, const std::string &group,
                                                                 const std::string &factory, const moveit_msgs::Constraints &msg, const std::string &filename,
								 const ompl::base::StateStoragePtr &storage) :
  group_(group), factory_(factory), constraint_msg_(msg), kmodel_(kinematic_model), ompldb_filename_(filename), state_storage_ptr_(storage)
{
  msgToHex(msg, serialization_);
  state_storage_ = static_cast<ConstraintApproximationStateStorage*>(state_storage_ptr_.get());
  state_storage_->getStateSpace()->computeSignature(space_signature_);
  kconstraints_set_.reset(new kinematic_constraints::KinematicConstraintSet(kinematic_model,
									    planning_models::TransformsConstPtr(new planning_models::Transforms(kinematic_model->getModelFrame()))));
  kconstraints_set_->add(msg);
}

void ompl_interface::ConstraintApproximation::visualizeDistribution(const std::string &link_name, unsigned int count, visualization_msgs::MarkerArray &arr) const
{    
  planning_models::KinematicState kstate(kmodel_);
  kstate.setToDefaultValues();
  
  ompl::RNG rng;
  std_msgs::ColorRGBA color;
  color.r = 0.0f;
  color.g = 1.0f;
  color.b = 1.0f;
  color.a = 1.0f;
  if (state_storage_->size() < count)
      count = state_storage_->size();
  
  for (std::size_t i = 0 ; i < count ; ++i)
  { 
      state_storage_->getStateSpace()->as<ModelBasedStateSpace>()->copyToKinematicState(kstate, state_storage_->getState(rng.uniformInt(0, state_storage_->size() - 1)));
      kstate.getJointStateGroup(group_)->updateLinkTransforms();
      const Eigen::Vector3d &pos = kstate.getLinkState(link_name)->getGlobalLinkTransform().translation();
      
      visualization_msgs::Marker mk;
      mk.header.stamp = ros::Time::now();
      mk.header.frame_id = kmodel_->getModelFrame();
      mk.ns = "stored_constraint_data";
      mk.id = i;
      mk.type = visualization_msgs::Marker::SPHERE;
      mk.action = visualization_msgs::Marker::ADD;
      mk.pose.position.x = pos.x();
      mk.pose.position.y = pos.y();
      mk.pose.position.z = pos.z();
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.035;
      mk.color = color;
      mk.lifetime = ros::Duration(30.0);
      arr.markers.push_back(mk);
  }
}

