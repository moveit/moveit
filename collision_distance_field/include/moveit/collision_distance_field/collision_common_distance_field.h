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

/* Author: E. Gil Jones */

#ifndef MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD_COLLISION_COMMON_
#define MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD_COLLISION_COMMON_

#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>

namespace collision_detection
{

struct DistanceFieldCacheEntry;

struct GroupStateRepresentation {
  GroupStateRepresentation() {};
  GroupStateRepresentation(const GroupStateRepresentation& gsr) {
    link_body_decompositions_.resize(gsr.link_body_decompositions_.size());
    for(unsigned int i = 0; i < gsr.link_body_decompositions_.size(); i++) {
      if(gsr.link_body_decompositions_[i]) {
        link_body_decompositions_[i].reset(new PosedBodySphereDecomposition(*gsr.link_body_decompositions_[i]));
      }
    }
    attached_body_decompositions_.resize(gsr.attached_body_decompositions_.size());
    for(unsigned int i = 0; i < gsr.attached_body_decompositions_.size(); i++) {
      (*attached_body_decompositions_[i]) = (*gsr.attached_body_decompositions_[i]);
    }
    gradients_ = gsr.gradients_;
  }
  boost::shared_ptr<const DistanceFieldCacheEntry> dfce_;
  std::vector<PosedBodySphereDecompositionPtr> link_body_decompositions_;
  std::vector<PosedBodySphereDecompositionVectorPtr> attached_body_decompositions_;
  std::vector<GradientInfo> gradients_;
};

struct DistanceFieldCacheEntry {
  std::string group_name_;
  boost::shared_ptr<robot_state::RobotState> state_;
  std::vector<unsigned int> state_check_indices_;
  std::vector<double> state_values_;
  collision_detection::AllowedCollisionMatrix acm_;
  boost::shared_ptr<distance_field::DistanceField> distance_field_;
  boost::shared_ptr<GroupStateRepresentation> pregenerated_group_state_representation_;
  std::vector<std::string> link_names_;
  std::vector<bool> link_has_geometry_;
  std::vector<unsigned int> link_body_indices_;
  std::vector<unsigned int> link_state_indices_;
  std::vector<std::string> attached_body_names_;
  std::vector<unsigned int> attached_body_link_state_indices_;
  std::vector<bool> self_collision_enabled_;
  std::vector<std::vector<bool> > intra_group_collision_enabled_;
};

BodyDecompositionConstPtr getBodyDecompositionCacheEntry(const shapes::ShapeConstPtr& shape,
                                                         double resolution);

PosedBodyPointDecompositionVectorPtr getCollisionObjectPointDecomposition(const collision_detection::CollisionWorld::Object& obj,
                                                                          double resolution);

PosedBodySphereDecompositionVectorPtr getAttachedBodySphereDecomposition(const robot_state::AttachedBody* att,
                                                                         double resolution);

PosedBodyPointDecompositionVectorPtr getAttachedBodyPointDecomposition(const robot_state::AttachedBody* att,
                                                                       double resolution);


}
#endif
