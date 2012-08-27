/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Ioan Sucan */

#include <rviz/robot/link_updater.h>
#include <planning_models/kinematic_state.h>

namespace moveit_rviz_plugin
{

/** \brief */
class PlanningLinkUpdater : public rviz::LinkUpdater
{
public:
  
  // ******************************************************************************************
  // Sub class constructor
  // ******************************************************************************************
  PlanningLinkUpdater(const planning_models::KinematicStateConstPtr &state)
    : kinematic_state_(state)
  {
  }
  
  // ******************************************************************************************
  //
  // ******************************************************************************************
  virtual bool getLinkTransforms(const std::string& link_name, Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
                                 Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation) const
  {
    const planning_models::KinematicState::LinkState* link_state = kinematic_state_->getLinkState( link_name );
    
    if ( !link_state )
    {
      return false;
    }
    
    const Eigen::Vector3d &robot_visual_position = link_state->getGlobalLinkTransform().translation();
    Eigen::Quaterniond robot_visual_orientation(link_state->getGlobalLinkTransform().rotation());
    visual_position = Ogre::Vector3( robot_visual_position.x(), robot_visual_position.y(), robot_visual_position.z() );
    visual_orientation = Ogre::Quaternion( robot_visual_orientation.w(), robot_visual_orientation.x(), robot_visual_orientation.y(), robot_visual_orientation.z() );
    
    const Eigen::Vector3d &robot_collision_position = link_state->getGlobalCollisionBodyTransform().translation();
    Eigen::Quaterniond robot_collision_orientation(link_state->getGlobalCollisionBodyTransform().rotation());
    collision_position = Ogre::Vector3( robot_collision_position.x(), robot_collision_position.y(), robot_collision_position.z() );
    collision_orientation = Ogre::Quaternion( robot_collision_orientation.w(), robot_collision_orientation.x(), robot_collision_orientation.y(), robot_collision_orientation.z() );
    
    return true;
  }
  
  
  // ******************************************************************************************
  // Private
  // ******************************************************************************************
private:
  planning_models::KinematicStateConstPtr kinematic_state_;
  
};

}
