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

#ifndef MOVEIT_ROBOT_INTERACTION_ROBOT_INTERACTION_
#define MOVEIT_ROBOT_INTERACTION_ROBOT_INTERACTION_

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <planning_models/kinematic_state.h>

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace robot_interaction
{
  
class RobotInteraction
{
public:
  
  static const std::string INTERACTIVE_MARKER_TOPIC;
  
  struct EndEffector
  {
    std::string group;
    std::string eef_group;
    std::string tip_link;
    double scale;
  };

  struct VirtualJoint
  { 
    std::string connecting_link;
    std::string joint_name;
    unsigned int dof;
    double scale;
  };
  
  class InteractionHandler
  {
  public:
    InteractionHandler(void)
    {
    }
    
    virtual ~InteractionHandler(void)
    {
    }
    
    virtual void handleEndEffector(const RobotInteraction::EndEffector& eef, int id, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) = 0;
    virtual void handleVirtualJoint(const RobotInteraction::VirtualJoint& vj, int id, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) = 0;
    virtual bool inError(const RobotInteraction::EndEffector& eef, int id) = 0;
    virtual bool inError(const RobotInteraction::VirtualJoint& vj, int id) = 0;
  };

  typedef boost::shared_ptr<InteractionHandler> InteractionHandlerPtr;
  typedef boost::shared_ptr<const InteractionHandler> InteractionHandlerConstPtr;
  
  RobotInteraction(const planning_models::KinematicModelConstPtr &kmodel,
                   const InteractionHandlerPtr &handler = InteractionHandlerPtr());
  ~RobotInteraction(void);
  
  void setInteractionHandler(const InteractionHandlerPtr &handler)
  {
    handler_ = handler;
  }

  const InteractionHandlerPtr& getInteractionHandler(void) const
  {
    return handler_;
  }
  
  void decideActiveComponents(const std::string &group);
  void decideActiveEndEffectors(const std::string &group);
  void decideActiveVirtualJoints(const std::string &group);
  
  void clear(void);
  
  void addInteractiveMarkers(const planning_models::KinematicState &state, int id);
  void publishInteractiveMarkers(void);
  void clearInteractiveMarkers(void);
  
  const std::vector<EndEffector>& getActiveEndEffectors(void) const
  {
    return active_eef_;
  }

  const std::vector<VirtualJoint>& getActiveVirtualJoints(void) const
  {
    return active_vj_;
  }
  
  static bool updateState(planning_models::KinematicState &state, const EndEffector &eef, const geometry_msgs::Pose &pose);
  static bool updateState(planning_models::KinematicState &state, const VirtualJoint &vj, const geometry_msgs::Pose &pose);
  
private:
  
  // return the diameter of the sphere that certainly can enclose the AABB of the links in this group
  double computeGroupScale(const std::string &group);    
  void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);  
  
  planning_models::KinematicModelConstPtr kmodel_;
  InteractionHandlerPtr handler_;
  
  std::vector<EndEffector> active_eef_;
  std::vector<VirtualJoint> active_vj_;

  std::map<std::string, std::size_t> shown_markers_;
  interactive_markers::InteractiveMarkerServer *int_marker_server_;
};

  
}

#endif
