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

#ifndef MOVEIT_RVIZ_PLUGIN_ROBOT_INTERACTION_
#define MOVEIT_RVIZ_PLUGIN_ROBOT_INTERACTION_

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <planning_models/kinematic_state.h>

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace rviz
{
class DisplayContext;
}


namespace moveit_rviz_plugin
{
class PlanningDisplay;

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
  
  RobotInteraction(PlanningDisplay *pdisplay, rviz::DisplayContext *context);
  ~RobotInteraction(void);
  

  void decideActiveComponents(void);
  void decideActiveEndEffectors(void);
  void decideActiveVirtualJoints(void);
  
  void clear(void);
  
  void publishInteractiveMarkers(void);
  void computeMetrics(double payload);
  void computeMetrics(bool start, const std::string &group, double payload);

  const std::map<std::string, double>& getComputedMetrics(bool start, const std::string &group, double payload)
  {
    return computed_metrics_[std::make_pair(start, group)];
  }
  
  const std::vector<EndEffector>& getActiveEndEffectors(void) const
  {
    return active_eef_;
  }

  const std::vector<VirtualJoint>& getActiveVirtualJoints(void) const
  {
    return active_vj_;
  }
  
private:
  
  // return the diameter of the sphere that certainly can enclose the AABB of the links in this group
  double computeGroupScale(const std::string &group);    
  void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);  
  void computeProcessInteractiveMarkerFeedback(visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);
  
  void computeMetricsInternal(std::map<std::string, double> &metrics, const EndEffector &eef, const planning_models::KinematicState &state, double payload);
  
  PlanningDisplay *planning_display_;  
  rviz::DisplayContext* context_;
  
  std::vector<EndEffector> active_eef_;
  std::vector<VirtualJoint> active_vj_;
  
  /// The metrics are pairs of name-value for each of the active end effectors, for both start & goal states.
  /// computed_metrics_[std::make_pair(IS_START_STATE, GROUP_NAME)] = a map of key-value pairs
  std::map<std::pair<bool, std::string>, std::map<std::string, double> > computed_metrics_;
  
  std::set<std::string> invalid_start_state_;
  std::set<std::string> invalid_goal_state_;
  
  std::map<std::string, std::size_t> shown_markers_;
  interactive_markers::InteractiveMarkerServer *int_marker_server_;
};

  
}

#endif
