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

#ifndef MOVEIT_RVIZ_PLUGIN_PLANNING_MARKERS_
#define MOVEIT_RVIZ_PLUGIN_PLANNING_MARKERS_

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <planning_models/kinematic_model.h>

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

class PlanningMarkers
{
public:

  struct IKMarker
  {
    std::string group;
    std::string eef_group;
    std::string tip_link;
    double scale;
  };

  PlanningMarkers(PlanningDisplay *pdisplay, rviz::DisplayContext *context);
  ~PlanningMarkers(void);
  
  void clear(void);
  
  void decideInteractiveMarkers(void);
  void publishInteractiveMarkers(void);
  
private:
  
  void computeMarkerScale(IKMarker &ik_marker);
  void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);  
  void updateMetrics(std::map<std::string,double> &metrics, const IKMarker &im, bool start_state);
  
  PlanningDisplay *planning_display_;  
  rviz::DisplayContext* context_;
  
  std::vector<IKMarker> ik_markers_;
  std::map<std::string, std::size_t> shown_markers_;
  interactive_markers::InteractiveMarkerServer* int_marker_server_;
};

  
}

#endif
