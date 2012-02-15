/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

// Author: E. Gil Jones

#ifndef _PLANNING_VISUALIZATION_QT_WRAPPER_H_
#define _PLANNING_VISUALIZATION_QT_WRAPPER_H_

#include <QObject>
#include <QString>

#include <moveit_visualization_ros/planning_visualization.h>
 
namespace moveit_visualization_ros
{

class PlanningVisualizationQtWrapper : public QObject, public PlanningVisualization
{
  Q_OBJECT
public:
  
  PlanningVisualizationQtWrapper(planning_scene::PlanningSceneConstPtr planning_scene,
                                 const std::map<std::string, std::vector<moveit_msgs::JointLimits> >& group_joint_limits_map,
                                 boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                 ros::Publisher& marker_publisher) :
    PlanningVisualization(planning_scene, group_joint_limits_map, interactive_marker_server, marker_publisher)
  {
  }

  ~PlanningVisualizationQtWrapper() {
  }

public Q_SLOTS: 

  void newGroupSelected(const QString&);

};

}

#endif
