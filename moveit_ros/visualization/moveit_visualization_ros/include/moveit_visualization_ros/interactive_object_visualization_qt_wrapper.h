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

#ifndef _INTERACTIVE_OBJECT_VISUALIZATION_QT_WRAPPER_H_
#define _INTERACTIVE_OBJECT_VISUALIZATION_QT_WRAPPER_H_

#include <ros/ros.h>
#include <QObject>
#include <QColor>
#include <QMetaType>

#include <moveit_visualization_ros/interactive_object_visualization.h>
#include <moveit_msgs/CollisionObject.h>
 
namespace moveit_visualization_ros
{

class InteractiveObjectVisualizationQtWrapper : public QObject, public InteractiveObjectVisualization 
{
  Q_OBJECT
public:
  
  InteractiveObjectVisualizationQtWrapper(planning_scene::PlanningSceneConstPtr planning_scene,
                                          boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                          const std_msgs::ColorRGBA& color) :
    InteractiveObjectVisualization(planning_scene, interactive_marker_server, color)
  {
    qRegisterMetaType<planning_scene::PlanningSceneConstPtr>("PlanningSceneConstPtr");
  }

  ~InteractiveObjectVisualizationQtWrapper() {
  }

  virtual void callUpdateCallback();

public Q_SLOTS: 

  void addCubeSignalled();

  void deleteSignalled(std::string);

  void addCollisionObjectSignalled(const moveit_msgs::CollisionObject&,
                                   const QColor&);

  void attachCollisionObjectSignalled(const std::string& name,
                                      const std::string& link_name,
                                      const std::vector<std::string>& touch_links);

  void loadPlanningSceneSignalled(moveit_msgs::PlanningScenePtr);

Q_SIGNALS:
  
  void updatePlanningSceneSignal(planning_scene::PlanningSceneConstPtr);

};

}

#endif
