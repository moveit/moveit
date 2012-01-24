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

#ifndef _INTERACTIVE_OBJECT_VISUALIZATION_H_
#define _INTERACTIVE_OBJECT_VISUALIZATION_H_

#include <ros/ros.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <moveit_visualization_ros/interactive_marker_helper_functions.h>

static const double DEFAULT_SCALE = .1;
static const double DEFAULT_X = .5;
static const double DEFAULT_Y = 0.0;
static const double DEFAULT_Z = .5;

namespace moveit_visualization_ros
{

class InteractiveObjectVisualization {
public:
  
  InteractiveObjectVisualization(planning_scene::PlanningSceneConstPtr planning_scene,
                                 boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                 const std_msgs::ColorRGBA& color); 

  ~InteractiveObjectVisualization() {
  }

  planning_scene::PlanningSceneConstPtr addCube(const std::string& name="");
  planning_scene::PlanningSceneConstPtr addCylinder(const std::string& name="");
  planning_scene::PlanningSceneConstPtr addSphere(const std::string& name="");

  void setUpdateCallback(const boost::function<void(planning_scene::PlanningSceneConstPtr)>& callback);

  void updateObjectPose(const std::string& name,
                        const geometry_msgs::Pose& pose);

protected:

  std::string generateNewCubeName() {
    std::stringstream iss;
    iss << "Cube_"<<cube_counter_++;
    return iss.str();
  }

  std::string generateNewSphereName() {
    std::stringstream iss;
    iss << "Sphere_"<<sphere_counter_++;
    return iss.str();
  }

  std::string generateNewCylinderName() {
    std::stringstream iss;
    iss << "Cylinder_"<<cylinder_counter_++;
    return iss.str();
  }

  planning_scene::PlanningSceneConstPtr addObject(const std::string& name,
                                                  shapes::Shape* shape);

  void callUpdateCallback();

  void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback); 
  
  planning_scene::PlanningSceneConstPtr planning_scene_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;

  planning_scene::PlanningScenePtr planning_scene_diff_;

  boost::function<void(planning_scene::PlanningSceneConstPtr)> update_callback_;

  unsigned int cube_counter_;
  unsigned int sphere_counter_;
  unsigned int cylinder_counter_;

};

}

#endif
