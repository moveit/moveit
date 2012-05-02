/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef _MOVEIT_MANIPULATION_VISUALIZER_H_
#define _MOVEIT_MANIPULATION_VISUALIZER_H_

#include <moveit_visualization_ros/moveit_visualizer.h>
#include <moveit_manipulation_visualization/grasp_evaluation_visualization_dialog.h>
#include <moveit_manipulation_visualization/object_recognition_qt_service_wrapper.h>
#include <moveit_manipulation_visualization/object_recognition_dialog.h>

namespace moveit_manipulation_visualization {

class MoveItManipulationVisualizer : public moveit_visualization_ros::MoveItVisualizer {

public:
  
  MoveItManipulationVisualizer();
  
  virtual ~MoveItManipulationVisualizer(){};
  
  virtual void updatePlanningScene(planning_scene::PlanningSceneConstPtr planning_scene);

  void attemptToGrasp(const std::string& object_name);

public Q_SLOTS:

  void addCabinet();
  
Q_SIGNALS:
 
  void addCollisionObjectRequested(const moveit_msgs::CollisionObject& obj,
                                   const QColor& color);
 

protected:

  //void attemptToGraspThread(const std::string& object_name);
  
  GraspEvaluationVisualizationDialog* grasp_evaluation_visualization_dialog_;

  boost::shared_ptr<ObjectRecognitionQtServiceWrapper> object_recognition_qt_service_wrapper_;
};

}

#endif
