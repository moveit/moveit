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

#ifndef _KINEMATICS_START_GOAL_VISUALIZATION_H_
#define _KINEMATICS_START_GOAL_VISUALIZATION_H_

#include <moveit_visualization_ros/kinematics_group_visualization.h>
#include <kinematics_plugin_loader/kinematics_plugin_loader.h>

namespace moveit_visualization_ros
{

class KinematicsStartGoalVisualization {
public:
  
  KinematicsStartGoalVisualization(planning_scene::PlanningSceneConstPtr planning_scene,
                                   boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                   boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader>& kinematics_plugin_loader,
                                   const std::string& group_name, 
                                   ros::Publisher& marker_publisher,
                                   bool show = true); 

  ~KinematicsStartGoalVisualization() {
  }

  const planning_models::KinematicState& getStartState() const {
    if(start_chained_) {
      return planning_scene_->getCurrentState();
    } else {
      return start_->getState();
    }
  }

  const planning_models::KinematicState& getGoalState() const {
    return goal_->getState();
  }

  void updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);

  void resetStartState();

  void addMenuEntry(const std::string& name, 
                    const boost::function<void(const std::string&)>& callback);

  void setRandomStartGoal();
  void resetStartGoal();

  void setStartState(const planning_models::KinematicState& state);
  void setGoalState(const planning_models::KinematicState& state);

  void setGoodBadMode(bool use_good_bad) {
    start_->setGoodBadMode(use_good_bad);
    goal_->setGoodBadMode(use_good_bad);
  }

  void hideRegularMarkers() {
    if(!start_chained_) {
      start_->hideRegularMarkers();
    }
    goal_->hideRegularMarkers();
  }
  void showRegularMarkers() {
    if(!start_chained_) {
      start_->showRegularMarkers();
    }
    goal_->showRegularMarkers();
  }

  void hideAllMarkers();
  void showAllMarkers();

  void setChainStartToCurrent(bool to_chain)
  {
    start_chained_ = to_chain;
    if(to_chain) {
      start_->hideAllMarkers();
    }
  }

protected:

  planning_scene::PlanningSceneConstPtr planning_scene_;

  bool start_chained_;

  void startOn();
  void goalOn();

  boost::shared_ptr<KinematicsGroupVisualization> start_;
  boost::shared_ptr<KinematicsGroupVisualization> goal_;
};

}

#endif
