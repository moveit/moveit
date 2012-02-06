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

namespace moveit_visualization_ros
{

class KinematicsStartGoalVisualization {
public:
  
  KinematicsStartGoalVisualization(planning_scene::PlanningSceneConstPtr planning_scene,
                                   boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                   const std::string& group_name, 
                                   const std::string& kinematics_solver_name,
                                   ros::Publisher& marker_publisher,
                                   bool show = true); 

  ~KinematicsStartGoalVisualization() {
  }

  const planning_models::KinematicState& getStartState() const {
    return start_->getState();
  }

  const planning_models::KinematicState& getGoalState() const {
    return goal_->getState();
  }

  void updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);

  void addMenuEntry(const std::string& name, 
                    const boost::function<void(const std::string&)>& callback);

  void setRandomStartGoal();
  void resetStartGoal();

  void hideAllMarkers();
  void showAllMarkers();

protected:

  void startOn();
  void goalOn();

  boost::shared_ptr<KinematicsGroupVisualization> start_;
  boost::shared_ptr<KinematicsGroupVisualization> goal_;
};

}

#endif
