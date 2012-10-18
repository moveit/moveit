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
#include <planning_models_loader/kinematic_model_loader.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


namespace moveit_visualization_ros
{

class KinematicsStartGoalVisualization {
public:
  
  KinematicsStartGoalVisualization(planning_scene::PlanningSceneConstPtr planning_scene,
                                   boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                   boost::shared_ptr<planning_models_loader::KinematicModelLoader>& kinematic_model_loader,
                                   const std::string& group_name, 
                                   ros::Publisher& marker_publisher,
                                   boost::shared_ptr<tf::TransformBroadcaster>& broadcaster,
                                   bool show = true); 

  ~KinematicsStartGoalVisualization() {
  }

  /** brief Returns the kinematic state of the start group */
  const planning_models::KinematicState& getStartState() const {
    if(start_chained_) {
      return planning_scene_->getCurrentState();
    } else {
      return start_->getState();
    }
  }

  /** brief Returns the kinematic state of the goal group */
  const planning_models::KinematicState& getGoalState() const {
    return goal_->getState();
  }

  /** brief Sets the kinematic state of the start group */
  void setStartState(const planning_models::KinematicState& state);

  /** brief Sets the kinematic state of the goal group */
  void setGoalState(const planning_models::KinematicState& state);

  /** brief Resets the kinematic state of the start and goal groups */
  void resetStartGoal();

  /** brief Sets a random kinematic state for the start and goal groups */
  void setRandomStartGoal();

  void updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);

  /** Resets start state to current state in the planning scene,
      does nothing if start_chained_ is true. */
  void resetStartState();

  void addMenuEntry(const std::string& name, 
                    const boost::function<void(const std::string&)>& callback);



  void setGoodBadMode(bool use_good_bad) {
    start_->setGoodBadMode(use_good_bad);
    goal_->setGoodBadMode(use_good_bad);
  }

  void hideRegularMarkers() {
    //if(!start_chained_) {
    //  start_->hideRegularMarkers();
    //}
    start_->hideRegularMarkers();
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

  void setStartInteractionEnabled(bool enabled)
  {
      start_->setInteractionEnabled(enabled);
  }


  void setGoalInteractionEnabled(bool enabled)
  {
      goal_->setInteractionEnabled(enabled);
  }

  void setStartVisible(bool visible)
  {
      start_->setVisible(visible);
  }


  void setGoalVisible(bool visible)
  {
      goal_->setVisible(visible);
  }

  void hideGoalRegularMarkers();
  void showGoalRegularMarkers();


  /** brief Activate the controls for the start chain. */
  void startOn();

  /** brief Activate the controls for the end chain. */
  void goalOn();

  void addStateChangedCallback(const boost::function<void(const std::string&,
                                                          const planning_models::KinematicState&)>& callback)
  {
    start_->addStateChangedCallback(callback);
    goal_->addStateChangedCallback(callback);
  }

//  const KinematicsGroupVisualization* getStartGroupVisualization() const { return start_.get(); }
//  const KinematicsGroupVisualization* getGoalGroupVisualization() const  { return goal_.get(); }

  geometry_msgs::PoseStamped getStartInteractiveMarkerPose() const { return start_->getInteractiveMarkerPose(); }
  geometry_msgs::PoseStamped getGoalInteractiveMarkerPose() const { return goal_->getInteractiveMarkerPose(); }

//  void publishEndEffectorTfFrames()
//  {
//    start_->publishEndEffectorTfFrame();
//    goal_->publishEndEffectorTfFrame();
//  }

protected:

  planning_scene::PlanningSceneConstPtr planning_scene_;

  bool start_chained_;

  boost::shared_ptr<KinematicsGroupVisualization> start_;
  boost::shared_ptr<KinematicsGroupVisualization> goal_;
};

}

#endif
