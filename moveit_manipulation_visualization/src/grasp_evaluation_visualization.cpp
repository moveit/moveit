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

#include <moveit_manipulation_visualization/grasp_evaluation_visualization.h>

namespace moveit_manipulation_visualization {

GraspEvaluationVisualization::
GraspEvaluationVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                             boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                             boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader>& kinematics_plugin_loader,
                             ros::Publisher& marker_publisher) :
  planning_scene_(planning_scene),
  marker_publisher_(marker_publisher)
{
  const boost::shared_ptr<const srdf::Model>& srdf = planning_scene->getSrdfModel();
  const std::vector<srdf::Model::Group>& srdf_groups = srdf->getGroups();

  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_plugin_loader->getLoaderFunction();
  
  std::map<std::string, kinematics::KinematicsBasePtr> solver_map;
  for(unsigned int i = 0; i < srdf_groups.size(); i++) {
    if(srdf_groups[i].subgroups_.size() == 0 &&
       srdf_groups[i].chains_.size() == 1) {
      if(!kinematics_plugin_loader->isGroupKnown(srdf_groups[i].name_)) {
        ROS_WARN_STREAM("Really should have loader for " << srdf_groups[i].name_);
        continue;
      }
      const planning_models::KinematicModel::JointModelGroup* jmg
        = planning_scene_->getKinematicModel()->getJointModelGroup(srdf_groups[i].name_); 

      solver_map[srdf_groups[i].name_] = kinematics_allocator(jmg);
    }
  }

  grasp_evaluator_fast_.reset(new grasp_place_evaluation::GraspEvaluatorFast(planning_scene_->getKinematicModel(),
                                                                             solver_map));
}

void GraspEvaluationVisualization::evaluateGrasps(const std::string& group_name,
                                                  const moveit_manipulation_msgs::PickupGoal& goal,
                                                  const planning_models::KinematicState* seed_state,
                                                  const std::vector<moveit_manipulation_msgs::Grasp>& grasps)
{
  grasp_evaluator_fast_->testGrasps(planning_scene_,
                                    seed_state,
                                    goal, 
                                    grasps,
                                    last_grasp_evaluation_info_,
                                    true);
  showGraspPose(0);
}

void GraspEvaluationVisualization::showGraspPose(unsigned int num) {
  if(num >= last_grasp_evaluation_info_.size()) {
    return;
  }

  std::vector<std::string> end_effector_links;
  grasp_evaluator_fast_->getGroupLinks(grasp_evaluator_fast_->getEndEffectorName(planning_scene_->getSrdfModel(),
                                                                                 last_grasp_evaluation_info_[num].group_name_),
                                       end_effector_links);
  
  planning_models::KinematicState state(planning_scene_->getCurrentState());

  state.updateStateWithLinkAt(grasp_evaluator_fast_->getTipLink(last_grasp_evaluation_info_[num].group_name_),
                              last_grasp_evaluation_info_[num].grasp_pose_);

  std_msgs::ColorRGBA col;
  col.g = col.a = 1.0;

  visualization_msgs::MarkerArray arr;

  state.getRobotMarkers(col,
                        "grasp",
                        ros::Duration(0.0),
                        arr,
                        end_effector_links);
  marker_publisher_.publish(arr);
}

}
