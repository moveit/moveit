/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): E. Gil JOnes

#include "object_manipulator/place_execution/place_tester_fast.h"

#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/vector_tools.h"
#include "object_manipulator/tools/mechanism_interface.h"

using object_manipulation_msgs::PlaceLocationResult;
using arm_navigation_msgs::ArmNavigationErrorCodes;

namespace object_manipulator {

  PlaceTesterFast::PlaceTesterFast(planning_environment::CollisionModels* cm,
				   const std::string& plugin_name) 
  : PlaceTester(), 
    consistent_angle_(M_PI/12.0), 
    num_points_(10), 
    redundancy_(2),
    cm_(cm),
    state_(NULL),
    kinematics_loader_("kinematics_base","kinematics::KinematicsBase")
  {  
    ros::NodeHandle nh;
    
  vis_marker_publisher_ = nh.advertise<visualization_msgs::Marker> ("grasp_executor_fast", 128);
  vis_marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray> ("grasp_executor_fast_array", 128);

  const std::map<std::string, planning_models::KinematicModel::GroupConfig>& group_config_map = 
    getCollisionModels()->getKinematicModel()->getJointModelGroupConfigMap();


  for(std::map<std::string, planning_models::KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin();
      it != group_config_map.end();
      it++) {
    if(!it->second.base_link_.empty() && !it->second.tip_link_.empty()) {
      kinematics::KinematicsBase* kinematics_solver = NULL;
      try
      {
        kinematics_solver = kinematics_loader_.createClassInstance(plugin_name);
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("The plugin failed to load. Error1: %s", ex.what());    //handle the class failing to load
        return;
      }

      ik_solver_map_[it->first] = new arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware(kinematics_solver,
                                                                                                          getCollisionModels(),
                                                                                                          it->first);
      ik_solver_map_[it->first]->setSearchDiscretization(.025);
    }
  }
}

PlaceTesterFast::~PlaceTesterFast()
{
  for(std::map<std::string, arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware*>::iterator it = ik_solver_map_.begin();
      it != ik_solver_map_.end();
      it++) {
    delete it->second;
  }
}

planning_environment::CollisionModels* PlaceTesterFast::getCollisionModels() {
  if(cm_ == NULL) {
    return &mechInterface().getCollisionModels();
  } else {
    return cm_;
  }
}

planning_models::KinematicState* PlaceTesterFast::getPlanningSceneState() {
  if(state_ == NULL) {
    return mechInterface().getPlanningSceneState();
  } else {
    return state_;
  }
}

/*! Zero padding on fingertip links */
std::vector<arm_navigation_msgs::LinkPadding> 
PlaceTesterFast::linkPaddingForPlace(const object_manipulation_msgs::PlaceGoal &place_goal)
{
  std::vector<arm_navigation_msgs::LinkPadding> ret = place_goal.additional_link_padding;
  concat(ret, MechanismInterface::gripperPadding(place_goal.arm_name, 0.0));
  arm_navigation_msgs::LinkPadding att_pad;
  att_pad.link_name = place_goal.collision_object_name;
  att_pad.padding = place_goal.place_padding;
  ret.push_back(att_pad);
  return ret;
}

void PlaceTesterFast::getGroupLinks(const std::string& group_name,
                                      std::vector<std::string>& group_links)
{
  group_links.clear();
  const planning_models::KinematicModel::JointModelGroup* jmg = 
    getCollisionModels()->getKinematicModel()->getModelGroup(group_name);
  if(jmg == NULL) return;
  group_links = jmg->getGroupLinkNames();
}

bool PlaceTesterFast::getInterpolatedIK(const std::string& arm_name,
                                        const tf::Transform& first_pose,
                                        const tf::Vector3& direction,
                                        const double& distance,
                                        const std::vector<double>& ik_solution,
                                        const bool& reverse, 
                                        const bool& premultiply,
                                        trajectory_msgs::JointTrajectory& traj) {

  std::map<std::string, double> ik_solution_map;
  for(unsigned int i = 0; i < traj.joint_names.size(); i++) {
    ik_solution_map[traj.joint_names[i]] = ik_solution[i];
  }

  getPlanningSceneState()->setKinematicState(ik_solution_map);

  geometry_msgs::Pose start_pose;
  tf::poseTFToMsg(first_pose, start_pose);

  arm_navigation_msgs::Constraints emp;
  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  return ik_solver_map_[arm_name]->interpolateIKDirectional(start_pose,
                                                            direction,
                                                            distance,
                                                            emp,
                                                            getPlanningSceneState(),
                                                            error_code,
                                                            traj,
                                                            redundancy_, 
                                                            consistent_angle_,
                                                            reverse,
                                                            premultiply,
                                                            num_points_,
                                                            ros::Duration(2.5),
                                                            false);
}

void PlaceTesterFast::testPlace(const object_manipulation_msgs::PlaceGoal &placre_goal,
                                const geometry_msgs::PoseStamped &place_locations,
                                PlaceExecutionInfo &execution_info)
{
}

void PlaceTesterFast::testPlaces(const object_manipulation_msgs::PlaceGoal &place_goal,
                                 const std::vector<geometry_msgs::PoseStamped> &place_locations,
                                 std::vector<PlaceExecutionInfo> &execution_info,
                                 bool return_on_first_hit) 

{
  ros::WallTime start = ros::WallTime::now();

  std::map<unsigned int, unsigned int> outcome_count;
    
  planning_environment::CollisionModels* cm = getCollisionModels();
  planning_models::KinematicState* state = getPlanningSceneState();

  std::map<std::string, double> planning_scene_state_values;
  state->getKinematicStateValues(planning_scene_state_values);
  
  std::vector<std::string> end_effector_links, arm_links; 
  getGroupLinks(handDescription().gripperCollisionName(place_goal.arm_name), end_effector_links);
  getGroupLinks(handDescription().armGroup(place_goal.arm_name), arm_links);
  
  collision_space::EnvironmentModel::AllowedCollisionMatrix original_acm = cm->getCurrentAllowedCollisionMatrix();
  cm->disableCollisionsForNonUpdatedLinks(place_goal.arm_name);
  collision_space::EnvironmentModel::AllowedCollisionMatrix group_disable_acm = cm->getCurrentAllowedCollisionMatrix();
  collision_space::EnvironmentModel::AllowedCollisionMatrix object_disable_acm = group_disable_acm;
  if(!place_goal.collision_support_surface_name.empty()) {
    if(place_goal.collision_support_surface_name == "\"all\"")
    {
      object_disable_acm.changeEntry(place_goal.collision_object_name, true);
    }
    else
    {
      object_disable_acm.changeEntry(place_goal.collision_object_name, place_goal.collision_support_surface_name, true);
    }
  }
  collision_space::EnvironmentModel::AllowedCollisionMatrix object_support_disable_acm = object_disable_acm;
  if(place_goal.allow_gripper_support_collision) {
    if(place_goal.collision_support_surface_name == "\"all\"")
    {
      for(unsigned int i = 0; i < end_effector_links.size(); i++){
	object_support_disable_acm.changeEntry(end_effector_links[i], true);
      }
    }
    else
    {
      object_support_disable_acm.changeEntry(place_goal.collision_support_surface_name, end_effector_links, true); 
    }
  }
  collision_space::EnvironmentModel::AllowedCollisionMatrix object_all_arm_disable_acm = object_disable_acm;
  collision_space::EnvironmentModel::AllowedCollisionMatrix object_support_all_arm_disable_acm = object_support_disable_acm;
  collision_space::EnvironmentModel::AllowedCollisionMatrix group_all_arm_disable_acm = group_disable_acm;

  //turning off collisions for the arm associated with this end effector
  for(unsigned int i = 0; i < arm_links.size(); i++) {
    object_all_arm_disable_acm.changeEntry(arm_links[i], true);
    object_support_all_arm_disable_acm.changeEntry(arm_links[i], true);
    group_all_arm_disable_acm.changeEntry(arm_links[i], true);
  }
  cm->setAlteredAllowedCollisionMatrix(object_support_all_arm_disable_acm);
  
  //first we apply link padding for place check
  cm->applyLinkPaddingToCollisionSpace(linkPaddingForPlace(place_goal));
    
  execution_info.clear();
  execution_info.resize(place_locations.size());

  tf::Vector3 approach_dir;
  tf::vector3MsgToTF(doNegate(place_goal.approach.direction.vector), approach_dir);
  approach_dir.normalize();
  tf::Vector3 distance_approach_dir = approach_dir*fabs(place_goal.approach.desired_distance);
  tf::Transform approach_trans(tf::Quaternion(0,0,0,1.0), distance_approach_dir);

  tf::Vector3 retreat_dir;
  tf::vector3MsgToTF(doNegate(handDescription().approachDirection(place_goal.arm_name)), retreat_dir);
  retreat_dir.normalize();
  tf::Vector3 distance_retreat_dir = retreat_dir*fabs(place_goal.desired_retreat_distance);    
  tf::Transform retreat_trans(tf::Quaternion(0,0,0,1.0), distance_retreat_dir);

  std::map<std::string, double> planning_scene_state_values_post_grasp = planning_scene_state_values_post_grasp;
  std::map<std::string, double> post_grasp_joint_vals;    
  std::map<std::string, double> grasp_joint_vals;
  for(unsigned int j = 0; j < place_goal.grasp.pre_grasp_posture.name.size(); j++) {
    planning_scene_state_values_post_grasp[place_goal.grasp.pre_grasp_posture.name[j]] = place_goal.grasp.pre_grasp_posture.position[j];
    post_grasp_joint_vals[place_goal.grasp.pre_grasp_posture.name[j]] = place_goal.grasp.pre_grasp_posture.position[j];
    grasp_joint_vals[place_goal.grasp.pre_grasp_posture.name[j]] = planning_scene_state_values[place_goal.grasp.pre_grasp_posture.name[j]];
  }

  std::vector<tf::Transform> place_poses(place_locations.size());

  //now this is place specific
  for(unsigned int i = 0; i < place_locations.size(); i++) {
    //using the grasp posture
    state->setKinematicState(post_grasp_joint_vals);
    
    //always true
    execution_info[i].result_.continuation_possible = true;
    
    if(!cm->convertPoseGivenWorldTransform(*state,
                                           cm->getWorldFrameId(),
                                           place_locations[i].header,
                                           place_locations[i].pose,
                                           execution_info[i].gripper_place_pose_)) {
      ROS_INFO_STREAM("Something wrong with pose conversion");
      continue;
    }
    tf::poseMsgToTF(execution_info[i].gripper_place_pose_.pose, place_poses[i]);
    tf::Transform grasp_trans;
    tf::poseMsgToTF(place_goal.grasp.grasp_pose, grasp_trans);
    //post multiply for object frame
    place_poses[i] = place_poses[i]*grasp_trans;
    tf::poseTFToMsg(place_poses[i], execution_info[i].gripper_place_pose_.pose);
    state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(place_goal.arm_name),place_poses[i]);
    
    if(cm->isKinematicStateInCollision(*state)) {
      ROS_DEBUG_STREAM("Place in collision");
      execution_info[i].result_.result_code = PlaceLocationResult::PLACE_IN_COLLISION;
      outcome_count[PlaceLocationResult::PLACE_IN_COLLISION]++;
    } else {
      execution_info[i].result_.result_code = 0;
    }
  }

  //now we revert link paddings for approach and retreat checks
  cm->revertCollisionSpacePaddingToDefault();

  //now we do the place approach pose, not allowing anything different 
  cm->setAlteredAllowedCollisionMatrix(group_all_arm_disable_acm);

  for(unsigned int i = 0; i < place_locations.size(); i++) {
  
    if(execution_info[i].result_.result_code != 0) continue;

    state->setKinematicState(planning_scene_state_values);
    
    tf::Transform approach_pose = approach_trans*place_poses[i];
    state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(place_goal.arm_name),approach_pose);
    
    if(cm->isKinematicStateInCollision(*state)) {
      ROS_DEBUG_STREAM("Preplace in collision");
      // std::vector<arm_navigation_msgs::ContactInformation> contacts;
      // cm->getAllCollisionsForState(*state, contacts,1);
      // if(contacts.size() == 0) {
      //   ROS_WARN_STREAM("Collision reported but no contacts");
      // }
      // for(unsigned int j = 0; j < contacts.size(); j++) {
      //   ROS_INFO_STREAM("Collision between " << contacts[j].contact_body_1 
      //                   << " and " << contacts[j].contact_body_2);
      // }
      execution_info[i].result_.result_code = PlaceLocationResult::PREPLACE_IN_COLLISION;
      outcome_count[PlaceLocationResult::PREPLACE_IN_COLLISION]++;
      continue;
    }
  }

  //TODO - for now, just have to hope that we're not in contact with the object
  //after we release, but there's not a good way to check this for now
  //so we just leave the object all arm disable on for the retreat position check
  cm->setAlteredAllowedCollisionMatrix(object_all_arm_disable_acm);

  //first we do retreat, with the gripper at post grasp position
  for(unsigned int i = 0; i < place_locations.size(); i++) {
  
    if(execution_info[i].result_.result_code != 0) continue;

    state->setKinematicState(post_grasp_joint_vals);

    tf::Transform retreat_pose = place_poses[i]*retreat_trans;
    state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(place_goal.arm_name),retreat_pose);
    
    if(cm->isKinematicStateInCollision(*state)) {
	std_msgs::ColorRGBA col_pregrasp;
	col_pregrasp.r = 0.0;
	col_pregrasp.g = 1.0;
	col_pregrasp.b = 1.0;
	col_pregrasp.a = 1.0;
	visualization_msgs::MarkerArray arr;
	cm_->getRobotMarkersGivenState(*state, arr, col_pregrasp,
				       "retreat",
				       ros::Duration(0.0), 
				       &end_effector_links);
	vis_marker_array_publisher_.publish(arr);
      ROS_DEBUG_STREAM("Retreat in collision");
      execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_IN_COLLISION;
      outcome_count[PlaceLocationResult::RETREAT_IN_COLLISION]++;
    }
  }

  std_msgs::Header world_header;
  world_header.frame_id = cm->getWorldFrameId();
  const std::vector<std::string>& joint_names = ik_solver_map_[place_goal.arm_name]->getJointNames();

  if(return_on_first_hit) {
    
    bool last_ik_failed = false;
    for(unsigned int i = 0; i < place_locations.size(); i++) {

      if(execution_info[i].result_.result_code != 0) continue;

      if(!last_ik_failed) {
        //now we move to the ik portion, which requires re-enabling collisions for the arms
        cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);
        
        //and also reducing link paddings
        cm->applyLinkPaddingToCollisionSpace(linkPaddingForPlace(place_goal));
      }
      //getting back to original state for seed
      state->setKinematicState(planning_scene_state_values_post_grasp);

      //now call ik for grasp
      geometry_msgs::Pose place_geom_pose;
      tf::poseTFToMsg(place_poses[i], place_geom_pose);
      geometry_msgs::PoseStamped base_link_place_pose;
      cm->convertPoseGivenWorldTransform(*state,
					 ik_solver_map_[place_goal.arm_name]->getBaseName(),
                                         world_header,
                                         place_geom_pose,
                                         base_link_place_pose);
      
      arm_navigation_msgs::Constraints emp;
      sensor_msgs::JointState solution;
      arm_navigation_msgs::ArmNavigationErrorCodes error_code;
      if(!ik_solver_map_[place_goal.arm_name]->findConstraintAwareSolution(base_link_place_pose.pose,
                                                                           emp,
                                                                           state,
                                                                           solution,
                                                                           error_code,
                                                                           false)) {
        ROS_DEBUG_STREAM("Place out of reach");
        execution_info[i].result_.result_code = PlaceLocationResult::PLACE_OUT_OF_REACH;
        outcome_count[PlaceLocationResult::PLACE_OUT_OF_REACH]++;
        last_ik_failed = true;
        continue;
      } else {
        last_ik_failed = false;
      }

      state->setKinematicState(grasp_joint_vals);
      
      //now we solve interpolated ik
      tf::Transform base_link_bullet_place_pose;
      tf::poseMsgToTF(base_link_place_pose.pose, base_link_bullet_place_pose);
      //now we need to do interpolated ik
      execution_info[i].descend_trajectory_.joint_names = joint_names;
      if(!getInterpolatedIK(place_goal.arm_name,
                            base_link_bullet_place_pose,
                            approach_dir,
                            place_goal.approach.desired_distance,
                            solution.position,
                            true,
                            true,
                            execution_info[i].descend_trajectory_)) {
        ROS_DEBUG_STREAM("No interpolated IK for approach to place");
        execution_info[i].result_.result_code = PlaceLocationResult::PLACE_UNFEASIBLE;
        outcome_count[PlaceLocationResult::PLACE_UNFEASIBLE]++;
        continue;
      }
      
      state->setKinematicState(planning_scene_state_values_post_grasp);
      execution_info[i].retreat_trajectory_.joint_names = joint_names;
      if(!getInterpolatedIK(place_goal.arm_name,
                            base_link_bullet_place_pose,
                            retreat_dir,
                            place_goal.desired_retreat_distance,
                            solution.position,
                            false,
                            false,
                            execution_info[i].retreat_trajectory_)) {
        ROS_DEBUG_STREAM("No interpolated IK for place to retreat");
        execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_UNFEASIBLE;
        outcome_count[PlaceLocationResult::RETREAT_UNFEASIBLE]++;
        continue;
      }

      //now we revert link paddings and object collisions and do a final check for the initial ik points
      cm->revertCollisionSpacePaddingToDefault();

      //the start of the place approach needs to be collision-free according to the default collision matrix
      cm->setAlteredAllowedCollisionMatrix(group_disable_acm);

      if(execution_info[i].descend_trajectory_.points.empty()) {
        ROS_WARN_STREAM("No result code and no points in approach trajectory");
        continue;
      }

      std::map<std::string, double> pre_place_ik = grasp_joint_vals;
      for(unsigned int j = 0; j < joint_names.size(); j++) {
        pre_place_ik[joint_names[j]] = execution_info[i].descend_trajectory_.points[0].positions[j];
      } 
      state->setKinematicState(pre_place_ik);
      if(cm->isKinematicStateInCollision(*state)) {
        ROS_DEBUG_STREAM("Final pre-place check failed");
        execution_info[i].result_.result_code = PlaceLocationResult::PREPLACE_OUT_OF_REACH;
        outcome_count[PlaceLocationResult::PREPLACE_OUT_OF_REACH]++;
        continue;
      }

      //the end of the place retreat also needs to be collision-free according to the default collision matrix
      if(execution_info[i].retreat_trajectory_.points.empty()) {
        ROS_WARN_STREAM("No result code and no points in retreat trajectory");
        continue;
      }    
      std::map<std::string, double> retreat_ik = post_grasp_joint_vals;
      for(unsigned int j = 0; j < joint_names.size(); j++) {
       retreat_ik[joint_names[j]] = execution_info[i].retreat_trajectory_.points.back().positions[j];
      } 
      state->setKinematicState(retreat_ik);
      if(cm->isKinematicStateInCollision(*state)) {
        ROS_DEBUG_STREAM("Final retreat check failed");
        execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_OUT_OF_REACH;
        outcome_count[PlaceLocationResult::RETREAT_OUT_OF_REACH]++;
        continue;
      } else {
        ROS_INFO_STREAM("Everything successful");
        execution_info[i].result_.result_code = PlaceLocationResult::SUCCESS;
	execution_info.resize(i+1);
        outcome_count[PlaceLocationResult::SUCCESS]++;
        break;
      }
    }
    for(std::map<unsigned int, unsigned int>::iterator it = outcome_count.begin();
        it != outcome_count.end();
        it++) {
      ROS_INFO_STREAM("Outcome " << it->first << " count " << it->second);
    }
    cm->setAlteredAllowedCollisionMatrix(original_acm);
    return;
  }
    
  //now we move to the ik portion, which requires re-enabling collisions for the arms
  cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);
  
  //and also reducing link paddings
  cm->applyLinkPaddingToCollisionSpace(linkPaddingForPlace(place_goal));
  
  for(unsigned int i = 0; i < place_locations.size(); i++) {

    if(execution_info[i].result_.result_code != 0) continue;

    //getting back to original state for seed
    state->setKinematicState(planning_scene_state_values_post_grasp);

    //now call ik for grasp
    geometry_msgs::Pose place_geom_pose;
    tf::poseTFToMsg(place_poses[i], place_geom_pose);
    geometry_msgs::PoseStamped base_link_place_pose;
    cm->convertPoseGivenWorldTransform(*state,
				       ik_solver_map_[place_goal.arm_name]->getBaseName(),
                                       world_header,
                                       place_geom_pose,
                                       base_link_place_pose);
    
    arm_navigation_msgs::Constraints emp;
    sensor_msgs::JointState solution;
    arm_navigation_msgs::ArmNavigationErrorCodes error_code;
    if(!ik_solver_map_[place_goal.arm_name]->findConstraintAwareSolution(base_link_place_pose.pose,
                                                                         emp,
                                                                         state,
                                                                         solution,
                                                                         error_code,
                                                                         false)) {
      ROS_DEBUG_STREAM("Place out of reach");
      execution_info[i].result_.result_code = PlaceLocationResult::PLACE_OUT_OF_REACH;
      outcome_count[PlaceLocationResult::PLACE_OUT_OF_REACH]++;
      continue;
    } 

    state->setKinematicState(grasp_joint_vals);

    //now we solve interpolated ik
    tf::Transform base_link_bullet_place_pose;
    tf::poseMsgToTF(base_link_place_pose.pose, base_link_bullet_place_pose);
    //now we need to do interpolated ik
    execution_info[i].descend_trajectory_.joint_names = joint_names;
    if(!getInterpolatedIK(place_goal.arm_name,
                          base_link_bullet_place_pose,
                          approach_dir,
                          place_goal.approach.desired_distance,
                          solution.position,
                          true,
                          true,
                          execution_info[i].descend_trajectory_)) {
      ROS_DEBUG_STREAM("No interpolated IK for approach to place");
      execution_info[i].result_.result_code = PlaceLocationResult::PLACE_UNFEASIBLE;
      outcome_count[PlaceLocationResult::PLACE_UNFEASIBLE]++;
      continue;
    }

    state->setKinematicState(planning_scene_state_values_post_grasp);
    execution_info[i].retreat_trajectory_.joint_names = joint_names;
    if(!getInterpolatedIK(place_goal.arm_name,
                          base_link_bullet_place_pose,
                          retreat_dir,
                          place_goal.desired_retreat_distance,
                          solution.position,
                          false,
                          false,
                          execution_info[i].retreat_trajectory_)) {
      ROS_DEBUG_STREAM("No interpolated IK for place to retreat");
      execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_UNFEASIBLE;
      outcome_count[PlaceLocationResult::RETREAT_UNFEASIBLE]++;
      continue;
    }
  }

  //now we revert link paddings and object collisions and do a final check for the initial ik points
  cm->revertCollisionSpacePaddingToDefault();

  cm->setAlteredAllowedCollisionMatrix(group_disable_acm);
  
  for(unsigned int i = 0; i < place_locations.size(); i++) {
    
    if(execution_info[i].result_.result_code != 0) continue;
    
    if(execution_info[i].descend_trajectory_.points.empty()) {
      ROS_WARN_STREAM("No result code and no points in approach trajectory");
      continue;
    }

    std::map<std::string, double> pre_place_ik = grasp_joint_vals;
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      pre_place_ik[joint_names[j]] = execution_info[i].descend_trajectory_.points[0].positions[j];
    } 
    state->setKinematicState(pre_place_ik);
    if(cm->isKinematicStateInCollision(*state)) {
      ROS_DEBUG_STREAM("Final pre-place check failed");
      execution_info[i].result_.result_code = PlaceLocationResult::PREPLACE_OUT_OF_REACH;
      outcome_count[PlaceLocationResult::PREPLACE_OUT_OF_REACH]++;
      continue;
    }
    
  }

  //now we need to disable collisions with the object for lift
  cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);

  for(unsigned int i = 0; i < place_locations.size(); i++) {
    
    if(execution_info[i].result_.result_code != 0) continue;
    
    if(execution_info[i].retreat_trajectory_.points.empty()) {
      ROS_WARN_STREAM("No result code and no points in retreat trajectory");
      continue;
    }    
    std::map<std::string, double> retreat_ik = post_grasp_joint_vals;
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      retreat_ik[joint_names[j]] = execution_info[i].retreat_trajectory_.points.back().positions[j];
    } 
    state->setKinematicState(retreat_ik);
    if(cm->isKinematicStateInCollision(*state)) {
      ROS_DEBUG_STREAM("Final lift check failed");
      execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_OUT_OF_REACH;
      outcome_count[PlaceLocationResult::RETREAT_OUT_OF_REACH]++;
      continue;
    } else {
      ROS_DEBUG_STREAM("Everything successful");
      execution_info[i].result_.result_code = PlaceLocationResult::SUCCESS;
      outcome_count[PlaceLocationResult::SUCCESS]++;
    }
  }
  cm->setAlteredAllowedCollisionMatrix(original_acm);

  ROS_INFO_STREAM("Took " << (ros::WallTime::now()-start).toSec());

  for(std::map<unsigned int, unsigned int>::iterator it = outcome_count.begin();
      it != outcome_count.end();
      it++) {
    ROS_INFO_STREAM("Outcome " << it->first << " count " << it->second);
  }
}


} //namespace object_manipulator
