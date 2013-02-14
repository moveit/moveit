/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/move_group/names.h>
#include <moveit/move_group/move_group_kinematics_service_capability.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>


move_group::MoveGroupKinematicsService::MoveGroupKinematicsService(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug):
  MoveGroupCapability(psm, debug)
{
  fk_service_ = root_node_handle_.advertiseService(FK_SERVICE_NAME, &MoveGroupKinematicsService::computeFKService, this);
  ik_service_ = root_node_handle_.advertiseService(IK_SERVICE_NAME, &MoveGroupKinematicsService::computeIKService, this);
}

namespace 
{
bool isIKSolutionValid(const planning_scene::PlanningScene *planning_scene,
                       const kinematic_constraints::KinematicConstraintSet *constraint_set,
                       robot_state::JointStateGroup *group, const std::vector<double> &ik_solution)
{
  group->setVariableValues(ik_solution);
  return (!planning_scene || !planning_scene->isStateColliding(*group->getRobotState(), group->getName())) &&
    (!constraint_set || constraint_set->decide(*group->getRobotState()).satisfied);
}
}

void move_group::MoveGroupKinematicsService::computeIK(moveit_msgs::PositionIKRequest &req, 
                                                       moveit_msgs::RobotState &solution,
                                                       moveit_msgs::MoveItErrorCodes &error_code,
                                                       const robot_state::StateValidityCallbackFn &constraint) const
{
  robot_state::RobotState rs = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState();
  robot_state::JointStateGroup *jsg = rs.getJointStateGroup(req.group_name);
  if (jsg)
  {  
    robot_state::robotStateMsgToRobotState(req.robot_state, rs);  
    const std::string &default_frame = planning_scene_monitor_->getRobotModel()->getModelFrame();
    
    if (req.pose_stamped_vector.empty() || req.pose_stamped_vector.size() == 1)
    {
      geometry_msgs::PoseStamped req_pose = req.pose_stamped_vector.empty() ? req.pose_stamped : req.pose_stamped_vector[0];
      std::string ik_link = req.pose_stamped_vector.empty() ? (req.ik_link_names.empty() ? "" : req.ik_link_names[0]) : req.ik_link_name;

      if (performTransform(req_pose, default_frame))
      {
        bool result_ik = false;        
        if (ik_link.empty())
          result_ik = jsg->setFromIK(req_pose.pose, req.attempts, req.timeout.toSec(), constraint);      
        else
          result_ik = jsg->setFromIK(req_pose.pose, ik_link, req.attempts, req.timeout.toSec(), constraint);
      
        if(result_ik)
        {
          robot_state::robotStateToRobotStateMsg(rs, solution, false);
          error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;        
        }
        else
          error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      }
      else
        error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
    }
    else
    {
      if (req.pose_stamped_vector.size() != req.ik_link_names.size())
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
      else
      {
        bool ok = true;
        EigenSTL::vector_Affine3d req_poses(req.pose_stamped_vector.size());
        for (std::size_t k = 0 ; k < req.pose_stamped_vector.size() ; ++k)
        {
          geometry_msgs::PoseStamped msg = req.pose_stamped_vector[k];
          if (performTransform(msg, default_frame))
            tf::poseMsgToEigen(msg.pose, req_poses[k]);
          else
          {   
            error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
            ok = false;
            break;
          }
        }
        if (ok)
        {
          if (jsg->setFromIK(req_poses, req.ik_link_names, req.attempts, req.timeout.toSec(), constraint))
          {
            robot_state::robotStateToRobotStateMsg(rs, solution, false);
            error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;        
          }
          else
            error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        }
      }
    }
  }
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
}

bool move_group::MoveGroupKinematicsService::computeIKService(moveit_msgs::GetPositionIK::Request &req, moveit_msgs::GetPositionIK::Response &res)
{
  // check if the planning scene needs to be kept locked; if so, call computeIK() in the scope of the lock
  if (req.ik_request.avoid_collisions || !kinematic_constraints::isEmpty(req.ik_request.constraints))
  {
    planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor_);
    kinematic_constraints::KinematicConstraintSet kset(ls->getRobotModel(), ls->getTransforms());
    kset.add(req.ik_request.constraints);
    computeIK(req.ik_request, res.solution, res.error_code, boost::bind(&isIKSolutionValid, req.ik_request.avoid_collisions ?
                                                                        static_cast<const planning_scene::PlanningSceneConstPtr&>(ls).get() : NULL, kset.empty() ? NULL : &kset, _1, _2));
  }
  else
    // compute unconstrained IK, no lock to planning scene maintained
    computeIK(req.ik_request, res.solution, res.error_code);
  return true;
}

bool move_group::MoveGroupKinematicsService::computeFKService(moveit_msgs::GetPositionFK::Request &req, moveit_msgs::GetPositionFK::Response &res)
{
  if (req.fk_link_names.empty())
  {
    ROS_ERROR("No links specified for FK request");   
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
    return true;
  }
  
  const std::string &default_frame = planning_scene_monitor_->getRobotModel()->getModelFrame();
  bool do_transform = !req.header.frame_id.empty() && req.header.frame_id != default_frame && planning_scene_monitor_->getTFClient();
  bool tf_problem = false;
  
  robot_state::RobotState rs = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState();
  robot_state::robotStateMsgToRobotState(req.robot_state, rs);
  for (std::size_t i = 0 ; i < req.fk_link_names.size() ; ++i)
    if (const robot_state::LinkState *ls = rs.getLinkState(req.fk_link_names[i]))
    {
      res.pose_stamped.resize(res.pose_stamped.size() + 1);
      tf::poseEigenToMsg(ls->getGlobalLinkTransform(), res.pose_stamped.back().pose);
      res.pose_stamped.back().header.frame_id = default_frame;
      res.pose_stamped.back().header.stamp = ros::Time::now();
      if (do_transform)
        if (!performTransform(res.pose_stamped.back(), req.header.frame_id))
          tf_problem = true;
      res.fk_link_names.push_back(req.fk_link_names[i]);
    }
  if (tf_problem)
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
  else
    if (res.fk_link_names.size() == req.fk_link_names.size())
      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    else
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
  return true;
}
