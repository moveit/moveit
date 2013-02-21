/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

// MoveIt!
#include <kinematics_planner_ros/kinematics_solver_ros.h>
#include <kinematics_base/kinematics_base.h>

namespace kinematics_planner_ros
{

const static std::string IK_WITH_COLLISION_SERVICE = "get_ik";

bool KinematicsSolverROS::initialize()
{
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  planning_scene_monitor_->startStateMonitor();
   
  std::map<std::string,kinematics::KinematicsBasePtr> kinematics_solver_map = planning_scene_monitor_->getRDFLoader()->generateKinematicsSolversMap();
  
  for(std::map<std::string,kinematics::KinematicsBasePtr>::iterator it = kinematics_solver_map.begin(); it != kinematics_solver_map.end(); ++it)
  {
    std::string group_name = it->first;
    std::string tip_name = it->second->getTipFrame();

    ROS_INFO("Group: %s, %s",group_name.c_str(),tip_name.c_str());

    kinematics_solver_[tip_name].reset(new kinematics_planner::KinematicsSolver());
    if(!kinematics_solver_[tip_name]->initialize(planning_scene_monitor_->getRobotModel(),
                                                 kinematics_solver_map,
                                                 group_name))
    {
      ROS_WARN("Will not solve IK for group %s",group_name.c_str());
      kinematics_solver_[tip_name].reset();      
    }    
    else
    {
      kinematics_solver_group_name_map_[group_name] = kinematics_solver_[tip_name];
    }    
  }
  get_ik_service_ = node_handle_.advertiseService(IK_WITH_COLLISION_SERVICE,&KinematicsSolverROS::getIK,this);
  return true;  
}

bool KinematicsSolverROS::getIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request,
                                kinematics_msgs::GetConstraintAwarePositionIK::Response &response)
{
  if(kinematics_solver_.find(request.ik_request.ik_link_name) == kinematics_solver_.end())
  {
    ROS_ERROR("Could not find kinematics solver for requested link: %s",request.ik_request.ik_link_name.c_str());    
    response.error_code.val = response.error_code.PLANNING_FAILED;
    return true;    
  }

  planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor_->getPlanningScene();
  kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(planning_scene_monitor_->getRobotModel(),planning_scene->getTransforms());
  planning_scene->setCurrentState(request.ik_request.robot_state);
  
  kinematics_solver_.find(request.ik_request.ik_link_name)->second->solve(request.ik_request.pose_stamped,
                                                                          planning_scene,
                                                                          request.timeout.toSec(),
                                                                          response.solution,
                                                                          response.error_code,kinematic_constraint_set);
  return true;  
}
  

}

