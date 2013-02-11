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
#include <moveit/move_group/move_group_cartesian_path_service_capability.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_tools.h>
#include <eigen_conversions/eigen_msg.h>

move_group::MoveGroupCartesianPathService::MoveGroupCartesianPathService(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug):
  MoveGroupCapability(psm, debug)
{
  cartesian_path_service_ = root_node_handle_.advertiseService(CARTESIAN_PATH_SERVICE_NAME, &MoveGroupCartesianPathService::computeService, this);
}

namespace 
{
bool isStateValid(const planning_scene::PlanningScene *planning_scene,
                  const kinematic_constraints::KinematicConstraintSet *constraint_set,
                  robot_state::JointStateGroup *group, const std::vector<double> &ik_solution)
{
  group->setVariableValues(ik_solution);
  return (!planning_scene || !planning_scene->isStateColliding(*group->getRobotState(), group->getName())) &&
    (!constraint_set || constraint_set->decide(*group->getRobotState()).satisfied);
}
}

bool move_group::MoveGroupCartesianPathService::computeService(moveit_msgs::GetCartesianPath::Request &req, moveit_msgs::GetCartesianPath::Response &res)
{
  robot_state::RobotState start_state = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState();
  robot_state::robotStateMsgToRobotState(req.start_state, start_state);
  if (robot_state::JointStateGroup *jsg = start_state.getJointStateGroup(req.group_name))
  {
    std::string link_name = req.link_name;
    if (link_name.empty() && !jsg->getJointModelGroup()->getLinkModelNames().empty())
      link_name = jsg->getJointModelGroup()->getLinkModelNames().back();
    
    bool ok = true;
    EigenSTL::vector_Affine3d waypoints(req.waypoints.size());
    const std::string &default_frame = planning_scene_monitor_->getRobotModel()->getModelFrame();
    bool no_transform = req.header.frame_id.empty() || req.header.frame_id == default_frame || req.header.frame_id == link_name;
    
    for (std::size_t i = 0 ; i < req.waypoints.size() ; ++i)
    {
      if (no_transform)
        tf::poseMsgToEigen(req.waypoints[i], waypoints[i]);
      else
      {
        geometry_msgs::PoseStamped p;
        p.header = req.header;
        p.pose = req.waypoints[i];
        if (performTransform(p, default_frame))
          tf::poseMsgToEigen(p.pose, waypoints[i]);
        else
        {
          ok = false;
          break;
        }
      }
    }
    
    if (ok)
    {
      if (req.max_step < std::numeric_limits<double>::epsilon())
      {
        ROS_ERROR("Maximum step to take between consecutive configrations along Cartesian path was not specified (this value needs to be > 0)");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      }
      else
      {
        if (waypoints.size() > 0)
        {
          robot_state::StateValidityCallbackFn constraint_fn;
          boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
          boost::scoped_ptr<kinematic_constraints::KinematicConstraintSet> kset;
          if (req.avoid_collisions || !kinematic_constraints::isEmpty(req.path_constraints))
          {
            ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));
            kset.reset(new kinematic_constraints::KinematicConstraintSet((*ls)->getRobotModel(), (*ls)->getTransforms()));
            kset->add(req.path_constraints); 
            constraint_fn = boost::bind(&isStateValid, req.avoid_collisions ? static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get() : NULL, kset->empty() ? NULL : kset.get(), _1, _2);
          }
          
          std::vector<boost::shared_ptr<robot_state::RobotState> > traj;
          res.fraction = jsg->computeCartesianPath(traj, link_name, waypoints, link_name == req.header.frame_id, req.max_step, req.jump_threshold, constraint_fn);
          robot_state::robotStateToRobotStateMsg(start_state, res.start_state);
          
          robot_trajectory::RobotTrajectory rt(planning_scene_monitor_->getRobotModel(), req.group_name);
          for (std::size_t i = 0 ; i < traj.size() ; ++i)
            rt.addSuffixWayPoint(traj[i], 0.0);
          rt.getRobotTrajectoryMsg(res.solution);
        }
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      }
    }
    else
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
  }
  else
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
  
  return true;
}
