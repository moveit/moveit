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
 *   * Neither the name of Willow Garage nor the names of its
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

#include "query_planners_service_capability.h"
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/move_group/capability_names.h>

move_group::MoveGroupQueryPlannersService::MoveGroupQueryPlannersService() : MoveGroupCapability("QueryPlannersService")
{
}

void move_group::MoveGroupQueryPlannersService::initialize()
{
  query_service_ = root_node_handle_.advertiseService(QUERY_PLANNERS_SERVICE_NAME,
                                                      &MoveGroupQueryPlannersService::queryInterface, this);
}

bool move_group::MoveGroupQueryPlannersService::queryInterface(moveit_msgs::QueryPlannerInterfaces::Request& req,
                                                               moveit_msgs::QueryPlannerInterfaces::Response& res)
{
  const planning_interface::PlannerManagerPtr& planner_interface = context_->planning_pipeline_->getPlannerManager();
  if (planner_interface)
  {
    std::vector<std::string> algs;
    planner_interface->getPlanningAlgorithms(algs);
    moveit_msgs::PlannerInterfaceDescription pi_desc;
    pi_desc.name = planner_interface->getDescription();
    planner_interface->getPlanningAlgorithms(pi_desc.planner_ids);
    res.planner_interfaces.push_back(pi_desc);
  }
  return true;
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupQueryPlannersService, move_group::MoveGroupCapability)
