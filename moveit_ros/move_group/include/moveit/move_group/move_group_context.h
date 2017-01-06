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

#ifndef MOVEIT_MOVE_GROUP_CONTEXT_
#define MOVEIT_MOVE_GROUP_CONTEXT_

#include <moveit/macros/class_forward.h>

namespace planning_scene_monitor
{
MOVEIT_CLASS_FORWARD(PlanningSceneMonitor);
}

namespace planning_pipeline
{
MOVEIT_CLASS_FORWARD(PlanningPipeline);
}

namespace plan_execution
{
MOVEIT_CLASS_FORWARD(PlanExecution);
MOVEIT_CLASS_FORWARD(PlanWithSensing);
}

namespace trajectory_execution_manager
{
MOVEIT_CLASS_FORWARD(TrajectoryExecutionManager);
}

namespace move_group
{
MOVEIT_CLASS_FORWARD(MoveGroupContext);

struct MoveGroupContext
{
  MoveGroupContext(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                   bool allow_trajectory_execution = false, bool debug = false);
  ~MoveGroupContext();

  bool status() const;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  plan_execution::PlanExecutionPtr plan_execution_;
  plan_execution::PlanWithSensingPtr plan_with_sensing_;
  bool allow_trajectory_execution_;
  bool debug_;
};
}

#endif
