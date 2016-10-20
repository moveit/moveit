/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Willow Garage, Inc.
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

/* Author: Chittaranjan Srinivas Swaminathan */

#ifndef CHOMP_INTERFACE_CHOMP_PLANNING_CONTEXT_H
#define CHOMP_INTERFACE_CHOMP_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include <chomp_interface/chomp_interface.h>
#include <chomp_interface/chomp_planning_context.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_listener.h>

#include <moveit/robot_state/conversions.h>

namespace chomp_interface
{
MOVEIT_CLASS_FORWARD(CHOMPPlanningContext);

class CHOMPPlanningContext : public planning_interface::PlanningContext
{
public:
  virtual bool solve(planning_interface::MotionPlanResponse &res);
  virtual bool solve(planning_interface::MotionPlanDetailedResponse &res);

  virtual void clear();
  virtual bool terminate();

  CHOMPPlanningContext(const std::string &name, const std::string &group, const robot_model::RobotModelConstPtr &model);

  virtual ~CHOMPPlanningContext();

  void initialize();

private:
  CHOMPInterfacePtr chomp_interface_;
  moveit::core::RobotModelConstPtr robot_model_;

  boost::shared_ptr<tf::TransformListener> tf_;
};

} /* namespace chomp_interface */

#endif /* CHOMP_PLANNING_CONTEXT_H_ */
