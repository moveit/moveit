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
*********************************************************************/

/* Author: Ioan Sucan, Brian Gerkey */

#ifndef MOVEIT_PLANNING_INTERFACE_PLANNING_INTERFACE_
#define MOVEIT_PLANNING_INTERFACE_PLANNING_INTERFACE_

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>

/** \brief This namespace includes the base class for MoveIt planners */
namespace planning_interface
{

/** \brief Base class for a MoveIt planner */
class Planner
{
public:
  
  Planner()
  {
  }
  
  virtual ~Planner() 
  {
  };
  
  /// Subclass may implement methods below
  virtual bool initialize(const robot_model::RobotModelConstPtr& model) { return true; }
  
  /// Get a short string that identifies the planning interface
  virtual std::string getDescription() const { return ""; }
  
  /// Get the names of the known planning algorithms (values that can be filled as planner_id in the planning request)
  virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const { }
  
  /// Subclass must implement methods below
  virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const MotionPlanRequest &req, MotionPlanResponse &res) const = 0;
  
  virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const MotionPlanRequest &req, MotionPlanDetailedResponse &res) const = 0;
  
  /// Determine whether this plugin instance is able to represent this planning request
  virtual bool canServiceRequest(const MotionPlanRequest &req)  const = 0;
  
  /// Request termination, if a solve() function is currently computing plans
  virtual void terminate() const = 0;
  
};

/// Shared pointer to a Planner
typedef boost::shared_ptr<Planner> PlannerPtr;

/// Shared pointer to a const Planner
typedef boost::shared_ptr<const Planner> PlannerConstPtr;

} // planning_interface

#endif
