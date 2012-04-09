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

#ifndef MOVEIT_PLANNING_REQUEST_ADAPTER_
#define MOVEIT_PLANNING_REQUEST_ADAPTER_

#include <planning_interface/planning_interface.h>
#include <planning_scene/planning_scene.h>
#include <boost/function.hpp>

namespace planning_request_adapter
{

typedef boost::function<bool(const planning_scene::PlanningSceneConstPtr& planning_scene,
                             const moveit_msgs::GetMotionPlan::Request &req, 
                             moveit_msgs::GetMotionPlan::Response &res)> PlannerFn;

class PlanningRequestAdapter
{
public:

  PlanningRequestAdapter(void)
  {
  }

  virtual ~PlanningRequestAdapter()
  {
  }
  
  /// Get a short string that identifies the planning request adapter
  virtual std::string getDescription(void) const { return ""; }

  bool adaptAndPlan(const planning_interface::PlannerPtr &planner,
                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const moveit_msgs::GetMotionPlan::Request &req, 
                    moveit_msgs::GetMotionPlan::Response &res) const;
  
  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const moveit_msgs::GetMotionPlan::Request &req, 
                            moveit_msgs::GetMotionPlan::Response &res) const = 0;
protected:
  
  /// Add the state \e prefix as the first state to the trajectory result in \e res. The maximum duration to
  /// heuristically assign to the new motion segment to be created is \e max_dt_offset seconds. Use \e transforms for
  /// and frame transformations that need to be done.
  void addPrefixState(const planning_models::KinematicState &prefix, moveit_msgs::GetMotionPlan::Response &res,
                      double max_dt_offset, const planning_models::TransformsConstPtr &transforms) const;
  
};

typedef boost::shared_ptr<PlanningRequestAdapter> PlanningRequestAdapterPtr;
typedef boost::shared_ptr<const PlanningRequestAdapter> PlanningRequestAdapterConstPtr;

/// Apply a sequence of adapters to a motion plan
class PlanningRequestAdapterChain
{
public:
  PlanningRequestAdapterChain(void)
  {
  }
  
  void addAdapter(const PlanningRequestAdapterConstPtr &adapter)
  {
    adapters_.push_back(adapter);
  }
  
  bool adaptAndPlan(const planning_interface::PlannerPtr &planner,
                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const moveit_msgs::GetMotionPlan::Request &req, 
                    moveit_msgs::GetMotionPlan::Response &res) const;
  
private:
  std::vector<PlanningRequestAdapterConstPtr> adapters_;
};

  
}

#endif
