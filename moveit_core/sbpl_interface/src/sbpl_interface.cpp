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

#include <sbpl_interface/sbpl_interface.h>
#include <sbpl_interface/environment_chain3d.h>
#include <planning_models/conversions.h>

namespace sbpl_interface {

bool SBPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const moveit_msgs::GetMotionPlan::Request &req, 
                          moveit_msgs::GetMotionPlan::Response &res) const
{
  planning_models::KinematicState start_state(planning_scene->getCurrentState());
  planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);

  ros::WallTime wt = ros::WallTime::now();  
  EnvironmentChain3D* env_chain = new EnvironmentChain3D(planning_scene);
  std::cerr << "Created" << std::endl;
  env_chain->setupForMotionPlan(planning_scene, 
                                req);

  //DummyEnvironment* dummy_env = new DummyEnvironment();
  SBPLPlanner *planner = new ARAPlanner(env_chain, true);
  std::cerr << "Creation took " << (ros::WallTime::now()-wt) << std::endl;
  return false;
}

}
