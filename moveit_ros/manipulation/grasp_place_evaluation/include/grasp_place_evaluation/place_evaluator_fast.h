/*********************************************************************
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

// Author(s): E. Gil Jones

#ifndef _PLACE_EVALUATOR_FAST_
#define _PLACE_EVALUATOR_FAST_

#include <grasp_place_evaluation/place_evaluator.h>
#include <grasp_place_evaluation/interpolation_evaluator.h>

namespace grasp_place_evaluation {

class PlaceEvaluatorFast : public PlaceEvaluator, public InterpolationEvaluator 
{
public:

  PlaceEvaluatorFast(const planning_models::KinematicModelConstPtr& kmodel,
                     const std::map<std::string, kinematics::KinematicsBasePtr>& solver_map);

  ~PlaceEvaluatorFast(){};


  virtual void testPlaceLocations(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const planning_models::KinematicState* seed_state,
                                  const moveit_manipulation_msgs::PlaceGoal &place_goal, 
                                  const geometry_msgs::Vector3& retreat_direction,
                                  const std::vector<geometry_msgs::PoseStamped>& place_locations,
                                  PlaceExecutionInfoVector &execution_info_vector,
                                  bool return_on_first_hit);

};

}

#endif

