/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Matei Ciocarlei, Sachin Chitta */

#ifndef UNNORMALIZE_JOINT_TRAJECTORY_SHORTCUTTER_H_
#define UNNORMALIZE_JOINT_TRAJECTORY_SHORTCUTTER_H_

#include <ros/ros.h>
#include <planning_models/kinematic_model.h>
#include <trajectory_processing/trajectory_shortcutter.h>

namespace trajectory_processing
{

class UnnormalizeShortcutter : public TrajectoryShortcutter
{
public:

  UnnormalizeShortcutter(const planning_models::KinematicModelConstPtr& kmodel) :
    kmodel_(kmodel) {
  };

  ~UnnormalizeShortcutter() {};

  virtual bool shortcut(const planning_scene::PlanningSceneConstPtr& scene,
                        const std::string& group,
                        const planning_models::KinematicState* start_state,
                        const std::vector<moveit_msgs::JointLimits>& joint_limits,
                        const moveit_msgs::Constraints& path_constraints,
                        const moveit_msgs::Constraints& goal_constraints,
                        const trajectory_msgs::JointTrajectory& trajectory_in,
                        const ros::Duration& allowed_time,
                        trajectory_msgs::JointTrajectory& trajectory_out,
                        moveit_msgs::MoveItErrorCodes& error_code) const;

protected:
  
  const planning_models::KinematicModelConstPtr& kmodel_;

};

}
#endif
