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

#include <planning_request_adapter/planning_request_adapter.h>
#include <boost/math/constants/constants.hpp>
#include <planning_models/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace default_planner_request_adapters
{

class FixStartStateBoundsPlanningRequestAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:

  static const std::string BOUNDS_PARAM_NAME;
  static const std::string DT_PARAM_NAME;
  
  FixStartStateBoundsPlanningRequestAdapter(void) : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
    if (!nh_.getParam(BOUNDS_PARAM_NAME, bounds_dist_))
    {
      bounds_dist_ = 0.05;
      ROS_INFO_STREAM("Param '" << BOUNDS_PARAM_NAME << "' was not set. Using default value: " << bounds_dist_);
    }
    else
      ROS_INFO_STREAM("Param '" << BOUNDS_PARAM_NAME << "' was set to " << bounds_dist_); 
    
    if (!nh_.getParam(DT_PARAM_NAME, max_dt_offset_))
    {
      max_dt_offset_ = 0.5;
      ROS_INFO_STREAM("Param '" << DT_PARAM_NAME << "' was not set. Using default value: " << max_dt_offset_);
    }
    else
      ROS_INFO_STREAM("Param '" << DT_PARAM_NAME << "' was set to " << max_dt_offset_);
  }
  
  virtual std::string getDescription(void) const { return "Fix Start State Bounds"; }
  
  
  virtual bool adaptAndPlan(const planning_request_adapter::PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const moveit_msgs::GetMotionPlan::Request &req, 
                            moveit_msgs::GetMotionPlan::Response &res) const
  {
    ROS_DEBUG("Running '%s'", getDescription().c_str());
    
    // get the specified start state
    planning_models::KinematicState start_state = planning_scene->getCurrentState();
    planning_models::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);

    const std::vector<planning_models::KinematicState::JointState*> &jstates = 
      planning_scene->getKinematicModel()->hasJointModelGroup(req.motion_plan_request.group_name) ? 
      start_state.getJointStateGroup(req.motion_plan_request.group_name)->getJointStateVector() : 
      start_state.getJointStateVector(); 
      
    std::map<std::string, double> update;
    std::map<std::string, double> continuous_joints;

    for (std::size_t i = 0 ; i < jstates.size() ; ++i)
    { 
      const std::vector<double> &vv = jstates[i]->getVariableValues();
      
      // Check if we have a revolute, continuous joint. If we do, then we only need to make sure
      // it is within de model's declared bounds (usually -Pi, Pi), since the values wrap around. 
      // It is possible that the encoder maintains values outside the range [-Pi, Pi], to inform
      // how many times the joint was wrapped. Because of this, we remember the offsets for continuous
      // joints, and we un-do them when the plan comes from the planner
      
      const planning_models::KinematicModel::JointModel* jm = jstates[i]->getJointModel();
      if (jm->getType() == planning_models::KinematicModel::JointModel::REVOLUTE)
        if (static_cast<const planning_models::KinematicModel::RevoluteJointModel*>(jm)->isContinuous())
        {
          double initial = vv[0];
          jstates[i]->enforceBounds();
          continuous_joints[jstates[i]->getName()] = initial - vv[0];
          continue;
        } 
      
      // Normalize yaw; no offset needs to be remembered
      if (jm->getType() == planning_models::KinematicModel::JointModel::PLANAR)
        static_cast<const planning_models::KinematicModel::PlanarJointModel*>(jm)->normalizeRotation(jstates[i]->getVariableValues());
      
      // Normalize quaternions
      if (jm->getType() == planning_models::KinematicModel::JointModel::FLOATING)
        static_cast<const planning_models::KinematicModel::FloatingJointModel*>(jm)->normalizeRotation(jstates[i]->getVariableValues());
      
      const std::vector<std::string> &vn = jstates[i]->getVariableNames();
      const std::vector<std::pair<double, double> > &vb = jstates[i]->getVariableBounds();      
      
      // we have some other type of joint; 
      for (std::size_t j = 0 ; j < vn.size() ; ++j)
      {
        // we make sure the bounds are ok
        if (vb[j].first > vv[j])
        {
          if (vb[j].first - vv[j] < bounds_dist_)
          {
            ROS_INFO("Starting state is just outside bounds (variable '%s'). Assuming within bounds.", vn[j].c_str());
            update[vn[j]] = std::min(vb[j].first + std::numeric_limits<double>::epsilon(),  vb[j].second);
          }
          else
            ROS_WARN("Variable '%s' from the starting state is outside bounds by a significant margin: %lf should be in the range [%lf, %lf] but the error is %lf (more than %lf). "
                     "Set ~%s on the param server to update the error margin.", vn[j].c_str(), vv[j], vb[j].first, vb[j].second, vb[j].first - vv[j], bounds_dist_, BOUNDS_PARAM_NAME.c_str()); 
        }
        
        if (vb[j].second < vv[j])
        {
          if (vv[j] - vb[j].second < bounds_dist_)
          {
            ROS_INFO("Starting state is just outside bounds (variable '%s'). Assuming within bounds.", vn[j].c_str());
            update[vn[j]] = std::max(vb[j].first, vb[j].second - std::numeric_limits<double>::epsilon());
          }
          else
            ROS_WARN("Variable '%s' from the starting state is outside bounds by a significant margin: %lf should be in the range [%lf, %lf] but the error is %lf (more than %lf). "
                     "Set ~%s on the param server to update the error margin.", vn[j].c_str(), vv[j], vb[j].first, vb[j].second, vv[j] - vb[j].second, bounds_dist_, BOUNDS_PARAM_NAME.c_str()); 
        }
      }
    }
    
    planning_models::KinematicStatePtr prefix;
    if (!update.empty())
    {
      prefix.reset(new planning_models::KinematicState(start_state));
      start_state.setStateValues(update);
    }
    
    // make sure we use the full, normalized start state, with potential updates applied
    moveit_msgs::GetMotionPlan::Request req2 = req;
    planning_models::kinematicStateToRobotState(start_state, req2.motion_plan_request.start_state);
    bool solved = planner(planning_scene, req2, res);
    
    // re-add the prefix state, if it was constructed
    if (prefix)
    {
      if (solved)
      {
        // heuristically decide a duration offset for the trajectory (induced by the additional point added as a prefix to the computed trajectory)
        double d = max_dt_offset_;
        if (res.trajectory.joint_trajectory.points.size() > 1 || res.trajectory.multi_dof_joint_trajectory.points.size() > 1)
        {
          double temp = (res.trajectory.joint_trajectory.points.size() > res.trajectory.multi_dof_joint_trajectory.points.size()) ? 
            res.trajectory.joint_trajectory.points.back().time_from_start.toSec() / (double)(res.trajectory.joint_trajectory.points.size() - 1) :
            res.trajectory.multi_dof_joint_trajectory.points.back().time_from_start.toSec() / (double)(res.trajectory.multi_dof_joint_trajectory.points.size() - 1);
          if (temp < d)
            d = temp;
        }
        addPrefixState(*prefix, res, d, planning_scene->getTransforms());
      }
      else
        planning_models::kinematicStateToRobotState(*prefix, res.trajectory_start);
    }

    trajectory_msgs::JointTrajectory orig_trajectory = res.trajectory.joint_trajectory;

    // re-add continuous joint offsets
    for (std::map<std::string, double>::const_iterator it = continuous_joints.begin() ; it != continuous_joints.end() ; ++it)
    {
      // update the start state with the original request value
      for (std::size_t i = 0 ; i < res.trajectory_start.joint_state.name.size() ; ++i)
        if (res.trajectory_start.joint_state.name[i] == it->first)
        {
          res.trajectory_start.joint_state.position[i] += it->second;
          break;
        }
      
      for (std::size_t i = 0 ; i < res.trajectory.joint_trajectory.joint_names.size() ; ++i)
        if (res.trajectory.joint_trajectory.joint_names[i] == it->first)
        {
          // unwrap continuous joints
          double running_offset = 0.0;
          for (std::size_t j = 1 ; j < res.trajectory.joint_trajectory.points.size() ; ++j)
          {
            if (orig_trajectory.points[j - 1].positions[i] > 
                orig_trajectory.points[j].positions[i] + boost::math::constants::pi<double>())
              running_offset += 2.0 * boost::math::constants::pi<double>();
            else
              if (orig_trajectory.points[j].positions[i] > 
                  orig_trajectory.points[j - 1].positions[i] + boost::math::constants::pi<double>())
                running_offset -= 2.0 * boost::math::constants::pi<double>();
            res.trajectory.joint_trajectory.points[j].positions[i] += running_offset;
          }
          // undo wrapping due to start state
          for (std::size_t j = 0 ; j < res.trajectory.joint_trajectory.points.size() ; ++j)
            res.trajectory.joint_trajectory.points[j].positions[i] += it->second;
          break;
        }
    }
    return solved;
  }
  
private:
  
  ros::NodeHandle nh_;    
  double bounds_dist_;
  double max_dt_offset_;
};


const std::string FixStartStateBoundsPlanningRequestAdapter::BOUNDS_PARAM_NAME = "start_state_max_bounds_error";
const std::string FixStartStateBoundsPlanningRequestAdapter::DT_PARAM_NAME = "state_state_max_dt";

}

PLUGINLIB_DECLARE_CLASS(default_planner_request_adapters, FixStartStateBoundsPlanningRequestAdapter,
                        default_planner_request_adapters::FixStartStateBoundsPlanningRequestAdapter,
                        planning_request_adapter::PlanningRequestAdapter);
