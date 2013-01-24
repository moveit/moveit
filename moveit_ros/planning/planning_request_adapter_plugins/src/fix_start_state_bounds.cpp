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

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <boost/math/constants/constants.hpp>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_state/conversions.h>
#include <class_loader/class_loader.h>
#include <ros/ros.h>

namespace default_planner_request_adapters
{

class FixStartStateBounds : public planning_request_adapter::PlanningRequestAdapter
{
public:

  static const std::string BOUNDS_PARAM_NAME;
  static const std::string DT_PARAM_NAME;
  
  FixStartStateBounds(void) : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
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
  
  
  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const moveit_msgs::MotionPlanRequest &req, 
                            moveit_msgs::MotionPlanResponse &res,
                            std::vector<std::size_t> &added_path_index) const
  {
    ROS_DEBUG("Running '%s'", getDescription().c_str());
    
    // get the specified start state
    kinematic_state::KinematicState start_state = planning_scene->getCurrentState();
    kinematic_state::robotStateToKinematicState(*planning_scene->getTransforms(), req.start_state, start_state);

    const std::vector<kinematic_state::JointState*> &jstates = 
      planning_scene->getKinematicModel()->hasJointModelGroup(req.group_name) ? 
      start_state.getJointStateGroup(req.group_name)->getJointStateVector() : 
      start_state.getJointStateVector(); 
    
    bool change_req = false;
    std::map<std::string, double> continuous_joints;
    for (std::size_t i = 0 ; i < jstates.size() ; ++i)
    { 
      // Check if we have a revolute, continuous joint. If we do, then we only need to make sure
      // it is within de model's declared bounds (usually -Pi, Pi), since the values wrap around. 
      // It is possible that the encoder maintains values outside the range [-Pi, Pi], to inform
      // how many times the joint was wrapped. Because of this, we remember the offsets for continuous
      // joints, and we un-do them when the plan comes from the planner
      
      const kinematic_model::JointModel* jm = jstates[i]->getJointModel();
      if (jm->getType() == kinematic_model::JointModel::REVOLUTE)
      {
        if (static_cast<const kinematic_model::RevoluteJointModel*>(jm)->isContinuous())
        {
          double initial = jstates[i]->getVariableValues()[0];
          jstates[i]->enforceBounds();
          double after = jstates[i]->getVariableValues()[0];
          continuous_joints[jstates[i]->getName()] = initial - after;
          if (fabs(initial - after) > std::numeric_limits<double>::epsilon())
            change_req = true;
        } 
      }
      else
        // Normalize yaw; no offset needs to be remembered
        if (jm->getType() == kinematic_model::JointModel::PLANAR)
        {   
          double initial = jstates[i]->getVariableValues()[2];
          if (static_cast<const kinematic_model::PlanarJointModel*>(jm)->normalizeRotation(jstates[i]->getVariableValues()))
            change_req = true;
        }
        else
          // Normalize quaternions
          if (jm->getType() == kinematic_model::JointModel::FLOATING)
          {
            if (static_cast<const kinematic_model::FloatingJointModel*>(jm)->normalizeRotation(jstates[i]->getVariableValues()))
              change_req = true;
          }
    }
    
    // pointer to a prefix state we could possibly add, if we detect we have to make changes
    kinematic_state::KinematicStatePtr prefix_state;
    for (std::size_t i = 0 ; i < jstates.size() ; ++i)
    {   
      if (!jstates[i]->satisfiesBounds())
      {    
        if (jstates[i]->satisfiesBounds(bounds_dist_))
        {
          if (!prefix_state)
            prefix_state.reset(new kinematic_state::KinematicState(start_state));
          jstates[i]->enforceBounds();
          change_req = true;
          ROS_INFO("Starting state is just outside bounds (joint '%s'). Assuming within bounds.", jstates[i]->getName().c_str());
        }
        else
        {
          std::stringstream joint_values;
          std::stringstream joint_bounds_low;
          std::stringstream joint_bounds_hi;
          for (std::size_t k = 0 ; k < jstates[i]->getVariableValues().size() ; ++k)
            joint_values << jstates[i]->getVariableValues()[k] << " ";
          for (std::size_t k = 0 ; k < jstates[i]->getVariableBounds().size() ; ++k)
          {
            joint_bounds_low << jstates[i]->getVariableBounds()[k].first << " ";
            joint_bounds_hi << jstates[i]->getVariableBounds()[k].second << " ";
          }
          ROS_WARN_STREAM("Joint '" << jstates[i]->getName() << "' from the starting state is outside bounds by a significant margin: [ " << joint_values.str() << "] should be in the range [ " << joint_bounds_low.str() <<
                          "], [ " << joint_bounds_hi.str() << "] but the error above the ~" << BOUNDS_PARAM_NAME << " parameter (currently set to " << bounds_dist_ << ")");
        }
      }
    }

    bool solved;
    // if we made any changes, use them
    if (change_req)
    {
      moveit_msgs::MotionPlanRequest req2 = req;
      kinematic_state::kinematicStateToRobotState(start_state, req2.start_state);
      solved = planner(planning_scene, req2, res);
    }
    else
      solved = planner(planning_scene, req, res);

    // re-add the prefix state, if it was constructed
    if (prefix_state)
    {      
      kinematic_state::kinematicStateToRobotState(*prefix_state, res.trajectory_start);
      if (solved)
      {
        // heuristically decide a duration offset for the trajectory (induced by the additional point added as a prefix to the computed trajectory)
        double d = std::min(max_dt_offset_, trajectory_processing::averageSegmentDuration(res.trajectory));
        trajectory_processing::addPrefixState(*prefix_state, res.trajectory, d, planning_scene->getTransforms());
        added_path_index.push_back(0);
      }
    }

    if (!continuous_joints.empty())
    {
      ROS_DEBUG("'%s' is now unwiding joints", getDescription().c_str());
      
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
    }
    return solved;
  }
  
private:
  
  ros::NodeHandle nh_;    
  double bounds_dist_;
  double max_dt_offset_;
};


const std::string FixStartStateBounds::BOUNDS_PARAM_NAME = "start_state_max_bounds_error";
const std::string FixStartStateBounds::DT_PARAM_NAME = "start_state_max_dt";

}

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::FixStartStateBounds,
                            planning_request_adapter::PlanningRequestAdapter);
