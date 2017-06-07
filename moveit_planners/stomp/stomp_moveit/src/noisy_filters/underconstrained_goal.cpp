/**
 * @file underconstrained_goal.cpp
 * @brief This defines a underconstrained goal filter.
 *
 * This will filter out goals that are not in the defined
 * task space.
 *
 * @author Jorge Nicho
 * @date April 21, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <stomp_moveit/noisy_filters/underconstrained_goal.h>
#include <XmlRpcException.h>
#include "stomp_moveit/utils/kinematics.h"

PLUGINLIB_EXPORT_CLASS(stomp_moveit::noisy_filters::UnderconstrainedGoal,stomp_moveit::noisy_filters::StompNoisyFilter);

static const int DOF_SIZE = 6;

namespace stomp_moveit
{
namespace noisy_filters
{

UnderconstrainedGoal::UnderconstrainedGoal():
    name_("UnderconstrainedGoal")
{

}

UnderconstrainedGoal::~UnderconstrainedGoal()
{
  // TODO Auto-generated destructor stub
}

bool UnderconstrainedGoal::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  group_name_ = group_name;
  robot_model_ = robot_model_ptr;

  return configure(config);
}

bool UnderconstrainedGoal::configure(const XmlRpc::XmlRpcValue& config)
{
  using namespace XmlRpc;

  try
  {
    XmlRpcValue params = config;

    XmlRpcValue dof_nullity_param = params["constrained_dofs"];
    XmlRpcValue dof_thresholds_param = params["cartesian_convergence"];
    XmlRpcValue joint_updates_param = params["joint_update_rates"];
    if((dof_nullity_param.getType() != XmlRpcValue::TypeArray) ||
        dof_nullity_param.size() < DOF_SIZE ||
        dof_thresholds_param.getType() != XmlRpcValue::TypeArray ||
        dof_thresholds_param.size() < DOF_SIZE  ||
        joint_updates_param.getType() != XmlRpcValue::TypeArray ||
        joint_updates_param.size() == 0)
    {
      ROS_ERROR("UnderconstrainedGoal received invalid array parameters");
      return false;
    }

    dof_nullity_.resize(DOF_SIZE);
    for(auto i = 0u; i < dof_nullity_param.size(); i++)
    {
      dof_nullity_(i) = static_cast<int>(dof_nullity_param[i]);
    }

    cartesian_convergence_thresholds_.resize(DOF_SIZE);
    for(auto i = 0u; i < dof_thresholds_param.size(); i++)
    {
      cartesian_convergence_thresholds_(i) = static_cast<double>(dof_thresholds_param[i]);
    }

    joint_update_rates_.resize(joint_updates_param.size());
    for(auto i = 0u; i < joint_updates_param.size(); i++)
    {
      joint_update_rates_(i) = static_cast<double>(joint_updates_param[i]);
    }

    max_iterations_ = static_cast<int>(params["max_ik_iterations"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("UnderconstrainedGoal failed to load parameters, %s",e.getMessage().c_str());
    return false;
  }

  return true;
}

bool UnderconstrainedGoal::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;
  using namespace moveit::core;

  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  int num_joints = joint_group->getActiveJointModels().size();
  tool_link_ = joint_group->getLinkModelNames().back();
  state_.reset(new RobotState(robot_model_));
  robotStateMsgToRobotState(req.start_state,*state_);

  const std::vector<moveit_msgs::Constraints>& goals = req.goal_constraints;
  if(goals.empty())
  {
    ROS_ERROR("A goal constraint was not provided");
    error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // storing tool goal pose
  if(goals.front().position_constraints.empty() ||
      goals.front().orientation_constraints.empty())
  {
    ROS_WARN("A goal constraint for the tool link was not provided, using forward kinematics");

    // check joint constraints
    if(goals.front().joint_constraints.empty())
    {
      ROS_ERROR_STREAM("No joint values for the goal were found");
      error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    // compute FK to obtain tool pose
    const std::vector<moveit_msgs::JointConstraint>& joint_constraints = goals.front().joint_constraints;

    // copying goal values into state
    for(auto& jc: joint_constraints)
    {
      state_->setVariablePosition(jc.joint_name,jc.position);
    }

    state_->update(true);
    state_->enforceBounds(joint_group);
    tool_goal_pose_ = state_->getGlobalLinkTransform(tool_link_);
  }
  else
  {
    // tool cartesian goal
    const moveit_msgs::PositionConstraint& pos_constraint = goals.front().position_constraints.front();
    const moveit_msgs::OrientationConstraint& orient_constraint = goals.front().orientation_constraints.front();

    geometry_msgs::Pose pose;
    pose.position = pos_constraint.constraint_region.primitive_poses[0].position;
    pose.orientation = orient_constraint.orientation;
    tf::poseMsgToEigen(pose,tool_goal_pose_);
  }

  Eigen::VectorXd joint_pose = Eigen::VectorXd::Zero(num_joints);
  ref_goal_joint_pose_.resize(num_joints);
  robotStateMsgToRobotState(req.start_state,*state_);
  state_->copyJointGroupPositions(joint_group,joint_pose);
  if(!stomp_moveit::utils::kinematics::solveIK(state_,group_name_,dof_nullity_,
                                               joint_update_rates_,cartesian_convergence_thresholds_,
                                               max_iterations_,tool_goal_pose_,joint_pose,
                                               ref_goal_joint_pose_))
  {

    ROS_WARN("%s IK failed using start pose as seed",getName().c_str());
    ref_goal_joint_pose_.resize(0);
  }



  error_code.val = error_code.SUCCESS;
  return true;
}


bool UnderconstrainedGoal::filter(std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    int rollout_number,
                    Eigen::MatrixXd& parameters,
                    bool& filtered)
{
  using namespace Eigen;
  using namespace moveit::core;
  using namespace stomp_moveit::utils;

  VectorXd init_joint_pose = parameters.rightCols(1);
  VectorXd joint_pose;

  filtered = false;

  if(!kinematics::solveIK(state_,group_name_,dof_nullity_,joint_update_rates_,cartesian_convergence_thresholds_,max_iterations_,
              tool_goal_pose_,init_joint_pose,joint_pose))
  {
    ROS_WARN("UnderconstrainedGoal failed to find valid ik close to current goal pose");

    if(ref_goal_joint_pose_.size() != 0)
    {
      joint_pose = ref_goal_joint_pose_; // assigning previously computed valid pose
      filtered = true;
      parameters.rightCols(1) = joint_pose;
    }

  }
  else
  {
    filtered = true;
    parameters.rightCols(1) = joint_pose;
  }

  return true;
}


} /* namespace filters */
} /* namespace stomp_moveit */
