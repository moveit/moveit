/**
 * @file joint_limits.cpp
 * @brief This defines a joint limit filter.
 *
 * @author Jorge Nicho
 * @date April 1, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/robot_state/conversions.h>
#include <stomp_moveit/noisy_filters/joint_limits.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::noisy_filters::JointLimits,stomp_moveit::noisy_filters::StompNoisyFilter);

namespace stomp_moveit
{
namespace noisy_filters
{

JointLimits::JointLimits():
    lock_start_(true),
    lock_goal_(true)
{
  // TODO Auto-generated constructor stub

}

JointLimits::~JointLimits()
{
  // TODO Auto-generated destructor stub
}

bool JointLimits::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  using namespace moveit::core;

  robot_model_ = robot_model_ptr;
  group_name_ = group_name;

  // creating states
  start_state_.reset(new RobotState(robot_model_));
  goal_state_.reset(new RobotState(robot_model_));

  return configure(config);
}

bool JointLimits::configure(const XmlRpc::XmlRpcValue& config)
{

  // check parameter presence
  auto members = {"lock_start","lock_goal"};
  for(auto& m : members)
  {
    if(!config.hasMember(m))
    {
      ROS_ERROR("%s failed to find one or more required parameters",getName().c_str());
      return false;
    }
  }

  try
  {
    XmlRpc::XmlRpcValue c = config;
    lock_start_ = static_cast<bool>(c["lock_start"]);
    lock_goal_ = static_cast<bool>(c["lock_goal"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("JointLimits plugin failed to load parameters %s",e.getMessage().c_str());
    return false;
  }

  return true;
}

bool JointLimits::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  error_code.val = error_code.val | moveit_msgs::MoveItErrorCodes::SUCCESS;

  // saving start state
  if(!robotStateMsgToRobotState(req.start_state,*start_state_))
  {
    ROS_ERROR_STREAM("Failed to save start state");
    return false;
  }

  if(!start_state_->satisfiesBounds(robot_model_->getJointModelGroup(group_name_)))
  {
    ROS_WARN("%s Requested Start State is out of bounds",getName().c_str());
  }

  // saving goal state
  if(lock_goal_)
  {
    bool goal_state_saved = false;
    for(auto& gc: req.goal_constraints)
    {
      for(auto& jc : gc.joint_constraints)
      {
        goal_state_->setVariablePosition(jc.joint_name,jc.position);
        goal_state_saved = true;
      }

      if(!goal_state_->satisfiesBounds(robot_model_->getJointModelGroup(group_name_)))
      {
        ROS_WARN("%s Requested Goal State is out of bounds",getName().c_str());
      }

      break;
    }

    if(!goal_state_saved)
    {
      ROS_ERROR_STREAM("Failed to save goal state");
      return false;
    }
  }

  return true;
}

bool JointLimits::filter(std::size_t start_timestep,std::size_t num_timesteps,
                         int iteration_number,int rollout_number,Eigen::MatrixXd& parameters,bool& filtered)
{
  using namespace moveit::core;

  filtered = false;
  const JointModelGroup* joint_group  = robot_model_->getJointModelGroup(group_name_);
  const std::vector<const JointModel*>& joint_models = joint_group->getActiveJointModels();
  std::size_t num_joints = joint_group->getActiveJointModelNames().size();
  if(parameters.rows() != num_joints)
  {
    ROS_ERROR("Incorrect number of joints in the 'parameters' matrix");
    return false;
  }

  if(lock_start_)
  {
    for(auto j = 0u; j < num_joints; j++)
    {
      parameters(j,0) =  *start_state_->getJointPositions(joint_models[j]);
    }

    filtered = true;
  }

  if(lock_goal_)
  {
    auto last_index = parameters.cols()-1;
    for(auto j = 0u; j < num_joints; j++)
    {
      parameters(j,last_index) =  *goal_state_->getJointPositions(joint_models[j]);
    }

    filtered = true;
  }

  double val;
  for (auto j = 0u; j < num_joints; ++j)
  {
    for (auto t=0u; t< parameters.cols(); ++t)
    {
      val = parameters(j,t);
      if(joint_models[j]->enforcePositionBounds(&val))
      {
        parameters(j,t) = val;
        filtered = true;
      }
    }
  }

  return true;
}

} /* namespace filters */
} /* namespace stomp_moveit */
