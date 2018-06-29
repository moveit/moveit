/**
 * @file obstacle_distance_gradient.cpp
 * @brief This defines a Robot Model for the Stomp Planner.
 *
 * @author Jorge Nicho
 * @date Jul 22, 2016
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

#include <stomp_moveit/cost_functions/obstacle_distance_gradient.h>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/robot_state/conversions.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::cost_functions::ObstacleDistanceGradient,stomp_moveit::cost_functions::StompCostFunction)
static const double LONGEST_VALID_JOINT_MOVE = 0.01;

namespace stomp_moveit
{
namespace cost_functions
{

ObstacleDistanceGradient::ObstacleDistanceGradient() :
    name_("ObstacleDistanceGradient"),
    robot_state_()
{

}

ObstacleDistanceGradient::~ObstacleDistanceGradient()
{

}

bool ObstacleDistanceGradient::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                                          const std::string& group_name, XmlRpc::XmlRpcValue& config)
{
  robot_model_ptr_ = robot_model_ptr;
  group_name_ = group_name;
  collision_request_.distance = true;
  collision_request_.group_name = group_name;
  collision_request_.cost = false;
  collision_request_.max_contacts = 1;
  collision_request_.max_contacts_per_pair = 1;
  collision_request_.contacts = false;
  collision_request_.verbose = false;
  return configure(config);
}

bool ObstacleDistanceGradient::configure(const XmlRpc::XmlRpcValue& config)
{

  try
  {
    // check parameter presence
    auto members = {"cost_weight" ,"max_distance"};
    for(auto& m : members)
    {
      if(!config.hasMember(m))
      {
        ROS_ERROR("%s failed to find the '%s' parameter",getName().c_str(),m);
        return false;
      }
    }

    XmlRpc::XmlRpcValue c = config;
    max_distance_ = static_cast<double>(c["max_distance"]);
    cost_weight_ = static_cast<double>(c["cost_weight"]);
    longest_valid_joint_move_ = c.hasMember("longest_valid_joint_move") ? static_cast<double>(c["longest_valid_joint_move"]):LONGEST_VALID_JOINT_MOVE;

    if(!c.hasMember("longest_valid_joint_move"))
    {
      ROS_WARN("%s using default value for 'longest_valid_joint_move' of %f",getName().c_str(),longest_valid_joint_move_);
    }
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to parse configuration parameters",name_.c_str());
    return false;
  }

  return true;
}

bool ObstacleDistanceGradient::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                    const moveit_msgs::MotionPlanRequest &req,
                                                    const stomp_core::StompConfiguration &config,
                                                    moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  planning_scene_ = planning_scene;
  plan_request_ = req;
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  // storing robot state
  robot_state_.reset(new RobotState(robot_model_ptr_));

  if(!robotStateMsgToRobotState(req.start_state,*robot_state_,true))
  {
    ROS_ERROR("%s Failed to get current robot state from request",getName().c_str());
    return false;
  }

  // copying into intermediate robot states
  for(auto& rs : intermediate_coll_states_)
  {
    rs.reset(new RobotState(*robot_state_));
  }

  return true;
}

bool ObstacleDistanceGradient::computeCosts(const Eigen::MatrixXd& parameters, std::size_t start_timestep,
                                            std::size_t num_timesteps, int iteration_number, int rollout_number,
                                            Eigen::VectorXd& costs, bool& validity)
{

  if(!robot_state_)
  {
    ROS_ERROR("%s Robot State has not been updated",getName().c_str());
    return false;
  }



  // allocating
  costs = Eigen::VectorXd::Zero(num_timesteps);
  const moveit::core::JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);

  if(parameters.cols()<start_timestep + num_timesteps)
  {
    ROS_ERROR_STREAM("Size in the 'parameters' matrix is less than required");
    return false;
  }

  // request the distance at each state
  double dist;
  bool skip_next_check = false;
  validity = true;
  for (auto t=start_timestep; t<start_timestep + num_timesteps; ++t)
  {

    if(!skip_next_check)
    {
      collision_result_.clear();
      robot_state_->setJointGroupPositions(joint_group,parameters.col(t));
      robot_state_->update();
      collision_result_.distance = max_distance_;

      planning_scene_->checkSelfCollision(collision_request_,collision_result_,*robot_state_,planning_scene_->getAllowedCollisionMatrix());
      dist = collision_result_.collision ? -1.0 :collision_result_.distance ;

      if(dist >= max_distance_)
      {
        costs(t) = 0; // away from obstacle
      }
      else if(dist < 0)
      {
        costs(t) = 1.0; // in collision
        validity = false;
      }
      else
      {
        costs(t) = (max_distance_ - dist)/max_distance_;
      }
    }

    skip_next_check = false;

    // check intermediate poses to the next position (skip the last one)
    if(t  < start_timestep + num_timesteps - 1)
    {
      if(!checkIntermediateCollisions(parameters.col(t),parameters.col(t+1),longest_valid_joint_move_))
      {
        costs(t) = 1.0;
        costs(t+1) = 1.0;
        validity = false;
        skip_next_check = true;
      }
      else
      {
        skip_next_check = false;
      }
    }

  }

  return true;
}

bool ObstacleDistanceGradient::checkIntermediateCollisions(const Eigen::VectorXd& start,
                                                           const Eigen::VectorXd& end,double longest_valid_joint_move)
{
  Eigen::VectorXd diff = end - start;
  int num_intermediate = std::ceil(((diff.cwiseAbs())/longest_valid_joint_move).maxCoeff()) - 1;
  if(num_intermediate < 1.0)
  {
    // no interpolation needed
    return true;
  }

  // grabbing states
  auto& start_state = intermediate_coll_states_[0];
  auto& mid_state = intermediate_coll_states_[1];
  auto& end_state = intermediate_coll_states_[2];

  if(!start_state || !mid_state || !end_state)
  {
    ROS_ERROR("%s intermediate states not initialized",getName().c_str());
    return false;
  }

  // setting up collision
  auto req = collision_request_;
  req.distance = false;
  collision_detection::CollisionResult res;
  const moveit::core::JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);
  start_state->setJointGroupPositions(joint_group,start);
  end_state->setJointGroupPositions(joint_group,end);

  // checking intermediate states
  double dt = 1.0/static_cast<double>(num_intermediate);
  double interval = 0.0;
  for(std::size_t i = 1; i < num_intermediate;i++)
  {
    interval = i*dt;
    start_state->interpolate(*end_state,interval,*mid_state) ;
    if(planning_scene_->isStateColliding(*mid_state))
    {
      return false;
    }
  }

  return true;
}

void ObstacleDistanceGradient::done(bool success,int total_iterations,double final_cost,const Eigen::MatrixXd& parameters)
{
  robot_state_.reset();
}

} /* namespace cost_functions */
} /* namespace stomp_moveit */
