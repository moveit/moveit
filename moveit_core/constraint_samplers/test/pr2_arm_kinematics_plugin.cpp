/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <algorithm>
#include <numeric>

#include "pr2_arm_kinematics_plugin.h"

using namespace KDL;
using namespace std;

namespace pr2_arm_kinematics {

bool PR2ArmIKSolver::getCount(int &count,
                              const int &max_count,
                              const int &min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}

PR2ArmIKSolver::PR2ArmIKSolver(const urdf::ModelInterface &robot_model,
                               const std::string &root_frame_name,
                               const std::string &tip_frame_name,
                               const double &search_discretization_angle,
                               const int &free_angle):ChainIkSolverPos()
{
  search_discretization_angle_ = search_discretization_angle;
  free_angle_ = free_angle;
  root_frame_name_ = root_frame_name;
  if(!pr2_arm_ik_.init(robot_model,root_frame_name,tip_frame_name))
    active_ = false;
  else
    active_ = true;
}

int PR2ArmIKSolver::CartToJnt(const KDL::JntArray& q_init,
                              const KDL::Frame& p_in,
                              KDL::JntArray &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  std::vector<std::vector<double> > solution_ik;
  if(free_angle_ == 0)
  {
    ROS_DEBUG("Solving with %f",q_init(0));
    pr2_arm_ik_.computeIKShoulderPan(b,q_init(0),solution_ik);
  }
  else
  {
    pr2_arm_ik_.computeIKShoulderRoll(b,q_init(2),solution_ik);
  }

  if(solution_ik.empty())
    return -1;

  double min_distance = 1e6;
  int min_index = -1;

  for(int i=0; i< (int) solution_ik.size(); i++)
  {
    ROS_DEBUG("Solution : %d",(int)solution_ik.size());

    for(int j=0; j < (int)solution_ik[i].size(); j++)
    {
      ROS_DEBUG("%d: %f",j,solution_ik[i][j]);
    }
    ROS_DEBUG(" ");
    ROS_DEBUG(" ");

    double tmp_distance = computeEuclideanDistance(solution_ik[i],q_init);
    if(tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min_index = i;
    }
  }

  if(min_index > -1)
  {
    q_out.resize((int)solution_ik[min_index].size());
    for(int i=0; i < (int)solution_ik[min_index].size(); i++)
    {
      q_out(i) = solution_ik[min_index][i];
    }
    return 1;
  }
  else
    return -1;
}

int PR2ArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in,
                                    const KDL::Frame& p_in,
                                    KDL::JntArray &q_out,
                                    const double &timeout)
{
  KDL::JntArray q_init = q_in;
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((pr2_arm_ik_.solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-pr2_arm_ik_.solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,pr2_arm_ik_.solver_info_.limits[free_angle_].max_position,pr2_arm_ik_.solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    return NO_IK_SOLUTION;
  }
  return NO_IK_SOLUTION;
}

bool getKDLChain(const urdf::ModelInterface& model, const std::string &root_name, const std::string &tip_name, KDL::Chain &kdl_chain)
{
  // create robot chain from root to tip
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_name, tip_name, kdl_chain))
  {
    ROS_ERROR_STREAM("Could not initialize chain object for base " << root_name << " tip " << tip_name);
    return false;
  }
  return true;
}

Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p)
{
  Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
  for(int i=0; i < 3; i++)
  {
    for(int j=0; j<3; j++)
    {
      b(i,j) = p.M(i,j);
    }
    b(i,3) = p.p(i);
  }
  return b;
}

double computeEuclideanDistance(const std::vector<double> &array_1, const KDL::JntArray &array_2)
{
  double distance = 0.0;
  for(int i=0; i< (int) array_1.size(); i++)
  {
    distance += (array_1[i] - array_2(i))*(array_1[i] - array_2(i));
  }
  return sqrt(distance);
}

void getKDLChainInfo(const KDL::Chain &chain,
                     moveit_msgs::KinematicSolverInfo &chain_info)
{
  int i=0; // segment number
  while(i < (int)chain.getNrOfSegments())
  {
    chain_info.link_names.push_back(chain.getSegment(i).getName());
    i++;
  }
}

PR2ArmKinematicsPlugin::PR2ArmKinematicsPlugin():active_(false){}

bool PR2ArmKinematicsPlugin::isActive()
{
  if(active_)
    return true;
  return false;
}

void PR2ArmKinematicsPlugin::setRobotModel(boost::shared_ptr<urdf::ModelInterface>& robot_model)
{
  robot_model_ = robot_model;
}

bool PR2ArmKinematicsPlugin::initialize(const std::string& robot_description,
                                        const std::string& group_name,
                                        const std::string& base_name,
                                        const std::string& tip_name,
                                        double search_discretization)
{
  setValues(robot_description, group_name, base_name, tip_name,search_discretization);

  std::string xml_string;
  dimension_ = 7;

  ROS_DEBUG("Loading KDL Tree");
  if(!getKDLChain(*robot_model_.get(),base_frame_,tip_frame_,kdl_chain_))
  {
    active_ = false;
    ROS_ERROR("Could not load kdl tree");
  }
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  free_angle_ = 2;

  pr2_arm_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(*robot_model_.get(), base_frame_,tip_frame_, search_discretization_,free_angle_));
  if(!pr2_arm_ik_solver_->active_)
  {
    ROS_ERROR("Could not load ik");
    active_ = false;
  }
  else
  {
    pr2_arm_ik_solver_->getSolverInfo(ik_solver_info_);
    pr2_arm_kinematics::getKDLChainInfo(kdl_chain_,fk_solver_info_);
    fk_solver_info_.joint_names = ik_solver_info_.joint_names;

    for(unsigned int i=0; i < ik_solver_info_.joint_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics:: joint name: %s",ik_solver_info_.joint_names[i].c_str());
    }
    for(unsigned int i=0; i < ik_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics can solve IK for %s",ik_solver_info_.link_names[i].c_str());
    }
    for(unsigned int i=0; i < fk_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics can solve FK for %s",fk_solver_info_.link_names[i].c_str());
    }
    ROS_DEBUG("PR2KinematicsPlugin::active for %s",group_name.c_str());
    active_ = true;
  }
  return active_;
}

bool PR2ArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.PLANNING_FAILED;
    return false;
  }
  KDL::Frame pose_desired;
  Eigen::Affine3d tp;
  tf::poseMsgToEigen(ik_pose, tp);
  tf::transformEigenToKDL(tp, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     jnt_pos_out,
                                                     timeout);
  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
  {
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code.val = error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limit,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limit,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool PR2ArmKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses) const
{
  return false;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getJointNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return ik_solver_info_.joint_names;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getLinkNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return fk_solver_info_.link_names;
}

} // namespace
