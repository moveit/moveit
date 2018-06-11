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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Mrinal Kalakrishnan */

#include <ros/ros.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <iostream>
#include <stdio.h>
using namespace std;

namespace chomp
{
ChompTrajectory::ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, double duration,
                                 double discretization, std::string group_name)
  : planning_group_name_(group_name)
  , num_points_((duration / discretization) + 1)
  , discretization_(discretization)
  , duration_(duration)
  , start_index_(1)
  , end_index_(num_points_ - 2)
{
  const moveit::core::JointModelGroup* model_group = robot_model->getJointModelGroup(planning_group_name_);
  num_joints_ = model_group->getActiveJointModels().size();
  init();
}

ChompTrajectory::ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, int num_points,
                                 double discretization, std::string group_name)
  : planning_group_name_(group_name)
  , num_points_(num_points)
  , discretization_(discretization)
  , duration_((num_points - 1) * discretization)
  , start_index_(1)
  , end_index_(num_points_ - 2)
{
  const moveit::core::JointModelGroup* model_group = robot_model->getJointModelGroup(planning_group_name_);
  num_joints_ = model_group->getActiveJointModels().size();
  init();
  std::cout << trajectory_ << " complete initialized TRAJECTORY..!!!!!" << std::endl;
}

/**
 * this contructor simply copies over the source trajectory into the current trajectory object
 * @param source_traj
 * @param planning_group
 * @param diff_rule_length
 */
ChompTrajectory::ChompTrajectory(const ChompTrajectory& source_traj, const std::string& planning_group,
                                 int diff_rule_length)
  : planning_group_name_(planning_group), discretization_(source_traj.discretization_)
{
  num_joints_ = source_traj.getNumJoints();

  // figure out the num_points_:
  // we need diff_rule_length-1 extra points on either side:
  int start_extra = (diff_rule_length - 1) - source_traj.start_index_;
  int end_extra = (diff_rule_length - 1) - ((source_traj.num_points_ - 1) - source_traj.end_index_);

  num_points_ = source_traj.num_points_ + start_extra + end_extra;
  start_index_ = diff_rule_length - 1;
  end_index_ = (num_points_ - 1) - (diff_rule_length - 1);
  duration_ = (num_points_ - 1) * discretization_;

  // allocate the memory:
  init();

  full_trajectory_index_.resize(num_points_);

  // now copy the trajectories over:
  for (int i = 0; i < num_points_; i++)
  {
    int source_traj_point = i - start_extra;
    if (source_traj_point < 0)
      source_traj_point = 0;
    if (source_traj_point >= source_traj.num_points_)
      source_traj_point = source_traj.num_points_ - 1;
    full_trajectory_index_[i] = source_traj_point;
    for (int j = 0; j < num_joints_; j++)
    {
      (*this)(i, j) = source_traj(source_traj_point, j);
    }
  }
}

ChompTrajectory::ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, const std::string& planning_group,
                                 const trajectory_msgs::JointTrajectory& traj)
  : planning_group_name_(planning_group)
{
  const moveit::core::JointModelGroup* model_group = robot_model->getJointModelGroup(planning_group_name_);
  num_joints_ = model_group->getActiveJointModels().size();
  double discretization = (traj.points[1].time_from_start - traj.points[0].time_from_start).toSec();

  double discretization2 = (traj.points[2].time_from_start - traj.points[1].time_from_start).toSec();

  if (fabs(discretization2 - discretization) > .001)
  {
    ROS_WARN_STREAM("Trajectory Discretization not constant " << discretization << " " << discretization2);
  }
  discretization_ = discretization;

  num_points_ = traj.points.size() + 1;
  duration_ = (traj.points.back().time_from_start - traj.points[0].time_from_start).toSec();

  start_index_ = 1;
  end_index_ = num_points_ - 2;

  init();

  for (int i = 0; i < num_points_; i++)
  {
    for (int j = 0; j < num_joints_; j++)
    {
      trajectory_(i, j) = 0.0;
    }
  }
  overwriteTrajectory(traj);
}

ChompTrajectory::~ChompTrajectory()
{
}

/**
 * copies the trajectory of the argument in the object's trajectory
 * @param traj
 */
void ChompTrajectory::overwriteTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
  for (unsigned int i = 1; i <= traj.points.size(); i++)
  {
    for (unsigned int j = 0; j < traj.joint_names.size(); j++)
    {
      trajectory_(i, j) = traj.points[i - 1].positions[j];
    }
  }
}

void ChompTrajectory::init()
{
  trajectory_.resize(num_points_, num_joints_);
  trajectory_ = Eigen::MatrixXd(num_points_, num_joints_);
}

void ChompTrajectory::updateFromGroupTrajectory(const ChompTrajectory& group_trajectory)
{
  int num_vars_free = end_index_ - start_index_ + 1;
  for (int i = 0; i < num_joints_; i++)
  {
    trajectory_.block(start_index_, i, num_vars_free, 1) =
        group_trajectory.trajectory_.block(group_trajectory.start_index_, i, num_vars_free, 1);
  }
}


void ChompTrajectory::fillInLinearInterpolation()
{
  double start_index  = start_index_ - 1;
  double end_index    = end_index_ + 1;
  int time_steps = end_index - start_index;
  for(int i = 0; i < num_joints_ ; i++)
  {
    double theta = ((*this)(end_index, i) - (*this)(start_index,i)) / (end_index- 1);
    for(int j=start_index+1 ; j< end_index ; j++)
    {
      (*this)(j,i) = (*this)(start_index,i) + j*theta;
    }
  }
}

void ChompTrajectory::fillInCubicInterpolation()
{
  double start_index  = start_index_ - 1;
  double end_index    = end_index_ + 1;
  double dt = 0.001;
  std::vector<double> coeffs(4,0);
  double total_time = (end_index - 1) * dt;
  for(int i = 0; i < num_joints_; i++)
  {
    coeffs[0] = (*this)(start_index,i);
    coeffs[2] = (3/(pow(total_time,2))) * ((*this)(end_index, i) - (*this)(start_index,i));
    coeffs[3] = (-2/(pow(total_time,3))) * ((*this)(end_index, i) - (*this)(start_index,i));

    double t;
    for(int j = start_index+1 ; j < end_index; j++)
    {
      t = j*dt;
      (*this)(j,i) = coeffs[0] + coeffs[2]*pow(t,2) + coeffs[3]*pow(t,3);
    }
  }
}

void ChompTrajectory::fillInMinJerk()
{
  double start_index = start_index_ - 1;
  double end_index = end_index_ + 1;
  double T[6];  // powers of the time duration
  T[0] = 1.0;
  T[1] = (end_index - start_index) * discretization_;

  for (int i = 2; i <= 5; i++)
    T[i] = T[i - 1] * T[1];

  // calculate the spline coefficients for each joint:
  // (these are for the special case of zero start and end vel and acc)
  double coeff[num_joints_][6];
  for (int i = 0; i < num_joints_; i++)
  {
    double x0 = (*this)(start_index, i);
    double x1 = (*this)(end_index, i);
    coeff[i][0] = x0;
    coeff[i][1] = 0;
    coeff[i][2] = 0;
    coeff[i][3] = (-20 * x0 + 20 * x1) / (2 * T[3]);
    coeff[i][4] = (30 * x0 - 30 * x1) / (2 * T[4]);
    coeff[i][5] = (-12 * x0 + 12 * x1) / (2 * T[5]);
  }

  // now fill in the joint positions at each time step
  for (int i = start_index + 1; i < end_index; i++)
  {
    double t[6];  // powers of the time index point
    t[0] = 1.0;
    t[1] = (i - start_index) * discretization_;
    for (int k = 2; k <= 5; k++)
      t[k] = t[k - 1] * t[1];

    for (int j = 0; j < num_joints_; j++)
    {
      (*this)(i, j) = 0.0;
      for (int k = 0; k <= 5; k++)
      {
        (*this)(i, j) += t[k] * coeff[j][k];
      }
    }
  }
}

}  // namespace chomp
