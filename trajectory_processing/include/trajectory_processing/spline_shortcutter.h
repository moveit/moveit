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

/** \author Sachin Chitta */

#ifndef CUBIC_SPLINE_SHORT_CUTTER_H_
#define CUBIC_SPLINE_SHORT_CUTTER_H_

#include <trajectory_processing/trajectory_shortcutter.h>
#include <trajectory_processing/cubic_trajectory.h>
#include <trajectory_processing/linear_trajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace trajectory_processing
{

/**
 * \brief Scales the time intervals stretching them if necessary so that the trajectory conforms to velocity limits
 */
class SplineShortcutter: public TrajectoryShortcutter
{
public:

  SplineShortcutter(double discretization);
  virtual ~SplineShortcutter();
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
private:
  virtual bool parameterize(const trajectory_msgs::JointTrajectory& trajectory_in,
                            const std::vector<moveit_msgs::JointLimits> &limits,
                            spline_msgs::SplineTrajectory& spline) const = 0;

  double discretization_;

  void discretizeTrajectory(const spline_msgs::SplineTrajectory &spline, 
                            const double &discretization,
                            trajectory_msgs::JointTrajectory &joint_trajectory) const;
  bool trimTrajectory(trajectory_msgs::JointTrajectory &trajectory_out, 
                      const double &segment_start_time, 
                      const double &segment_end_time) const;
  bool findTrajectoryPointsInInterval(const trajectory_msgs::JointTrajectory &trajectory,
                                      const double &segment_start_time, 
                                      const double &segment_end_time,
                                      int &index_1,
                                      int &index_2) const;
  bool getWaypoints(const spline_msgs::SplineTrajectory &spline, 
                    trajectory_msgs::JointTrajectory &joint_trajectory) const;
  bool addToTrajectory(trajectory_msgs::JointTrajectory &trajectory_out, 
                       const trajectory_msgs::JointTrajectoryPoint &trajectory_point,
                       const ros::Duration& delta_time) const;
  
  void printTrajectory(const trajectory_msgs::JointTrajectory &joint_trajectory) const;
  
  void discretizeAndAppendSegment(const spline_msgs::SplineTrajectorySegment &spline_segment,
                                  const double &discretization,
                                  trajectory_msgs::JointTrajectory &joint_trajectory,
                                  const ros::Duration &segment_start_time,
                                  const bool &include_segment_end) const;

  double maxLInfDistance(const trajectory_msgs::JointTrajectoryPoint &start, 
                         const trajectory_msgs::JointTrajectoryPoint &end) const;
  
  void refineTrajectory(trajectory_msgs::JointTrajectory &trajectory,
                        const std::vector<moveit_msgs::JointLimits>& joint_limits) const;
 
};

class CubicSplineShortcutter : public SplineShortcutter
{
public:
  CubicSplineShortcutter(double discretization) : 
    SplineShortcutter(discretization) {};
  virtual ~CubicSplineShortcutter(){};

private:
  bool parameterize(const trajectory_msgs::JointTrajectory& trajectory_in,
                    const std::vector<moveit_msgs::JointLimits> &limits,
                    spline_msgs::SplineTrajectory& spline) const {
    trajectory_processing::CubicTrajectory traj;
    return traj.parameterize(trajectory_in, limits, spline);
  }
};

class LinearSplineShortcutter : public SplineShortcutter
{
public:
  LinearSplineShortcutter(double discretization) : 
    SplineShortcutter(discretization) {};
  virtual ~LinearSplineShortcutter(){};

private:
  bool parameterize(const trajectory_msgs::JointTrajectory& trajectory_in,
                    const std::vector<moveit_msgs::JointLimits> &limits,
                    spline_msgs::SplineTrajectory& spline) const {
    trajectory_processing::LinearTrajectory traj;
    return traj.parameterize(trajectory_in, limits, spline);
  }
};


}

#endif
