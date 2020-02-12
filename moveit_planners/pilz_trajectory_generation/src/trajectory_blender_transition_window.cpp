/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "pilz_trajectory_generation/trajectory_blender_transition_window.h"

#include <algorithm>
#include <math.h>

bool pilz::TrajectoryBlenderTransitionWindow::blend(const pilz::TrajectoryBlendRequest& req,
                                         pilz::TrajectoryBlendResponse& res)
{
  ROS_INFO("Start trajectory blending using transition window.");

  double sampling_time = 0.;
  if(!validateRequest(req, sampling_time, res.error_code))
  {
    ROS_ERROR("Trajectory blend request is not valid.");
    return false;
  }

  // search for intersection points of the two trajectories with the blending sphere
  // intersection points belongs to blend trajectory after blending
  std::size_t first_intersection_index;
  std::size_t second_intersection_index;
  if(!searchIntersectionPoints(req, first_intersection_index, second_intersection_index))
  {
    ROS_ERROR("Blend radius to large.");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // Select blending period and adjust the start and end point of the blend phase
  std::size_t blend_align_index;
  determineTrajectoryAlignment(req, first_intersection_index, second_intersection_index, blend_align_index);

  // blend the trajectories in Cartesian space
  pilz::CartesianTrajectory blend_trajectory_cartesian;
  blendTrajectoryCartesian(req,
                           first_intersection_index,
                           second_intersection_index,
                           blend_align_index,
                           sampling_time,
                           blend_trajectory_cartesian);

  // generate the blending trajectory in joint space
  std::map<std::string, double> initial_joint_position, initial_joint_velocity;
  for(const std::string& joint_name :
      req.first_trajectory->getFirstWayPointPtr()->getJointModelGroup(req.group_name)->getActiveJointModelNames())
  {
    initial_joint_position[joint_name]
        = req.first_trajectory->getWayPoint(first_intersection_index-1).getVariablePosition(joint_name);
    initial_joint_velocity[joint_name]
        = req.first_trajectory->getWayPoint(first_intersection_index-1).getVariableVelocity(joint_name);
  }
  trajectory_msgs::JointTrajectory blend_joint_trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  if(!generateJointTrajectory(req.first_trajectory->getFirstWayPointPtr()->getRobotModel(),
                              limits_.getJointLimitContainer(),
                              blend_trajectory_cartesian,
                              req.group_name,
                              req.link_name,
                              initial_joint_position,
                              initial_joint_velocity,
                              blend_joint_trajectory,
                              error_code,
                              true))
  {
    // LCOV_EXCL_START
    ROS_INFO("Failed to generate joint trajectory for blending trajectory.");
    res.error_code.val = error_code.val;
    return false;
    // LCOV_EXCL_STOP
  }

  res.first_trajectory = std::shared_ptr<robot_trajectory::RobotTrajectory>(new robot_trajectory::RobotTrajectory(
                                                                              req.first_trajectory->getRobotModel(),
                                                                              req.first_trajectory->getGroup()));
  res.blend_trajectory = std::shared_ptr<robot_trajectory::RobotTrajectory>(new robot_trajectory::RobotTrajectory(
                                                                              req.first_trajectory->getRobotModel(),
                                                                              req.first_trajectory->getGroup()));
  res.second_trajectory = std::shared_ptr<robot_trajectory::RobotTrajectory>(new robot_trajectory::RobotTrajectory(
                                                                               req.first_trajectory->getRobotModel(),
                                                                               req.first_trajectory->getGroup()));

  // set the three trajectories after blending in response
  // erase the points [first_intersection_index, back()] from the first trajectory
  for(size_t i = 0; i < first_intersection_index; ++i)
  {
    res.first_trajectory->insertWayPoint(i, req.first_trajectory->getWayPoint(i),
                                         req.first_trajectory->getWayPointDurationFromPrevious(i));
  }

  // append the blend trajectory
  res.blend_trajectory->setRobotTrajectoryMsg(req.first_trajectory->getFirstWayPoint(), blend_joint_trajectory);
  // copy the points [second_intersection_index, len] from the second trajectory
  for(size_t i = second_intersection_index+1; i < req.second_trajectory->getWayPointCount(); ++i)
  {
    res.second_trajectory->insertWayPoint(i-(second_intersection_index+1),
                                          req.second_trajectory->getWayPoint(i),
                                          req.second_trajectory->getWayPointDurationFromPrevious(i));
  }

  // adjust the time from start
  res.second_trajectory->setWayPointDurationFromPrevious(0, sampling_time);

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}

bool pilz::TrajectoryBlenderTransitionWindow::validateRequest(const pilz::TrajectoryBlendRequest &req,
                                                   double& sampling_time,
                                                   moveit_msgs::MoveItErrorCodes &error_code) const
{
  ROS_DEBUG("Validate the trajectory blend request.");

  // check planning group
  if (!req.first_trajectory->getRobotModel()->hasJointModelGroup(req.group_name))
  {
    ROS_ERROR_STREAM("Unknown planning group: " << req.group_name);
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }

  // check link exists
  if (!req.first_trajectory->getRobotModel()->hasLinkModel(req.link_name) &&
      !req.first_trajectory->getLastWayPoint().hasAttachedBody(req.link_name))
  {
    ROS_ERROR_STREAM("Unknown link name: " << req.link_name);
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
    return false;
  }

  if(req.blend_radius <=0 )
  {
    ROS_ERROR("Blending radius must be positive");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // end position of the first trajectory and start position of second trajectory must be the same
  if(!pilz::isRobotStateEqual(req.first_trajectory->getLastWayPoint(),
                              req.second_trajectory->getFirstWayPoint(),
                              req.group_name,
                              epsilon))
  {
    ROS_ERROR_STREAM("During blending the last point (" << req.first_trajectory->getLastWayPoint()
                     << " of the preceding and the first point of the succeding trajectory ("
                     << req.second_trajectory->getFirstWayPoint() << " do not match");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // same uniform sampling time
  if (!pilz::determineAndCheckSamplingTime(req.first_trajectory,
                                           req.second_trajectory,
                                           epsilon,
                                           sampling_time))
  {

    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  //end position of the first trajectory and start position of second trajectory must have zero velocities/accelerations
  if(!pilz::isRobotStateStationary(req.first_trajectory->getLastWayPoint(), req.group_name, epsilon) ||
     !pilz::isRobotStateStationary(req.second_trajectory->getFirstWayPoint(), req.group_name, epsilon) )
  {
    ROS_ERROR("Intersection point of the blending trajectories has non-zero velocities/accelerations.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  return true;
}

void pilz::TrajectoryBlenderTransitionWindow::blendTrajectoryCartesian(const pilz::TrajectoryBlendRequest &req,
                                                            const std::size_t first_interse_index,
                                                            const std::size_t second_interse_index,
                                                            const std::size_t blend_align_index,
                                                            double sampling_time,
                                                            pilz::CartesianTrajectory& trajectory) const
{
  // other fields of the trajectory
  trajectory.group_name = req.group_name;
  trajectory.link_name = req.link_name;

  // Pose on first trajectory
  Eigen::Isometry3d blend_sample_pose1
      = req.first_trajectory->getWayPoint(first_interse_index).getFrameTransform(req.link_name);

  // Pose on second trajectory
  Eigen::Isometry3d blend_sample_pose2
      = req.second_trajectory->getWayPoint(second_interse_index).getFrameTransform(req.link_name);

  // blend the trajectory
  double blend_sample_num = second_interse_index + blend_align_index - first_interse_index +1 ;
  pilz::CartesianTrajectoryPoint waypoint;
  geometry_msgs::Pose waypoint_pose;
  blend_sample_pose2 = req.second_trajectory->getFirstWayPoint().getFrameTransform(req.link_name);

  // Pose on blending trajectory
  Eigen::Isometry3d  blend_sample_pose;
  for(std::size_t i = 0; i < blend_sample_num; ++i)
  {
    // if the first trajectory does not reach the last sample, update
    if((first_interse_index+i) < req.first_trajectory->getWayPointCount())
    {
      blend_sample_pose1 = req.first_trajectory->getWayPoint(first_interse_index+i).getFrameTransform(req.link_name);
    }

    // if after the alignment, the second trajectory starts, update
    if((first_interse_index+i) > blend_align_index)
    {
      blend_sample_pose2
         = req.second_trajectory->getWayPoint(first_interse_index+i-blend_align_index).getFrameTransform(req.link_name);
    }

    double s = (i+1)/blend_sample_num;
    double alpha = 6*std::pow(s,5) - 15*std::pow(s,4) + 10*std::pow(s,3);

    // blend the translation
    blend_sample_pose.translation() = blend_sample_pose1.translation()
        + alpha*(blend_sample_pose2.translation() - blend_sample_pose1.translation());

    // blend the orientation
    Eigen::Quaterniond start_quat(blend_sample_pose1.rotation());
    Eigen::Quaterniond end_quat(blend_sample_pose2.rotation());
    blend_sample_pose.linear() = start_quat.slerp(alpha, end_quat).toRotationMatrix();

    // push to the trajectory
    tf::poseEigenToMsg(blend_sample_pose, waypoint_pose);
    waypoint.pose = waypoint_pose;
    waypoint.time_from_start = ros::Duration((i+1.0)*sampling_time);
    trajectory.points.push_back(waypoint);

  }
}

bool pilz::TrajectoryBlenderTransitionWindow::searchIntersectionPoints(const pilz::TrajectoryBlendRequest &req,
                                                            std::size_t &first_interse_index,
                                                            std::size_t &second_interse_index) const
{
  ROS_INFO("Search for start and end point of blending trajectory.");

  // compute the position of the center of the blend sphere
  // (last point of the first trajectory, first point of the second trajectory)
  Eigen::Isometry3d circ_pose = req.first_trajectory->getLastWayPoint().getFrameTransform(req.link_name);

  // Searh for intersection points according to distance
  if(!linearSearchIntersectionPoint(req.link_name, circ_pose.translation(), req.blend_radius,
                                    req.first_trajectory, true, first_interse_index))
  {
    ROS_ERROR_STREAM("Intersection point of first trajectory not found.");
    return false;
  }
  ROS_INFO_STREAM("Intersection point of first trajectory found, index: " << first_interse_index);

  if(!linearSearchIntersectionPoint(req.link_name, circ_pose.translation(), req.blend_radius,
                                    req.second_trajectory, false, second_interse_index))
  {
    ROS_ERROR_STREAM("Intersection point of second trajectory not found.");
    return false;
  }

  ROS_INFO_STREAM("Intersection point of second trajectory found, index: " << second_interse_index);
  return true;
}

void pilz::TrajectoryBlenderTransitionWindow::determineTrajectoryAlignment(const pilz::TrajectoryBlendRequest &req,
                                                                std::size_t first_interse_index,
                                                                std::size_t second_interse_index,
                                                                std::size_t &blend_align_index) const
{
  size_t way_point_count_1 = req.first_trajectory->getWayPointCount() - first_interse_index;
  size_t way_point_count_2 = second_interse_index+1;

  if(way_point_count_1 > way_point_count_2)
  {
    blend_align_index = req.first_trajectory->getWayPointCount() - second_interse_index -1;
  }
  else
  {
    blend_align_index = first_interse_index;
  }
}
