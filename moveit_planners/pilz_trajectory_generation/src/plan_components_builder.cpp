/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#include "pilz_trajectory_generation/plan_components_builder.h"

#include <cassert>

#include <pilz_trajectory_generation/tip_frame_getter.h>

namespace pilz_trajectory_generation
{

std::vector<robot_trajectory::RobotTrajectoryPtr> PlanComponentsBuilder::build() const
{
  std::vector<robot_trajectory::RobotTrajectoryPtr> res_vec {traj_cont_};
  if (traj_tail_)
  {
    assert(!res_vec.empty());
    appendWithStrictTimeIncrease(*(res_vec.back()), *traj_tail_);
  }
  return res_vec;
}

void PlanComponentsBuilder::appendWithStrictTimeIncrease(robot_trajectory::RobotTrajectory &result, const robot_trajectory::RobotTrajectory &source)
{
  if (result.empty() ||
      !pilz::isRobotStateEqual(result.getLastWayPoint(), source.getFirstWayPoint(),
                               result.getGroupName(), ROBOT_STATE_EQUALITY_EPSILON) )
  {
    result.append(source, 0.0);
    return;
  }

  for (size_t i = 1; i < source.getWayPointCount(); ++i)
  {
    result.addSuffixWayPoint(source.getWayPoint(i), source.getWayPointDurationFromPrevious(i));
  }
}

void PlanComponentsBuilder::blend(const robot_trajectory::RobotTrajectoryPtr& other,
                                  const double blend_radius)
{
  if (!blender_)
  {
    throw NoBlenderSetException("No blender set");
  }

  assert(other->getGroupName() == traj_tail_->getGroupName());

  pilz::TrajectoryBlendRequest blend_request;

  blend_request.first_trajectory = traj_tail_;
  blend_request.second_trajectory = other;
  blend_request.blend_radius = blend_radius;
  blend_request.group_name = traj_tail_->getGroupName();
  blend_request.link_name = getSolverTipFrame(model_->getJointModelGroup(blend_request.group_name));

  pilz::TrajectoryBlendResponse blend_response;
  if (!blender_->blend(blend_request, blend_response))
  {
    throw BlendingFailedException("Blending failed");
  }

  // Append the new trajectory elements
  appendWithStrictTimeIncrease(*(traj_cont_.back()),*blend_response.first_trajectory);
  traj_cont_.back()->append(*blend_response.blend_trajectory, 0.0);
  // Store the last new trajectory element for future processing
  traj_tail_ = blend_response.second_trajectory; // first for next blending segment
}

void PlanComponentsBuilder::append(const robot_trajectory::RobotTrajectoryPtr& other,
                                   const double blend_radius)
{
  if (!model_)
  {
    throw NoRobotModelSetException("No robot model set");
  }

  if (!traj_tail_)
  {
    traj_tail_ = other;
    // Reserve space in container for new trajectory
    traj_cont_.emplace_back( new robot_trajectory::RobotTrajectory(model_, other->getGroupName()) );
    return;
  }

  // Create new trajectory for every group change
  if (other->getGroupName() != traj_tail_->getGroupName())
  {
    appendWithStrictTimeIncrease(*(traj_cont_.back()), *traj_tail_);
    traj_tail_ = other;
    // Create new container element
    traj_cont_.emplace_back( new robot_trajectory::RobotTrajectory(model_, other->getGroupName()) );
    return;
  }

  // No blending
  if (blend_radius <= 0.0)
  {
    appendWithStrictTimeIncrease(*(traj_cont_.back()), *traj_tail_);
    traj_tail_ = other;
    return;
  }

  blend(other, blend_radius);
}

} // namespace pilz_trajectory_generation
