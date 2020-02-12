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

#ifndef TIP_FRAME_GETTER_H
#define TIP_FRAME_GETTER_H

#include <string>
#include <cassert>
#include <stdexcept>
#include <vector>

#include "pilz_trajectory_generation/trajectory_generation_exceptions.h"

namespace pilz_trajectory_generation
{

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoSolverException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(MoreThanOneTipFrameException, moveit_msgs::MoveItErrorCodes::FAILURE);

/**
 * @returns true if the JointModelGroup has a solver, false otherwise.
 *
 * @tparam JointModelGroup aims at moveit::core::JointModelGroup
 * @throws exception in case group is null.
 */
template<class JointModelGroup>
static bool hasSolver(const JointModelGroup* group)
{
  if (group == nullptr)
  {
    throw std::invalid_argument("Group must not be null");
  }
  return group->getSolverInstance() != nullptr;
}

/**
 * @return The name of the tip frame (link) of the specified group
 * returned by the solver.
 *
 * @tparam JointModelGroup aims at moveit::core::JointModelGroup
 * @throws exception in case the group has no solver.
 * @throws exception in case the solver for the group has more than one tip frame.
 */
template<class JointModelGroup>
static const std::string& getSolverTipFrame(const JointModelGroup* group)
{
  if( !hasSolver(group) )
  {
    throw NoSolverException("No solver for group " + group->getName());
  }

  const std::vector<std::string>& tipFrames {group->getSolverInstance()->getTipFrames()};
  if (tipFrames.size() > 1)
  {
    throw MoreThanOneTipFrameException("Solver for group \"" + group->getName() +
                                       "\" has more than one tip frame");
  }
  return tipFrames.front();
}

}

#endif // TIP_FRAME_GETTER_H
