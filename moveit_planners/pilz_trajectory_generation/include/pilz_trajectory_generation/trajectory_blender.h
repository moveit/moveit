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

#ifndef TRAJECTORY_BLENDER_H
#define TRAJECTORY_BLENDER_H

#include <moveit/robot_model/robot_model.h>
#include "pilz_trajectory_generation/limits_container.h"

#include "pilz_trajectory_generation/trajectory_blend_request.h"
#include "pilz_trajectory_generation/trajectory_blend_response.h"

namespace pilz {

/**
 * @brief Base class of trajectory blenders
 */
class TrajectoryBlender
{
public:

  TrajectoryBlender(const pilz::LimitsContainer& planner_limits)
    :limits_(planner_limits)
  {
  }

  virtual ~TrajectoryBlender(){}

  /**
   * @brief Blend two robot trajectories with the given blending radius
   * @param req: trajectory blend request
   * @param res: trajectroy blend response
   * @return true if blend succeed
   */
  virtual bool blend(const pilz::TrajectoryBlendRequest& req,
                     pilz::TrajectoryBlendResponse& res) = 0;

protected:
  const pilz::LimitsContainer limits_;
};

typedef std::unique_ptr<TrajectoryBlender> TrajectoryBlenderUniquePtr;

}

#endif // TRAJECTORY_BLENDER_H
