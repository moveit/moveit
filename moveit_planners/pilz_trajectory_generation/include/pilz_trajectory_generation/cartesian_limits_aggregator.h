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

#ifndef CARTESIAN_LIMITS_AGGREGATOR_H
#define CARTESIAN_LIMITS_AGGREGATOR_H

#include "pilz_trajectory_generation/cartesian_limit.h"

namespace pilz {

/**
 * @brief Obtains cartesian limits from the parameter server
 */
class CartesianLimitsAggregator
{
  public:

   /**
     * @brief Loads cartesian limits from the parameter server
     *
     * The parameters are expected to be under "~/cartesian_limits" of the given node handle.
     * The following limits can be specified:
     * - "max_trans_vel", the maximum translational velocity [m/s]
     * - "max_trans_acc, the maximum translational acceleration [m/s^2]
     * - "max_trans_dec", the maximum translational deceleration (<= 0) [m/s^2]
     * - "max_rot_vel", the maximum rotational velocity [rad/s]
     * - "max_rot_acc", the maximum rotational acceleration [rad/s^2]
     * - "max_rot_dec", the maximum rotational deceleration (<= 0)[rad/s^2]
     * @param nh node handle to access the parameters
     * @return the obtained cartesian limits
     */
    static CartesianLimit getAggregatedLimits(const ros::NodeHandle& nh);
};

}

#endif // CARTESIAN_LIMITS_AGGREGATOR_H
