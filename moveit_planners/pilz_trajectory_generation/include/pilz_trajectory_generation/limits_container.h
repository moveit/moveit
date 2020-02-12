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

#ifndef LIMITS_CONTAINER_H
#define LIMITS_CONTAINER_H

#include <math.h>
#include "pilz_trajectory_generation/cartesian_limit.h"
#include "pilz_trajectory_generation/joint_limits_container.h"

namespace pilz {

/**
 * @brief This class combines CartesianLimit and JointLimits into on single class.
 */
class LimitsContainer
{
  public:
    LimitsContainer();

    /**
     * @brief Return if this LimitsContainer has defined joint limits
     * @return True if container contains joint limits
     */
    bool hasJointLimits() const;

    /**
     * @brief Set joint limits
     * @param joint_limits
     */
    void setJointLimits(JointLimitsContainer& joint_limits);

    /**
     * @brief Obtain the Joint Limits from the container
     * @return the joint limits
     */
    const JointLimitsContainer& getJointLimitContainer() const;

    /**
     * @brief Return if this LimitsContainer has defined cartesian limits
     *
     * @return True if container contains cartesian limits including maximum velocity/acceleration/deceleration
     */
    bool hasFullCartesianLimits() const;

    /**
     * @brief Set cartesian limits
     * @param cartesian_limit
     */
    void setCartesianLimits(CartesianLimit& cartesian_limit);

    /**
     * @brief Return the cartesian limits
     * @return the cartesian limits
     */
    const CartesianLimit& getCartesianLimits() const;

  private:
    /// Flag if joint limits where set
    bool has_joint_limits_;

    /// The joint limits
    JointLimitsContainer joint_limits_;

    /// Flag if cartesian limits have been set
    bool has_cartesian_limits_;

    /// The cartesian limits
    CartesianLimit cartesian_limit_;



};

}

#endif // LIMITS_CONTAINER_H
