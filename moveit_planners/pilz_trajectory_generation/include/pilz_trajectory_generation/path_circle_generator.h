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

#ifndef PATH_CIRCLE_GENERATOR_H
#define PATH_CIRCLE_GENERATOR_H

#include <kdl/path.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/utilities/error.h>
#include <kdl/rotational_interpolation_sa.hpp>

namespace pilz {
/**
 * @brief Generator class for KDL::Path_Circle from different circle representations
 */
class PathCircleGenerator
{
public:
  /**
   * @brief set the path circle from start, goal and center point
   *
   * Note that a half circle should use interim point and cannot be defined
   * by circle center since start/goal/center points are colinear.
   * @throws KDL::Error_MotionPlanning in case start and goal have different radii to the center point.
   */
  static std::unique_ptr<KDL::Path> circleFromCenter(
      const KDL::Frame& start_pose,
      const KDL::Frame& goal_pose,
      const KDL::Vector& center_point,
      double eqradius);

  /**
   * @brief set circle from start, goal and interim point

   * @throws KDL::Error_MotionPlanning if the given points are colinear.
   */
  static std::unique_ptr<KDL::Path> circleFromInterim(
      const KDL::Frame& start_pose,
      const KDL::Frame& goal_pose,
      const KDL::Vector& interim_point,
      double eqradius);

private:
  PathCircleGenerator() {}; // no instantiation of this helper class!

  /**
   * @brief law of cosines: returns angle gamma in c² = a²+b²-2ab cos(gamma)
   *
   * @return angle in radians
   */
  static double cosines(const double a, const double b, const double c);

  static constexpr double MAX_RADIUS_DIFF {1e-2};
  static constexpr double MAX_COLINEAR_NORM {1e-5};
};

}

class Error_MotionPlanning_CenterPointDifferentRadius: public KDL::Error_MotionPlanning {
public:
  virtual const char* Description() const { return "Distances between start-center and goal-center are different."
                                                   " A circle cannot be created.";}
  virtual int GetType() const {return ERROR_CODE_CENTER_POINT_DIFFERENT_RADIUS;} // LCOV_EXCL_LINE

private:
  static constexpr int ERROR_CODE_CENTER_POINT_DIFFERENT_RADIUS {3006};
};

#endif // PATH_CIRCLE_GENERATOR_H
