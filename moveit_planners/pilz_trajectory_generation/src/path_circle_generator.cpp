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

#include "pilz_trajectory_generation/path_circle_generator.h"

namespace pilz {

std::unique_ptr<KDL::Path> PathCircleGenerator::circleFromCenter(
    const KDL::Frame &start_pose,
    const KDL::Frame &goal_pose,
    const KDL::Vector &center_point,
    double eqradius
    )
{
  double a = (start_pose.p - center_point).Norm();
  double b = (goal_pose.p - center_point).Norm();
  double c = (start_pose.p - goal_pose.p).Norm();

  if(fabs(a-b) > MAX_RADIUS_DIFF)
  {
    throw Error_MotionPlanning_CenterPointDifferentRadius();
  }

  // compute the rotation angle
  double alpha = cosines(a,b,c);

  KDL::RotationalInterpolation* rot_interpo = new KDL::RotationalInterpolation_SingleAxis();
  double old_kdl_epsilon = KDL::epsilon;
  try
  {
    KDL::epsilon = MAX_COLINEAR_NORM;
    return std::unique_ptr<KDL::Path>(new KDL::Path_Circle(start_pose,
                                           center_point,
                                           goal_pose.p,
                                           goal_pose.M,
                                           alpha,
                                           rot_interpo,
                                           eqradius,
                                           true /* take ownership of RotationalInterpolation */));
    KDL::epsilon = old_kdl_epsilon;
  }
  catch(KDL::Error_MotionPlanning &)
  {
    delete rot_interpo; // in case we could not construct the Path object, avoid a memory leak
    KDL::epsilon = old_kdl_epsilon;
    throw; // and pass the exception on to the caller
  }
}

std::unique_ptr<KDL::Path> PathCircleGenerator::circleFromInterim(
    const KDL::Frame &start_pose,
    const KDL::Frame &goal_pose,
    const KDL::Vector &interim_point,
    double eqradius
    )
{
  // compute the center point from interim point
  // triangle edges
  const KDL::Vector t = interim_point - start_pose.p;
  const KDL::Vector u = goal_pose.p - start_pose.p;
  const KDL::Vector v = goal_pose.p - interim_point;
  // triangle normal
  const KDL::Vector w = t*u; // cross product

  // circle center
  if (w.Norm() < MAX_COLINEAR_NORM)
  {
    throw KDL::Error_MotionPlanning_Circle_No_Plane();
  }
  const KDL::Vector center_point = start_pose.p + (u*dot(t,t)*dot(u,v) - t*dot(u,u)*dot(t,v))* 0.5/pow(w.Norm(),2);

  // compute the rotation angle
  // triangle edges
  const KDL::Vector t_center = center_point - start_pose.p;
  const KDL::Vector v_center = goal_pose.p - center_point;
  double a = t_center.Norm();
  double b = v_center.Norm();
  double c = u.Norm();
  double alpha = cosines(a,b,c);

  KDL::Vector kdl_aux_point(interim_point);

  // if the angle at the interim is an acute angle (<90deg), rotation angle is an obtuse angle (>90deg)
  // in this case using the interim as auxiliary point for KDL::Path_Circle can lead to a path in the wrong direction
  double interim_angle = cosines(t.Norm(), v.Norm(), u.Norm());
  if(interim_angle < M_PI/2)
  {
    alpha = 2*M_PI - alpha;

    // exclude that the goal is not colinear with start and center, then use the opposite of the goal as auxiliary point
    if ((t_center*v_center).Norm() > MAX_COLINEAR_NORM)
    {
      kdl_aux_point = 2*center_point - goal_pose.p;
    }
  }

  KDL::RotationalInterpolation* rot_interpo = new KDL::RotationalInterpolation_SingleAxis();
  try
  {
    return std::unique_ptr<KDL::Path>(new KDL::Path_Circle(start_pose,
                                                  center_point,
                                                  kdl_aux_point,
                                                  goal_pose.M,
                                                  alpha,
                                                  rot_interpo,
                                                  eqradius,
                                                  true /* take ownership of RotationalInterpolation */));
  }
  catch(KDL::Error_MotionPlanning &) // LCOV_EXCL_START // The cases where KDL would throw are already handled above,
                                                        // we keep these lines to be safe
  {
    delete rot_interpo; // in case we could not construct the Path object, avoid a memory leak
    throw; // and pass the exception on to the caller
    // LCOV_EXCL_STOP
  }
}

double PathCircleGenerator::cosines(const double a, const double b, const double c)
{
   return acos(std::max(std::min((pow(a,2) + pow(b,2) - pow(c,2))/(2.0*a*b), 1.0), -1.0));
}

} // namespace pilz
