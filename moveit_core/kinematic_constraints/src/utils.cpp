/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/kinematic_constraints/utils.h>
#include <shape_tools/solid_primitive_dims.h>

moveit_msgs::Constraints kinematic_constraints::mergeConstraints(const moveit_msgs::Constraints &first, const moveit_msgs::Constraints &second)
{
  moveit_msgs::Constraints r;

  // add all joint constraints that are in first but not in second
  // and merge joint constraints that are for the same joint
  for (std::size_t i = 0 ; i < first.joint_constraints.size() ; ++i)
  {
    bool add = true;
    for (std::size_t j = 0 ; j < second.joint_constraints.size() ; ++j)
      if (second.joint_constraints[j].joint_name == first.joint_constraints[i].joint_name)
      {
        add = false;
        // now we merge
        moveit_msgs::JointConstraint m;
        const moveit_msgs::JointConstraint &a = first.joint_constraints[i];
        const moveit_msgs::JointConstraint &b = second.joint_constraints[j];
        double low = std::max(a.position - a.tolerance_below, b.position - b.tolerance_below);
        double high = std::min(a.position + a.tolerance_above, b.position + b.tolerance_above);
        if (low > high)
          logError("Attempted to merge incompatible constraints for joint '%s'. Discarding constraint.", a.joint_name.c_str());
        else
        {
          m.joint_name = a.joint_name;
          m.position = std::max(low, std::min((a.position * a.weight + b.position * b.weight) / (a.weight + b.weight), high));
          m.weight = (a.weight + b.weight) / 2.0;
          m.tolerance_above = std::max(0.0, high - m.position);
          m.tolerance_below = std::max(0.0, m.position - low);
          r.joint_constraints.push_back(m);
        }
        break;
      }
    if (add)
      r.joint_constraints.push_back(first.joint_constraints[i]);
  }

  // add all joint constraints that are in second but not in first
  for (std::size_t i = 0 ; i < second.joint_constraints.size() ; ++i)
  {
    bool add = true;
    for (std::size_t j = 0 ; j < first.joint_constraints.size() ; ++j)
      if (second.joint_constraints[i].joint_name == first.joint_constraints[j].joint_name)
      {
        add = false;
        break;
      }
    if (add)
      r.joint_constraints.push_back(second.joint_constraints[i]);
  }

  // merge rest of constraints
  r.position_constraints = first.position_constraints;
  for (std::size_t i = 0 ; i < second.position_constraints.size() ; ++i)
    r.position_constraints.push_back(second.position_constraints[i]);

  r.orientation_constraints = first.orientation_constraints;
  for (std::size_t i = 0 ; i < second.orientation_constraints.size() ; ++i)
    r.orientation_constraints.push_back(second.orientation_constraints[i]);

  r.visibility_constraints = first.visibility_constraints;
  for (std::size_t i = 0 ; i < second.visibility_constraints.size() ; ++i)
    r.visibility_constraints.push_back(second.visibility_constraints[i]);

  return r;
}

bool kinematic_constraints::isEmpty(const moveit_msgs::Constraints &constr)
{
  return constr.position_constraints.empty() && constr.orientation_constraints.empty() &&
    constr.visibility_constraints.empty() && constr.joint_constraints.empty();
}

std::size_t kinematic_constraints::countIndividualConstraints(const moveit_msgs::Constraints &constr)
{
  return constr.position_constraints.size() + constr.orientation_constraints.size() +
    constr.visibility_constraints.size() + constr.joint_constraints.size();
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const robot_state::JointStateGroup *jsg,
                                                                         double tolerance)
{
  return constructGoalConstraints(jsg, tolerance, tolerance);
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const robot_state::JointStateGroup *jsg,
                                                                         double tolerance_below, double tolerance_above)
{
  moveit_msgs::Constraints goal;

  std::map<std::string, double> vals;
  jsg->getVariableValues(vals);

  goal.joint_constraints.resize(vals.size());
  unsigned int i = 0;
  for (std::map<std::string, double>::iterator it = vals.begin() ; it != vals.end(); ++it, ++i)
  {
    goal.joint_constraints[i].joint_name = it->first;
    goal.joint_constraints[i].position = it->second;
    goal.joint_constraints[i].tolerance_above = tolerance_below;
    goal.joint_constraints[i].tolerance_below = tolerance_above;
    goal.joint_constraints[i].weight = 1.0;
  }

  return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string &link_name, const geometry_msgs::PoseStamped &pose,
                                                                         double tolerance_pos, double tolerance_angle)
{
  moveit_msgs::Constraints goal;

  goal.position_constraints.resize(1);
  moveit_msgs::PositionConstraint &pcm = goal.position_constraints[0];
  pcm.link_name = link_name;
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  shape_msgs::SolidPrimitive &bv = pcm.constraint_region.primitives[0];
  bv.type = shape_msgs::SolidPrimitive::SPHERE;
  bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
  bv.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = tolerance_pos;

  pcm.header = pose.header;
  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position = pose.pose.position;

  // orientation of constraint region does not affect anything, since it is a sphere
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  goal.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = goal.orientation_constraints[0];
  ocm.link_name = link_name;
  ocm.header = pose.header;
  ocm.orientation = pose.pose.orientation;
  ocm.absolute_x_axis_tolerance = tolerance_angle;
  ocm.absolute_y_axis_tolerance = tolerance_angle;
  ocm.absolute_z_axis_tolerance = tolerance_angle;
  ocm.weight = 1.0;

  return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string &link_name, const geometry_msgs::PoseStamped &pose,
                                                                         const std::vector<double> &tolerance_pos, const std::vector<double> &tolerance_angle)
{
  moveit_msgs::Constraints goal = constructGoalConstraints(link_name, pose);
  if (tolerance_pos.size() == 3)
  {
    shape_msgs::SolidPrimitive &bv = goal.position_constraints[0].constraint_region.primitives[0];
    bv.type = shape_msgs::SolidPrimitive::BOX;
    bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_X] = tolerance_pos[0];
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = tolerance_pos[1];
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tolerance_pos[2];
  }
  if (tolerance_angle.size() == 3)
  {
    moveit_msgs::OrientationConstraint &ocm = goal.orientation_constraints[0];
    ocm.absolute_x_axis_tolerance = tolerance_angle[0];
    ocm.absolute_y_axis_tolerance = tolerance_angle[1];
    ocm.absolute_z_axis_tolerance = tolerance_angle[2];
  }
  return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string &link_name, const geometry_msgs::QuaternionStamped &quat, double tolerance)
{
  moveit_msgs::Constraints goal;
  goal.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = goal.orientation_constraints[0];
  ocm.link_name = link_name;
  ocm.header = quat.header;
  ocm.orientation = quat.quaternion;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;
  ocm.weight = 1.0;
  return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string &link_name, const geometry_msgs::PointStamped &goal_point, double tolerance)
{
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  return constructGoalConstraints(link_name, p, goal_point, tolerance );
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string &link_name, const geometry_msgs::Point &reference_point, const geometry_msgs::PointStamped &goal_point, double tolerance)
{
  moveit_msgs::Constraints goal;
  goal.position_constraints.resize(1);
  moveit_msgs::PositionConstraint &pcm = goal.position_constraints[0];
  pcm.link_name = link_name;
  pcm.target_point_offset.x = reference_point.x;
  pcm.target_point_offset.y = reference_point.y;
  pcm.target_point_offset.z = reference_point.z;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
  pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = tolerance;

  pcm.header = goal_point.header;
  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position = goal_point.point;

  // orientation of constraint region does not affect anything, since it is a sphere
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  return goal;
}
