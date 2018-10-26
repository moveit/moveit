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
 *   * Neither the name of Willow Garage nor the names of its
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
#include <geometric_shapes/solid_primitive_dims.h>
#include <eigen_conversions/eigen_msg.h>

moveit_msgs::Constraints kinematic_constraints::mergeConstraints(const moveit_msgs::Constraints& first,
                                                                 const moveit_msgs::Constraints& second)
{
  moveit_msgs::Constraints r;

  // add all joint constraints that are in first but not in second
  // and merge joint constraints that are for the same joint
  for (std::size_t i = 0; i < first.joint_constraints.size(); ++i)
  {
    bool add = true;
    for (std::size_t j = 0; j < second.joint_constraints.size(); ++j)
      if (second.joint_constraints[j].joint_name == first.joint_constraints[i].joint_name)
      {
        add = false;
        // now we merge
        moveit_msgs::JointConstraint m;
        const moveit_msgs::JointConstraint& a = first.joint_constraints[i];
        const moveit_msgs::JointConstraint& b = second.joint_constraints[j];
        double low = std::max(a.position - a.tolerance_below, b.position - b.tolerance_below);
        double high = std::min(a.position + a.tolerance_above, b.position + b.tolerance_above);
        if (low > high)
          ROS_ERROR_NAMED("kinematic_constraints",
                          "Attempted to merge incompatible constraints for joint '%s'. Discarding constraint.",
                          a.joint_name.c_str());
        else
        {
          m.joint_name = a.joint_name;
          m.position =
              std::max(low, std::min((a.position * a.weight + b.position * b.weight) / (a.weight + b.weight), high));
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
  for (std::size_t i = 0; i < second.joint_constraints.size(); ++i)
  {
    bool add = true;
    for (std::size_t j = 0; j < first.joint_constraints.size(); ++j)
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
  for (std::size_t i = 0; i < second.position_constraints.size(); ++i)
    r.position_constraints.push_back(second.position_constraints[i]);

  r.orientation_constraints = first.orientation_constraints;
  for (std::size_t i = 0; i < second.orientation_constraints.size(); ++i)
    r.orientation_constraints.push_back(second.orientation_constraints[i]);

  r.visibility_constraints = first.visibility_constraints;
  for (std::size_t i = 0; i < second.visibility_constraints.size(); ++i)
    r.visibility_constraints.push_back(second.visibility_constraints[i]);

  return r;
}

bool kinematic_constraints::isEmpty(const moveit_msgs::Constraints& constr)
{
  return constr.position_constraints.empty() && constr.orientation_constraints.empty() &&
         constr.visibility_constraints.empty() && constr.joint_constraints.empty();
}

std::size_t kinematic_constraints::countIndividualConstraints(const moveit_msgs::Constraints& constr)
{
  return constr.position_constraints.size() + constr.orientation_constraints.size() +
         constr.visibility_constraints.size() + constr.joint_constraints.size();
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const robot_state::RobotState& state,
                                                                         const robot_model::JointModelGroup* jmg,
                                                                         double tolerance)
{
  return constructGoalConstraints(state, jmg, tolerance, tolerance);
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const robot_state::RobotState& state,
                                                                         const robot_model::JointModelGroup* jmg,
                                                                         double tolerance_below, double tolerance_above)
{
  moveit_msgs::Constraints goal;
  std::vector<double> vals;
  state.copyJointGroupPositions(jmg, vals);
  goal.joint_constraints.resize(vals.size());
  for (std::size_t i = 0; i < vals.size(); ++i)
  {
    goal.joint_constraints[i].joint_name = jmg->getVariableNames()[i];
    goal.joint_constraints[i].position = vals[i];
    goal.joint_constraints[i].tolerance_above = tolerance_below;
    goal.joint_constraints[i].tolerance_below = tolerance_above;
    goal.joint_constraints[i].weight = 1.0;
  }

  return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string& link_name,
                                                                         const geometry_msgs::PoseStamped& pose,
                                                                         double tolerance_pos, double tolerance_angle)
{
  moveit_msgs::Constraints goal;

  goal.position_constraints.resize(1);
  moveit_msgs::PositionConstraint& pcm = goal.position_constraints[0];
  pcm.link_name = link_name;
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  shape_msgs::SolidPrimitive& bv = pcm.constraint_region.primitives[0];
  bv.type = shape_msgs::SolidPrimitive::SPHERE;
  bv.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
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
  moveit_msgs::OrientationConstraint& ocm = goal.orientation_constraints[0];
  ocm.link_name = link_name;
  ocm.header = pose.header;
  ocm.orientation = pose.pose.orientation;
  ocm.absolute_x_axis_tolerance = tolerance_angle;
  ocm.absolute_y_axis_tolerance = tolerance_angle;
  ocm.absolute_z_axis_tolerance = tolerance_angle;
  ocm.weight = 1.0;

  return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string& link_name,
                                                                         const geometry_msgs::PoseStamped& pose,
                                                                         const std::vector<double>& tolerance_pos,
                                                                         const std::vector<double>& tolerance_angle)
{
  moveit_msgs::Constraints goal = constructGoalConstraints(link_name, pose);
  if (tolerance_pos.size() == 3)
  {
    shape_msgs::SolidPrimitive& bv = goal.position_constraints[0].constraint_region.primitives[0];
    bv.type = shape_msgs::SolidPrimitive::BOX;
    bv.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_X] = tolerance_pos[0];
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = tolerance_pos[1];
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tolerance_pos[2];
  }
  if (tolerance_angle.size() == 3)
  {
    moveit_msgs::OrientationConstraint& ocm = goal.orientation_constraints[0];
    ocm.absolute_x_axis_tolerance = tolerance_angle[0];
    ocm.absolute_y_axis_tolerance = tolerance_angle[1];
    ocm.absolute_z_axis_tolerance = tolerance_angle[2];
  }
  return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string& link_name,
                                                                         const geometry_msgs::QuaternionStamped& quat,
                                                                         double tolerance)
{
  moveit_msgs::Constraints goal;
  goal.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint& ocm = goal.orientation_constraints[0];
  ocm.link_name = link_name;
  ocm.header = quat.header;
  ocm.orientation = quat.quaternion;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;
  ocm.weight = 1.0;
  return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string& link_name,
                                                                         const geometry_msgs::PointStamped& goal_point,
                                                                         double tolerance)
{
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  return constructGoalConstraints(link_name, p, goal_point, tolerance);
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string& link_name,
                                                                         const geometry_msgs::Point& reference_point,
                                                                         const geometry_msgs::PointStamped& goal_point,
                                                                         double tolerance)
{
  moveit_msgs::Constraints goal;
  goal.position_constraints.resize(1);
  moveit_msgs::PositionConstraint& pcm = goal.position_constraints[0];
  pcm.link_name = link_name;
  pcm.target_point_offset.x = reference_point.x;
  pcm.target_point_offset.y = reference_point.y;
  pcm.target_point_offset.z = reference_point.z;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
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

bool kinematic_constraints::validatePositionConstraints(const robot_state::RobotState& state,
                                                        moveit_msgs::Constraints& c)
{
  ROS_DEBUG_NAMED("kinematic_constraints_utils", "Validating position constraints.");

  for (auto& pos_con_it : c.position_constraints)
  {
    ROS_DEBUG_NAMED("kinematic_constraints_utils", "Cycling through position constraint.");
    ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils",
                           "Current position constraint: link_name "
                               << pos_con_it.link_name << ", offset " << pos_con_it.target_point_offset.x << " "
                               << pos_con_it.target_point_offset.y << " " << pos_con_it.target_point_offset.z
                               << ". header/frame_id: " << pos_con_it.header.frame_id);
    // If the link is not found in the robot model
    if (!state.hasLinkModel(pos_con_it.link_name))
    {
      // NOTE: This could use state.knowsFrameTransform() and state.getFrameTransform(), but those don't return the
      // link_frame of the robot, and I can't think of a good name for a function that returns bool frame_exists,
      // Eigen::Affine3d transform and std::string link_name.
      // TODO(felixvd): Make this comment understandable.
      std::vector<const robot_state::AttachedBody*> bodies;
      state.getAttachedBodies(bodies);
      Eigen::Affine3d transform;
      bool transform_found = false;
      std::string robot_link_name;
      // Try finding it in attached bodies
      if (state.hasAttachedBody(pos_con_it.link_name))
      {
        ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils", "State has attached body with name "
                                                                  << pos_con_it.link_name);
        const robot_state::AttachedBody* body = state.getAttachedBody(pos_con_it.link_name);
        transform = body->getFixedTransforms()[0];
        robot_link_name = body->getAttachedLinkName();
        transform_found = true;
      }
      else  // Try finding it in attached bodies' named frames
      {
        ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils", "State has no attached body with name "
                                                                  << pos_con_it.link_name
                                                                  << ". Checking named frames.");
        for (auto body : bodies)
        {
          ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils", "Checking attached body " << body->getName());
          ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils",
                                 body->getName() << " has " << body->getNamedTransforms().size() << " named frames.");
          if (body->hasNamedTransform(pos_con_it.link_name))
          {
            transform = body->getNamedTransform(pos_con_it.link_name);
            robot_link_name = body->getAttachedLinkName();
            ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils", "State has "
                                                                      << " named frame called " << pos_con_it.link_name
                                                                      << " on attached body " << body->getName());
            transform_found = true;
          }
        }
      }

      if (!transform_found)
        return false;
      Eigen::Vector3d pos_in_link_frame,
          pos_in_original_frame(pos_con_it.target_point_offset.x, pos_con_it.target_point_offset.y,
                                pos_con_it.target_point_offset.z);

      ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils",
                             "Original position constraint in frame "
                                 << pos_con_it.link_name << " is " << pos_con_it.target_point_offset.x << " "
                                 << pos_con_it.target_point_offset.y << " " << pos_con_it.target_point_offset.z);
      pos_in_link_frame = transform * pos_in_original_frame;
      pos_con_it.link_name = robot_link_name;
      pos_con_it.target_point_offset.x = pos_in_link_frame[0];
      pos_con_it.target_point_offset.y = pos_in_link_frame[1];
      pos_con_it.target_point_offset.z = pos_in_link_frame[2];
      ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils",
                             "New position constraint in frame "
                                 << pos_con_it.link_name << " is " << pos_con_it.target_point_offset.x << " "
                                 << pos_con_it.target_point_offset.y << " " << pos_con_it.target_point_offset.z);
    }
  }
  return true;
}

bool kinematic_constraints::validateOrientationConstraints(const robot_state::RobotState& state,
                                                           moveit_msgs::Constraints& c)
{
  ROS_DEBUG_NAMED("kinematic_constraints_utils", "Validating orientation constraints.");

  for (auto& ori_con_it : c.orientation_constraints)
  {
    ROS_DEBUG_NAMED("kinematic_constraints_utils", "Cycling through orientation constraints.");
    // If the link is not found in the robot model
    if (!state.hasLinkModel(ori_con_it.link_name))
    {
      // NOTE: This could use state.knowsFrameTransform() and state.getFrameTransform(), but those don't return the
      // link_frame of the robot, and I can't think of a good name for a function that returns bool frame_exists,
      // Eigen::Affine3d transform and std::string link_name.
      // TODO(felixvd): Make this comment understandable.
      std::vector<const robot_state::AttachedBody*> bodies;
      state.getAttachedBodies(bodies);
      Eigen::Quaterniond q_body_to_link;
      bool transform_found = false;
      std::string robot_link_name;
      // Try finding it in attached bodies
      if (state.hasAttachedBody(ori_con_it.link_name))
      {
        ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils", "State has attached body with name "
                                                                  << ori_con_it.link_name);
        const robot_state::AttachedBody* body = state.getAttachedBody(ori_con_it.link_name);
        q_body_to_link = body->getFixedTransforms()[0].inverse().rotation();
        robot_link_name = body->getAttachedLinkName();
        transform_found = true;
      }
      else  // Try finding it in attached bodies' named frames
      {
        for (auto body : bodies)
        {
          if (body->hasNamedTransform(ori_con_it.link_name))
          {
            q_body_to_link = body->getNamedTransform(ori_con_it.link_name).inverse().rotation();
            robot_link_name = body->getAttachedLinkName();
            ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils", "State has "
                                                                      << " named frame called " << ori_con_it.link_name
                                                                      << " on attached body " << body->getName());
            transform_found = true;
          }
        }
      }

      if (!transform_found)
        return false;
      Eigen::Quaterniond q_target(ori_con_it.orientation.w, ori_con_it.orientation.x, ori_con_it.orientation.y,
                                  ori_con_it.orientation.z);
      Eigen::Quaterniond q_in_link = q_body_to_link * q_target;

      ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils",
                             "Original orientation constraint in frame "
                                 << ori_con_it.link_name << " is (xyz) " << ori_con_it.orientation.x << " "
                                 << ori_con_it.orientation.y << " " << ori_con_it.orientation.z);
      ori_con_it.link_name = robot_link_name;
      ori_con_it.orientation.x = q_in_link.x();
      ori_con_it.orientation.y = q_in_link.y();
      ori_con_it.orientation.z = q_in_link.z();
      ori_con_it.orientation.w = q_in_link.w();
      ROS_DEBUG_STREAM_NAMED("kinematic_constraints_utils",
                             "New orientation constraint in frame "
                                 << ori_con_it.link_name << " is (xyz)  " << ori_con_it.orientation.x << " "
                                 << ori_con_it.orientation.y << " " << ori_con_it.orientation.z);
    }
  }
  return true;
}

bool kinematic_constraints::validatePositionOrientationConstraints(const robot_state::RobotState& state,
                                                                   moveit_msgs::Constraints& c)
{
  return (validatePositionConstraints(state, c) && validateOrientationConstraints(state, c));
}
