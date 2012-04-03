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

#include "kinematic_constraints/utils.h"

moveit_msgs::Constraints kinematic_constraints::mergeConstraints(const moveit_msgs::Constraints &first, const moveit_msgs::Constraints &second)
{
    moveit_msgs::Constraints r = first;

    // merge joint constraints
    for (std::size_t i = 0 ; i < second.joint_constraints.size() ; ++i)
    {
        bool keep = true;
        for (std::size_t j = 0 ; j < first.joint_constraints.size() ; ++j)
            if (second.joint_constraints[i].joint_name == first.joint_constraints[j].joint_name)
            {
                keep = false;
                break;
            }
        if (keep)
            r.joint_constraints.push_back(second.joint_constraints[i]);
    }

    // merge rest of constraints
    for (std::size_t i = 0 ; i < second.position_constraints.size() ; ++i)
        r.position_constraints.push_back(second.position_constraints[i]);

    for (std::size_t i = 0 ; i < second.orientation_constraints.size() ; ++i)
        r.orientation_constraints.push_back(second.orientation_constraints[i]);

    for (std::size_t i = 0 ; i < second.visibility_constraints.size() ; ++i)
        r.visibility_constraints.push_back(second.visibility_constraints[i]);

    return r;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const planning_models::KinematicState::JointStateGroup *jsg,
                                                                         double tolerance)
{
    return constructGoalConstraints(jsg, tolerance, tolerance);
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const planning_models::KinematicState::JointStateGroup *jsg,
                                                                         double tolerance_below, double tolerance_above)
{
    moveit_msgs::Constraints goal;

    std::map<std::string, double> vals;
    jsg->getGroupStateValues(vals);

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
    pcm.constraint_region_shape.type = shape_msgs::Shape::SPHERE;
    pcm.constraint_region_shape.dimensions.push_back(tolerance_pos);

    pcm.constraint_region_pose.header = pose.header;
    pcm.constraint_region_pose.pose.position = pose.pose.position;

    // orientation of constraint region does not affect anything, since it is a sphere
    pcm.constraint_region_pose.pose.orientation.x = 0.0;
    pcm.constraint_region_pose.pose.orientation.y = 0.0;
    pcm.constraint_region_pose.pose.orientation.z = 0.0;
    pcm.constraint_region_pose.pose.orientation.w = 1.0;
    pcm.weight = 1.0;

    goal.orientation_constraints.resize(1);
    moveit_msgs::OrientationConstraint &ocm = goal.orientation_constraints[0];
    ocm.link_name = link_name;
    ocm.orientation.header = pose.header;
    ocm.orientation.quaternion = pose.pose.orientation;
    ocm.absolute_x_axis_tolerance = tolerance_angle;
    ocm.absolute_y_axis_tolerance = tolerance_angle;
    ocm.absolute_z_axis_tolerance = tolerance_angle;
    ocm.weight = 1.0;

    return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string &link_name, const geometry_msgs::QuaternionStamped &quat, double tolerance)
{
    moveit_msgs::Constraints goal;
    goal.orientation_constraints.resize(1);
    moveit_msgs::OrientationConstraint &ocm = goal.orientation_constraints[0];
    ocm.link_name = link_name;
    ocm.orientation = quat;
    ocm.absolute_x_axis_tolerance = tolerance;
    ocm.absolute_y_axis_tolerance = tolerance;
    ocm.absolute_z_axis_tolerance = tolerance;
    ocm.weight = 1.0;
    return goal;
}

moveit_msgs::Constraints kinematic_constraints::constructGoalConstraints(const std::string &link_name, const geometry_msgs::PointStamped &point, double tolerance)
{
    moveit_msgs::Constraints goal;
    goal.position_constraints.resize(1);
    moveit_msgs::PositionConstraint &pcm = goal.position_constraints[0];
    pcm.link_name = link_name;
    pcm.target_point_offset.x = 0;
    pcm.target_point_offset.y = 0;
    pcm.target_point_offset.z = 0;
    pcm.constraint_region_shape.type = shape_msgs::Shape::SPHERE;
    pcm.constraint_region_shape.dimensions.push_back(tolerance);

    pcm.constraint_region_pose.header = point.header;
    pcm.constraint_region_pose.pose.position = point.point;

    // orientation of constraint region does not affect anything, since it is a sphere
    pcm.constraint_region_pose.pose.orientation.x = 0.0;
    pcm.constraint_region_pose.pose.orientation.y = 0.0;
    pcm.constraint_region_pose.pose.orientation.z = 0.0;
    pcm.constraint_region_pose.pose.orientation.w = 1.0;
    pcm.weight = 1.0;

    return goal;
}
