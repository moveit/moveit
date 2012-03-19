/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <moveit_msgs/ConstructConstraintApproximation.h>

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string CONSTRUCT_CONSTRAINT_APPROXIMATION_SERVICE_NAME="ompl_planning/construct_constraint_approximation";

moveit_msgs::ConstructConstraintApproximation::Request getDualArmConstraint(void)
{  
  moveit_msgs::Constraints c;
  moveit_msgs::PositionConstraint pcm2;
  pcm2.link_name = "r_wrist_roll_link";
  pcm2.target_point_offset.x = 0.7;
  pcm2.target_point_offset.y = 0;
  pcm2.target_point_offset.z = 0;
  pcm2.constraint_region_shape.type = moveit_msgs::Shape::BOX;
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  
  pcm2.constraint_region_pose.header.frame_id = "l_wrist_roll_link";
  pcm2.constraint_region_pose.pose.position.x = 0.0;
  pcm2.constraint_region_pose.pose.position.y = 0.0;
  pcm2.constraint_region_pose.pose.position.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.x = 0.0;
  pcm2.constraint_region_pose.pose.orientation.y = 0.0;
  pcm2.constraint_region_pose.pose.orientation.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.w = 1.0;
  pcm2.weight = 1.0;
  c.position_constraints.push_back(pcm2);

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.orientation.header.frame_id = "odom_combined";
  ocm.orientation.quaternion.x = 0.5;
  ocm.orientation.quaternion.y = 0.5;
  ocm.orientation.quaternion.z = 0.5;
  ocm.orientation.quaternion.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = M_PI;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  ocm.link_name = "r_wrist_roll_link";
  ocm.orientation.header.frame_id = "l_wrist_roll_link";
  ocm.orientation.quaternion.x = 0.0;
  ocm.orientation.quaternion.y = 0.0;
  ocm.orientation.quaternion.z = 1.0;
  ocm.orientation.quaternion.w = 0.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);
  
  moveit_msgs::ConstructConstraintApproximation::Request cca;
  cca.constraint = c;
  cca.group = "arms";
  cca.state_space_parameterization = "PoseModel";
  cca.samples = 1000;
  cca.edges_per_sample = 0;  
  return cca;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "call_cca", ros::init_options::AnonymousName);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::service::waitForService(CONSTRUCT_CONSTRAINT_APPROXIMATION_SERVICE_NAME);
  ros::ServiceClient cca_service_client = nh.serviceClient<moveit_msgs::ConstructConstraintApproximation>(CONSTRUCT_CONSTRAINT_APPROXIMATION_SERVICE_NAME);

  moveit_msgs::ConstructConstraintApproximation::Request req = getDualArmConstraint();
  moveit_msgs::ConstructConstraintApproximation::Request res;
  if (cca_service_client.call(req, res))
    ROS_INFO("Done");
  else
    ROS_ERROR("Failed");
  
  return 0;
}

