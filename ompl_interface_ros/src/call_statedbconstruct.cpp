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

#include <planning_scene_monitor/planning_scene_monitor.h>
#include <ompl_interface_ros/ompl_interface_ros.h>
#include <kinematic_constraints/utils.h>
#include <moveit_msgs/ComputeConstraintSpaceApproximation.h>

static const std::string DBCONSTR_SERVICE_NAME="/ompl_db/compute_space_approximation";
static const std::string ROBOT_DESCRIPTION="robot_description";

void constructSimpleConstraint(void)
{
    ros::NodeHandle nh;
    ros::service::waitForService(DBCONSTR_SERVICE_NAME);
    ros::ServiceClient service_client = nh.serviceClient<moveit_msgs::ComputeConstraintSpaceApproximation>(DBCONSTR_SERVICE_NAME);

    moveit_msgs::ComputeConstraintSpaceApproximation::Request db_req;
    moveit_msgs::ComputeConstraintSpaceApproximation::Response db_res;
    db_req.group = "right_arm";
    db_req.filename = "constraint_demo.ompldb";
    db_req.size = 1000;
    
    moveit_msgs::Constraints &constr = db_req.constraints;    
    constr.orientation_constraints.resize(1);
    moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
    ocm.link_name = "r_wrist_roll_link";
    ocm.orientation.header.frame_id = "base_footprint";
    ocm.orientation.quaternion.x = 0.0;
    ocm.orientation.quaternion.y = 0.0;
    ocm.orientation.quaternion.z = 0.0;
    ocm.orientation.quaternion.w = 1.0;
    ocm.absolute_roll_tolerance = 0.01;
    ocm.absolute_pitch_tolerance = 0.01;
    ocm.absolute_yaw_tolerance = M_PI;
    ocm.weight = 1.0;

    service_client.call(db_req, db_res);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "call_ompl_db", ros::init_options::AnonymousName);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    constructSimpleConstraint();
    
    return 0;
}
