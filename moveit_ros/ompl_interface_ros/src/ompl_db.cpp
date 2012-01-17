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

#include "ompl_interface_ros/ompl_interface_ros.h"
#include "planning_scene_monitor/planning_scene_monitor.h"
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_planning");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    tf::TransformListener tf;
    planning_scene_monitor::PlanningSceneMonitor psm("robot_description", &tf);
    psm.startWorldGeometryMonitor();
    psm.startSceneMonitor();
    psm.startStateMonitor();

    ompl_interface_ros::OMPLInterfaceROS o(psm.getPlanningScene());
    o.printStatus();
    ompl_interface::PlanningGroupPtr pg = o.getPlanningConfiguration("right_arm");
    moveit_msgs::Constraints constr;

    constr.orientation_constraints.resize(1);
    moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
    ocm.link_name = "r_wrist_roll_link";
    ocm.orientation.header.frame_id = psm.getPlanningScene()->getPlanningFrame();
    ocm.orientation.quaternion.x = 0.0;
    ocm.orientation.quaternion.y = 0.0;
    ocm.orientation.quaternion.z = 0.0;
    ocm.orientation.quaternion.w = 1.0;
    ocm.absolute_roll_tolerance = 0.01;
    ocm.absolute_pitch_tolerance = 0.01;
    ocm.absolute_yaw_tolerance = M_PI;
    ocm.weight = 1.0;

    pg->constructValidStateDatabase(psm.getPlanningScene()->getCurrentState(), constr,
                                    100000, "/home/isucan/right_arm.ompldb");
    sleep(1);
    
    return 0;
}
