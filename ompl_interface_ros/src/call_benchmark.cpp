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

#include <planning_scene_monitor/planning_scene_monitor.h>
#include <ompl_interface_ros/ompl_interface_ros.h>
#include <moveit_msgs/ComputePlanningBenchmark.h>
#include <kinematic_constraints/utils.h>

static const std::string PLANNER_SERVICE_NAME="/ompl_planning/benchmark_planning_problem";
static const std::string ROBOT_DESCRIPTION="robot_description";

void benchmarkSimplePlan(const std::string &config)
{
    ros::NodeHandle nh;
    ros::service::waitForService(PLANNER_SERVICE_NAME);
    ros::ServiceClient benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningBenchmark>(PLANNER_SERVICE_NAME);

    moveit_msgs::ComputePlanningBenchmark::Request mplan_req;
    moveit_msgs::ComputePlanningBenchmark::Response mplan_res;

    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScene &scene = *psm.getPlanningScene();

    mplan_req.motion_plan_request.planner_id = config;
    mplan_req.motion_plan_request.group_name = "right_arm";
    mplan_req.motion_plan_request.num_planning_attempts = 100;
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
    const std::vector<std::string>& joint_names = scene.getKinematicModel()->getJointModelGroup("right_arm")->getJointModelNames();
    mplan_req.motion_plan_request.goal_constraints.resize(1);
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints.resize(joint_names.size());
    for(unsigned int i = 0; i < joint_names.size(); i++)
    {
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name = joint_names[i];
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].position = 0.0;
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_above = 0.001;
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_below = 0.001;
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].weight = 1.0;
    }
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[0].position = -2.0;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[3].position = -.2;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[5].position = -.2;

    benchmark_service_client.call(mplan_req, mplan_res);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "call_ompl_planning", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    /*    benchmarkSimplePlan("LBKPIECEkConfigDefault");
    benchmarkSimplePlan("RRTkConfigDefault");
    benchmarkSimplePlan("RRTConnectkConfigDefault");
    benchmarkSimplePlan("ESTkConfigDefault");
    benchmarkSimplePlan("KPIECEkConfigDefault");
    benchmarkSimplePlan("BKPIECEkConfigDefault");
    benchmarkSimplePlan("SBLkConfigDefault");
    benchmarkSimplePlan("RRTkConfigDefault"); */

    benchmarkSimplePlan("SBLkConfigDefault");
    benchmarkSimplePlan("SBLkConfig1");
    benchmarkSimplePlan("SBLkConfig2");
    benchmarkSimplePlan("SBLkConfig3");
    benchmarkSimplePlan("SBLkConfig4");
}
