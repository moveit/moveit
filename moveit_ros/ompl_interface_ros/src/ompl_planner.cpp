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

/* Author: Ioan Sucan, Sachin Chitta */

#include "ompl_interface_ros/ompl_interface_ros.h"
#include "planning_scene_monitor/planning_scene_monitor.h"
#include <tf/transform_listener.h>

static const std::string PLANNER_NODE_NAME="ompl_planning";          // name of node
static const std::string PLANNER_SERVICE_NAME="plan_kinematic_path"; // name of the advertised service (within the ~ namespace)
static const std::string BENCHMARK_SERVICE_NAME="benchmark_planning_problem"; // name of the advertised service (within the ~ namespace)
static const std::string ROBOT_DESCRIPTION="robot_description";      // name of the robot description (a param name, so it can be changed externally)

class OMPLPlannerService
{
public:

    OMPLPlannerService(ompl_interface_ros::OMPLInterfaceROS *ompl_interface) : nh_("~"), ompl_interface_(ompl_interface)
    {
        plan_service_ = nh_.advertiseService(PLANNER_SERVICE_NAME, &OMPLPlannerService::computePlan, this);
        benchmark_service_ = nh_.advertiseService(BENCHMARK_SERVICE_NAME, &OMPLPlannerService::computeBenchmark, this);
    }

    bool computePlan(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
    {
        ROS_INFO("Received new planning request...");
        return ompl_interface_->solve(req, res);
    }

    bool computeBenchmark(moveit_msgs::ComputePlanningBenchmark::Request &req, moveit_msgs::ComputePlanningBenchmark::Response &res)
    {
        ROS_INFO("Received new benchmark request...");
        return ompl_interface_->benchmark(req, res);
    }

    void status(void)
    {
        ompl_interface_->printStatus();
        ROS_INFO("Responding to planning and bechmark requests");
    }

private:

    ros::NodeHandle                       nh_;
    ompl_interface_ros::OMPLInterfaceROS *ompl_interface_;
    ros::ServiceServer                    plan_service_;
    ros::ServiceServer                    benchmark_service_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, PLANNER_NODE_NAME);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    tf::TransformListener tf;
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, &tf);
    if (psm.getPlanningScene()->isConfigured())
    {
        psm.startWorldGeometryMonitor();
        psm.startSceneMonitor();
        psm.startStateMonitor();

        ompl_interface_ros::OMPLInterfaceROS o(psm.getPlanningScene());
        OMPLPlannerService pservice(&o);
        pservice.status();
        ros::waitForShutdown();
    }
    else
        ROS_ERROR("Planning scene not configured");

    return 0;
}
