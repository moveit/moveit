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

/* Author: Ioan Sucan, Mario Prats */

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ComputePlanningPluginsBenchmark.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <unistd.h>
#include <fstream>

static const std::string ROBOT_DESCRIPTION = "robot_description";      // name of the robot description (a param name, so it can be changed externally)
static const std::string BENCHMARK_SERVICE_NAME = "benchmark_kinematic_problem"; // name of the advertised benchmarking service

class KinematicBenchmarkService
{
public:

  KinematicBenchmarkService(void) : scene_monitor_(ROBOT_DESCRIPTION)
  {
    // initialize a planning scene
    if (scene_monitor_.getPlanningScene())
    {
      benchmark_service_ = nh_.advertiseService(BENCHMARK_SERVICE_NAME, &KinematicBenchmarkService::computeBenchmark, this);
    }
    else
      ROS_ERROR("Unable to get the planning scene");
  }

  void collectMetrics()
  {
  }

  bool computeBenchmark(moveit_msgs::ComputePlanningPluginsBenchmark::Request &req, moveit_msgs::ComputePlanningPluginsBenchmark::Response &res)
  {
    // configure planning context
    if (req.scene.robot_model_name != scene_monitor_.getKinematicModel()->getName())
    {
      // if we have a different robot, use the world geometry only
      // clear all geometry from the scene
      scene_monitor_.getPlanningScene()->getCollisionWorld()->clearObjects();
      scene_monitor_.getPlanningScene()->getCurrentState().clearAttachedBodies();
      scene_monitor_.getPlanningScene()->getCurrentState().setToDefaultValues();

      scene_monitor_.getPlanningScene()->processPlanningSceneWorldMsg(req.scene.world);
      scene_monitor_.getPlanningScene()->setName(req.scene.name);
    }
    else
      scene_monitor_.getPlanningScene()->usePlanningSceneMsg(req.scene);

    if ( req.motion_plan_request.goal_constraints.size() == 0 ||
        req.motion_plan_request.goal_constraints[0].position_constraints.size() == 0 ||
        req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.size() == 0 ||
        req.motion_plan_request.goal_constraints[0].orientation_constraints.size() == 0)
    {
      ROS_ERROR("Invalid constraints");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    geometry_msgs::Pose ik_pose;
    ik_pose.position.x = req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.x;
    ik_pose.position.y = req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.y;
    ik_pose.position.z = req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.z;
    ik_pose.orientation.x = req.motion_plan_request.goal_constraints[0].orientation_constraints[0].orientation.x;
    ik_pose.orientation.y = req.motion_plan_request.goal_constraints[0].orientation_constraints[0].orientation.y;
    ik_pose.orientation.z = req.motion_plan_request.goal_constraints[0].orientation_constraints[0].orientation.z;
    ik_pose.orientation.w = req.motion_plan_request.goal_constraints[0].orientation_constraints[0].orientation.w;

    kinematic_state::KinematicState kinematic_state(scene_monitor_.getPlanningScene()->getCurrentState());

    //Compute IK
    ROS_INFO_STREAM("Processing goal " << req.motion_plan_request.goal_constraints[0].name << " ...");
    ros::WallTime startTime = ros::WallTime::now();
    reachable_=false;
    bool success = kinematic_state.getJointStateGroup(req.motion_plan_request.group_name)->setFromIK(ik_pose, 1,
                                                                                                     req.motion_plan_request.allowed_planning_time.toSec(),
                                                                                                     boost::bind(&KinematicBenchmarkService::isIKSolutionCollisionFree, this, _1, _2));

    if (success)
    {
      ROS_INFO("  Success!");
    }
    else if (reachable_)
    {
      ROS_INFO("  Reachable in collision");
    }
    else
    {
      ROS_INFO("  Not reachable");
    }

    //Log
    double duration = (ros::WallTime::now() - startTime).toSec();
    std::string host = getHostname();
    res.filename = req.filename.empty() ? ("moveit_benchmarks_" + host + "_" + boost::posix_time::to_iso_extended_string(startTime.toBoost()) + ".log.kin") : req.filename + ".kin";
    std::ofstream out(res.filename.c_str());
    out << "Experiment " << (scene_monitor_.getPlanningScene()->getName().empty() ? "NO_NAME" : scene_monitor_.getPlanningScene()->getName()) << std::endl;
    out << "Running on " << (host.empty() ? "UNKNOWN" : host) << std::endl;
    out << "Starting at " << boost::posix_time::to_iso_extended_string(startTime.toBoost()) << std::endl;
    out << "<<<|" << std::endl << "ROS" << std::endl << req.motion_plan_request << std::endl << "|>>>" << std::endl;
    out << req.motion_plan_request.allowed_planning_time.toSec() << " seconds per run" << std::endl;
    out << duration << " seconds spent to collect the data" << std::endl;
    out << "reachable BOOLEAN" << std::endl;
    out << "collision_free BOOLEAN" << std::endl;
    out << "total_time REAL" << std::endl;
    out << reachable_ << "; " << success << "; " << duration << std::endl;
    out.close();
    ROS_INFO("Results saved to '%s'", res.filename.c_str());

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
  }

private:

  std::string getHostname(void) const
  {
    static const int BUF_SIZE = 1024;
    char buffer[BUF_SIZE];
    int err = gethostname(buffer, sizeof(buffer));
    if (err != 0)
      return std::string();
    else
    {
      buffer[BUF_SIZE - 1] = '\0';
      return std::string(buffer);
    }
  }

  bool isIKSolutionCollisionFree(kinematic_state::JointStateGroup *group, const std::vector<double> &ik_solution)
  {
    group->setVariableValues(ik_solution);

    reachable_ = true;
    if (scene_monitor_.getPlanningScene()->isStateColliding(*group->getKinematicState(), group->getName(), false))
    {
      return false;
    }
    else
      return true;
  }


  bool reachable_;
  ros::NodeHandle nh_;
  planning_scene_monitor::PlanningSceneMonitor scene_monitor_;
  ros::ServiceServer benchmark_service_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_kinematic_benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  KinematicBenchmarkService bs;
  ros::waitForShutdown();

  return 0;
}
