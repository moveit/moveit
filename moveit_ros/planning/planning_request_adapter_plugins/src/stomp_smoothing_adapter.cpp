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

/* Author: Raghavender Sahdev */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <class_loader/class_loader.hpp>
#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <stomp_moveit/stomp_planner_manager.h>
#include <ros/node_handle.h>
#include <stomp_moveit/stomp_planner.h>
#include <stomp_core/stomp.h>
#include <stomp_core/utils.h>

using namespace stomp_moveit;

namespace default_planner_request_adapters
{
class StompSmoothingAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  std::map<std::string, planning_interface::PlanningContextPtr> planners_; /**< The planners for each planning group */
  moveit::core::RobotModelPtr robot_model_;
  planning_interface::PlannerConfigurationMap pcs;
  // moveit_msgs::MoveItErrorCodes error_code;

  StompSmoothingAdapter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
  }

  virtual std::string getDescription() const
  {
    return "Fix Start State In Collision";
  }

  virtual bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest& req,
                            planning_interface::MotionPlanResponse& res,
                            std::vector<std::size_t>& added_path_index) const
  {
    // robot_model_ = planning_scene->getRobotModel();
    // std::shared_ptr<StompPlanner> planner = std::static_pointer_cast<StompPlanner>(planners_.at(req.group_name));

    // stomp_moveit::StompPlanner planner1(NULL, NULL, NULL);

    // TODO LATER THIS WEEK

    std::cout << "I am in STOMP PLanning adapter" << std::endl;

    // stomp_moveit::StompPlanner stomp("panda_arm",  )

    // const planning_scene::PlanningSceneConstPtr& planning_scene
    // const planning_interface::MotionPlanRequest& req
    // planning_interface::MotionPlanResponse& res

    planning_interface::MotionPlanDetailedResponse res2;
    robot_model::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
    std::string ns = "/move_group";

    /* StompPlannerManager manager;
     manager.initialize(robot_model, ns);
       // initializing response
       res2.description_.resize(1, "");
       res2.processing_time_.resize(1);
       res2.trajectory_.resize(1);
       res2.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
       ros::WallTime start_time = ros::WallTime::now();
       bool success = false;
       trajectory_msgs::JointTrajectory trajectory;
       Eigen::MatrixXd parameters;
       bool planning_success;
       // local stomp config copy
       //auto config_copy = stomp_config_;
       // look for seed trajectory
       Eigen::MatrixXd initial_parameters;
       //bool use_seed = getSeedParameters(initial_parameters);
 */
    return planner(planning_scene, req, res);
  }

private:
  ros::NodeHandle nh_;
};
}

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::StompSmoothingAdapter,
                            planning_request_adapter::PlanningRequestAdapter);