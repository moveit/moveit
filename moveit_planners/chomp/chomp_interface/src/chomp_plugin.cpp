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

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <chomp_interface/chomp_planning_context.h>

#include <boost/shared_ptr.hpp>

#include <pluginlib/class_list_macros.h>

namespace chomp_interface
{
class CHOMPPlannerManager : public planning_interface::PlannerManager
{
public:
  CHOMPPlannerManager() : planning_interface::PlannerManager()
  {
  }

  bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns)
  {
    // model->printModelInfo(std::cout);
    std::vector<std::string> groups = model->getJointModelGroupNames();
    ROS_INFO_STREAM("Following groups exist:");
    for (int i = 0; i < groups.size(); i++)
    {
      ROS_INFO("%s", groups[i].c_str());
      planning_contexts_[groups[i]] =
          CHOMPPlanningContextPtr(new CHOMPPlanningContext("chomp_planning_context", groups[i], model));
    }
    return true;
  }

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                            const planning_interface::MotionPlanRequest &req,
                                                            moveit_msgs::MoveItErrorCodes &error_code) const
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    if (req.group_name.empty())
    {
      ROS_ERROR("No group specified to plan for");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      ROS_ERROR("No planning scene supplied as input");
      error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    planning_contexts_.at(req.group_name)->setMotionPlanRequest(req);
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return planning_contexts_.at(req.group_name);
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest &req) const
  {
    // TODO: this is a dummy implementation
    //      capabilities.dummy = false;
    return true;
  }

  std::string getDescription() const
  {
    return "CHOMP";
  }

  void getPlanningAlgorithms(std::vector<std::string> &algs) const
  {
    algs.resize(1);
    algs[0] = "CHOMP";
  }

protected:
  std::map<std::string, CHOMPPlanningContextPtr> planning_contexts_;
};

}  // ompl_interface_ros

PLUGINLIB_EXPORT_CLASS(chomp_interface::CHOMPPlannerManager, planning_interface::PlannerManager);
