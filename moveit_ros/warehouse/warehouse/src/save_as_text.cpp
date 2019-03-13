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

/* Author: Ioan Sucan */

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

typedef std::pair<geometry_msgs::Point, geometry_msgs::Quaternion> LinkConstraintPair;
typedef std::map<std::string, LinkConstraintPair> LinkConstraintMap;

void collectLinkConstraints(const moveit_msgs::Constraints& constraints, LinkConstraintMap& lcmap)
{
  for (std::size_t i = 0; i < constraints.position_constraints.size(); ++i)
  {
    LinkConstraintPair lcp;
    const moveit_msgs::PositionConstraint& pc = constraints.position_constraints[i];
    lcp.first = pc.constraint_region.primitive_poses[0].position;
    lcmap[constraints.position_constraints[i].link_name] = lcp;
  }

  for (std::size_t i = 0; i < constraints.orientation_constraints.size(); ++i)
  {
    if (lcmap.count(constraints.orientation_constraints[i].link_name))
    {
      lcmap[constraints.orientation_constraints[i].link_name].second =
          constraints.orientation_constraints[i].orientation;
    }
    else
    {
      ROS_WARN("Orientation constraint for %s has no matching position constraint",
               constraints.orientation_constraints[i].link_name.c_str());
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_warehouse_as_text", ros::init_options::AnonymousName);

  boost::program_options::options_description desc;
  desc.add_options()("help", "Show help message")("host", boost::program_options::value<std::string>(), "Host for the "
                                                                                                        "DB.")(
      "port", boost::program_options::value<std::size_t>(), "Port for the DB.");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  // Set up db
  warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
  if (vm.count("host") && vm.count("port"))
    conn->setParams(vm["host"].as<std::string>(), vm["port"].as<std::size_t>());
  if (!conn->connect())
    return 1;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);

  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_warehouse::RobotStateStorage rss(conn);
  moveit_warehouse::ConstraintsStorage cs(conn);

  std::vector<std::string> scene_names;
  pss.getPlanningSceneNames(scene_names);

  for (std::size_t i = 0; i < scene_names.size(); ++i)
  {
    moveit_warehouse::PlanningSceneWithMetadata pswm;
    if (pss.getPlanningScene(pswm, scene_names[i]))
    {
      ROS_INFO("Saving scene '%s'", scene_names[i].c_str());
      psm.getPlanningScene()->setPlanningSceneMsg(static_cast<const moveit_msgs::PlanningScene&>(*pswm));
      std::ofstream fout((scene_names[i] + ".scene").c_str());
      psm.getPlanningScene()->saveGeometryToStream(fout);
      fout.close();

      std::vector<std::string> robot_state_names;
      robot_model::RobotModelConstPtr km = psm.getRobotModel();
      // Get start states for scene
      std::stringstream rsregex;
      rsregex << ".*" << scene_names[i] << ".*";
      rss.getKnownRobotStates(rsregex.str(), robot_state_names);

      // Get goal constraints for scene
      std::vector<std::string> constraint_names;

      std::stringstream csregex;
      csregex << ".*" << scene_names[i] << ".*";
      cs.getKnownConstraints(csregex.str(), constraint_names);

      if (!(robot_state_names.empty() && constraint_names.empty()))
      {
        std::ofstream qfout((scene_names[i] + ".queries").c_str());
        qfout << scene_names[i] << std::endl;
        if (!robot_state_names.empty())
        {
          qfout << "start" << std::endl;
          qfout << robot_state_names.size() << std::endl;
          for (std::size_t k = 0; k < robot_state_names.size(); ++k)
          {
            ROS_INFO("Saving start state %s for scene %s", robot_state_names[k].c_str(), scene_names[i].c_str());
            qfout << robot_state_names[k] << std::endl;
            moveit_warehouse::RobotStateWithMetadata robot_state;
            rss.getRobotState(robot_state, robot_state_names[k]);
            robot_state::RobotState ks(km);
            robot_state::robotStateMsgToRobotState(*robot_state, ks, false);
            ks.printStateInfo(qfout);
            qfout << "." << std::endl;
          }
        }

        if (!constraint_names.empty())
        {
          qfout << "goal" << std::endl;
          qfout << constraint_names.size() << std::endl;
          for (std::size_t k = 0; k < constraint_names.size(); ++k)
          {
            ROS_INFO("Saving goal %s for scene %s", constraint_names[k].c_str(), scene_names[i].c_str());
            qfout << "link_constraint" << std::endl;
            qfout << constraint_names[k] << std::endl;
            moveit_warehouse::ConstraintsWithMetadata constraints;
            cs.getConstraints(constraints, constraint_names[k]);

            LinkConstraintMap lcmap;
            collectLinkConstraints(*constraints, lcmap);
            for (LinkConstraintMap::iterator iter = lcmap.begin(); iter != lcmap.end(); iter++)
            {
              std::string link_name = iter->first;
              LinkConstraintPair lcp = iter->second;
              qfout << link_name << std::endl;
              qfout << "xyz " << lcp.first.x << " " << lcp.first.y << " " << lcp.first.z << std::endl;
              Eigen::Quaterniond orientation(lcp.second.w, lcp.second.x, lcp.second.y, lcp.second.z);
              Eigen::Vector3d rpy = orientation.matrix().eulerAngles(0, 1, 2);
              qfout << "rpy " << rpy[0] << " " << rpy[1] << " " << rpy[2] << std::endl;
            }
            qfout << "." << std::endl;
          }
        }
        qfout.close();
      }
    }
  }

  ROS_INFO("Done.");

  return 0;
}
