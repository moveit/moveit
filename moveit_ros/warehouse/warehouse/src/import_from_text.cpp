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
/* Author: Raphael Bricout */
/* Author: Shadow Software Team */

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <ros/ros.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

void parseStart(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                moveit_warehouse::RobotStateStorage* rs, moveit_msgs::RobotState& startState)
{
  std::map<std::string, double> v;
  std::string joint;
  std::string marker;
  double value;
  in >> joint;
  while (joint != "." && in.good() && !in.eof())
  {
    in >> marker;
    if (marker != "=")
      joint = ".";
    else
      in >> value;
    v[joint] = value;
    if (joint != ".")
      in >> joint;
  }
  if (!v.empty())
  {
    robot_state::RobotState st = psm->getPlanningScene()->getCurrentState();
    st.setVariablePositions(v);
    moveit_msgs::RobotState msg;
    robot_state::robotStateToRobotStateMsg(st, msg);
    startState = msg;
  }
}

void parseLinkConstraint(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                         moveit_warehouse::ConstraintsStorage* cs)
{
  Eigen::Translation3d pos;
  Eigen::Quaterniond rot;

  bool have_position = false;
  bool have_orientation = false;

  std::string name;
  std::getline(in, name);

  // The link name is optional, in which case there would be a blank line
  std::string link_name;
  std::getline(in, link_name);

  std::string type;
  in >> type;

  while (type != "." && in.good() && !in.eof())
  {
    if (type == "xyz")
    {
      have_position = true;
      double x, y, z;
      in >> x >> y >> z;
      pos = Eigen::Translation3d(x, y, z);
    }
    else if (type == "rpy")
    {
      have_orientation = true;
      double r, p, y;
      in >> r >> p >> y;
      rot = Eigen::Quaterniond(Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()));
    }
    else
      ROS_ERROR("Unknown link constraint element: '%s'", type.c_str());
    in >> type;
  }

  // Convert to getLine method by eating line break
  std::string end_link;
  std::getline(in, end_link);

  if (have_position && have_orientation)
  {
    geometry_msgs::PoseStamped pose;
    tf::poseEigenToMsg(pos * rot, pose.pose);
    pose.header.frame_id = psm->getRobotModel()->getModelFrame();
    moveit_msgs::Constraints constr = kinematic_constraints::constructGoalConstraints(link_name, pose);
    constr.name = name;
    ROS_INFO("Parsed link constraint '%s'", name.c_str());
    cs->addConstraints(constr);
  }
}

void parseJointConstraint(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                          moveit_warehouse::RobotStateStorage* rs, moveit_msgs::Constraints& goalState)
{
  std::string joint;
  std::string marker;

  double pos;
  double tol_above;
  double tol_below;

  moveit_msgs::Constraints msg;
  std::vector<moveit_msgs::JointConstraint> joint_constraints;

  in >> joint;

  while (joint != "." && in.good() && !in.eof())
  {
    in >> marker;
    if (marker != "=")
      joint = ".";
    else
    {
      in >> pos;
      moveit_msgs::JointConstraint joint_constraint;
      joint_constraint.joint_name = joint;
      joint_constraint.position = pos;
      joint_constraint.weight = 1.0;
      joint_constraints.push_back(joint_constraint);
    }
    if (joint != ".")
      in >> joint;
  }
  msg.joint_constraints = joint_constraints;
  goalState = msg;
}

void parsePositionConstraint(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                             moveit_warehouse::RobotStateStorage* rs, moveit_msgs::Constraints& goalState)
{
  std::string label;
  std::string marker;

  geometry_msgs::Quaternion orientation;
  geometry_msgs::Point position;
  std::string eef_name = "";

  in >> label;
  in >> marker;
  if (label == "End_effector" && marker == "=")
  {
    in >> eef_name;
  }

  in >> label;
  in >> marker;
  if (label == "Position" && marker == "=")
  {
    in >> position.x >> position.y >> position.z;
  }
  in >> label;
  in >> marker;
  if (label == "Orientation" && marker == "=")
  {
    in >> orientation.x;
    in >> orientation.y;
    in >> orientation.z;
    in >> orientation.w;
  }
  in >> label;
  if (label != ".")
  {
    ROS_ERROR("Parsing failed.");
  }

  robot_model::RobotModelConstPtr km = psm->getRobotModel();

  const std::vector<std::string>& variable_names = km->getVariableNames();
  const std::vector<std::string>& id_names = km->getLinkModelNames();
  const robot_model::JointModel* eef_joint = km->getJointModel(variable_names[variable_names.size() - 1]);

  geometry_msgs::PoseStamped pose;
  pose.pose.orientation = orientation;
  pose.pose.position = position;
  pose.header.frame_id = id_names[0];

  moveit_msgs::Constraints msg;
  msg = kinematic_constraints::constructGoalConstraints(eef_name, pose);
  goalState = msg;
}

void parseQueries(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                  moveit_warehouse::RobotStateStorage* rs, moveit_warehouse::ConstraintsStorage* cs,
                  moveit_warehouse::PlanningSceneStorage* pss)
{
  std::string scene_name;
  in >> scene_name;

  if (pss->hasPlanningScene(scene_name))
  {
    while (in.good() && !in.eof())
    {
      std::string query_name;
      in >> query_name;

      moveit_msgs::RobotState startState;
      moveit_msgs::Constraints goalState;

      if (in.good() && !in.eof())
      {
        std::string start_type;
        in >> start_type;

        // Get the start state of the query
        if (boost::iequals(start_type, "start") && in.good() && !in.eof())
        {
          parseStart(in, psm, rs, startState);
        }
        else
        {
          ROS_ERROR("Unknown start type: '%s'", start_type.c_str());
        }

        std::string goal_type;
        in >> goal_type;

        // Get the goal of the query
        if (boost::iequals(goal_type, "goal") && in.good() && !in.eof())
        {
          std::string joint_constraint;
          in >> joint_constraint;
          if (joint_constraint == "joint_constraint")
            parseJointConstraint(in, psm, rs, goalState);
          if (joint_constraint == "position_constraint")
            parsePositionConstraint(in, psm, rs, goalState);
          if (joint_constraint == "link_constraint")
            parseLinkConstraint(in, psm, cs);
        }
        else
        {
          ROS_ERROR("Unknown goal type: '%s'", goal_type.c_str());
        }

        if (goalState.joint_constraints.size() ||
            (goalState.position_constraints.size() && goalState.orientation_constraints.size()))
        {
          // Save the query
          moveit_msgs::MotionPlanRequest planning_query;
          planning_query.start_state = startState;
          planning_query.goal_constraints = { goalState };
          pss->addPlanningQuery(planning_query, scene_name, query_name);
          ROS_INFO("Loaded query '%s' to scene '%s'", query_name.c_str(), scene_name.c_str());
        }
      }
    }
  }
  else
  {
    ROS_ERROR("The scene doesn't exist!");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "import_from_text_to_warehouse", ros::init_options::AnonymousName);

  boost::program_options::options_description desc;
  desc.add_options()("help", "Show help message")("queries", boost::program_options::value<std::string>(),
                                                  "Name of file containing motion planning queries.")(
      "scene", boost::program_options::value<std::string>(), "Name of file containing motion planning scene.")(
      "host", boost::program_options::value<std::string>(),
      "Host for the DB.")("port", boost::program_options::value<std::size_t>(), "Port for the DB.");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || argc == 1)  // show help if no parameters passed
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

  ros::NodeHandle nh;
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  if (!psm.getPlanningScene())
  {
    ROS_ERROR("Unable to initialize PlanningSceneMonitor");
    return 1;
  }

  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_warehouse::ConstraintsStorage cs(conn);
  moveit_warehouse::RobotStateStorage rs(conn);

  if (vm.count("scene"))
  {
    std::ifstream fin(vm["scene"].as<std::string>().c_str());
    psm.getPlanningScene()->loadGeometryFromStream(fin);
    fin.close();
    moveit_msgs::PlanningScene psmsg;
    psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
    pss.addPlanningScene(psmsg);
  }

  if (vm.count("queries"))
  {
    std::ifstream fin(vm["queries"].as<std::string>().c_str());
    if (fin.good() && !fin.eof())
    {
      parseQueries(fin, &psm, &rs, &cs, &pss);
    }
    fin.close();
  }

  return 0;
}
