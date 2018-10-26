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

/* Author: Dave Coleman */

#include <moveit/setup_assistant/tools/moveit_config_data.h>
// Reading/Writing Files
#include <iostream>  // For writing yaml and launch files
#include <fstream>
#include <boost/filesystem.hpp>        // for creating folders/files
#include <boost/algorithm/string.hpp>  // for string find and replace in templates

// ROS
#include <ros/console.h>
#include <ros/package.h>  // for getting file path for loading images

namespace moveit_setup_assistant
{
// File system
namespace fs = boost::filesystem;

// ******************************************************************************************
// Constructor
// ******************************************************************************************
MoveItConfigData::MoveItConfigData() : config_pkg_generated_timestamp_(0)
{
  // Create an instance of SRDF writer and URDF model for all widgets to share
  srdf_.reset(new srdf::SRDFWriter());
  urdf_model_.reset(new urdf::Model());

  // Not in debug mode
  debug_ = false;

  // Get MoveIt! Setup Assistant package path
  setup_assistant_path_ = ros::package::getPath("moveit_setup_assistant");
  if (setup_assistant_path_.empty())
  {
    setup_assistant_path_ = ".";
  }
}

// ******************************************************************************************
// Destructor
// ******************************************************************************************
MoveItConfigData::~MoveItConfigData()
{
}

// ******************************************************************************************
// Load a robot model
// ******************************************************************************************
void MoveItConfigData::setRobotModel(robot_model::RobotModelPtr robot_model)
{
  robot_model_ = robot_model;
}

// ******************************************************************************************
// Provide a kinematic model. Load a new one if necessary
// ******************************************************************************************
robot_model::RobotModelConstPtr MoveItConfigData::getRobotModel()
{
  if (!robot_model_)
  {
    // Initialize with a URDF Model Interface and a SRDF Model
    robot_model_.reset(new robot_model::RobotModel(urdf_model_, srdf_->srdf_model_));
  }

  return robot_model_;
}

// ******************************************************************************************
// Update the Kinematic Model with latest SRDF modifications
// ******************************************************************************************
void MoveItConfigData::updateRobotModel()
{
  ROS_INFO("Updating kinematic model");

  // Tell SRDF Writer to create new SRDF Model, use original URDF model
  srdf_->updateSRDFModel(*urdf_model_);

  // Create new kin model
  robot_model_.reset(new robot_model::RobotModel(urdf_model_, srdf_->srdf_model_));

  // Reset the planning scene
  planning_scene_.reset();
}

// ******************************************************************************************
// Provide a shared planning scene
// ******************************************************************************************
planning_scene::PlanningScenePtr MoveItConfigData::getPlanningScene()
{
  if (!planning_scene_)
  {
    // make sure kinematic model exists
    getRobotModel();

    // Allocate an empty planning scene
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  }
  return planning_scene_;
}

// ******************************************************************************************
// Load the allowed collision matrix from the SRDF's list of link pairs
// ******************************************************************************************
void MoveItConfigData::loadAllowedCollisionMatrix()
{
  // Clear the allowed collision matrix
  allowed_collision_matrix_.clear();

  // Update the allowed collision matrix, in case there has been a change
  for (std::vector<srdf::Model::DisabledCollision>::const_iterator pair_it = srdf_->disabled_collisions_.begin();
       pair_it != srdf_->disabled_collisions_.end(); ++pair_it)
  {
    allowed_collision_matrix_.setEntry(pair_it->link1_, pair_it->link2_, true);
  }
}

// ******************************************************************************************
// Output MoveIt! Setup Assistant hidden settings file
// ******************************************************************************************
bool MoveItConfigData::outputSetupAssistantFile(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Output every available planner ---------------------------------------------------
  emitter << YAML::Key << "moveit_setup_assistant_config";

  emitter << YAML::Value << YAML::BeginMap;

  // URDF Path Location
  emitter << YAML::Key << "URDF";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "package" << YAML::Value << urdf_pkg_name_;
  emitter << YAML::Key << "relative_path" << YAML::Value << urdf_pkg_relative_path_;
  emitter << YAML::Key << "xacro_args" << YAML::Value << xacro_args_;
  emitter << YAML::EndMap;

  /// SRDF Path Location
  emitter << YAML::Key << "SRDF";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "relative_path" << YAML::Value << srdf_pkg_relative_path_;
  emitter << YAML::EndMap;

  /// Package generation time
  emitter << YAML::Key << "CONFIG";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "author_name" << YAML::Value << author_name_;
  emitter << YAML::Key << "author_email" << YAML::Value << author_email_;
  emitter << YAML::Key << "generated_timestamp" << YAML::Value << std::time(NULL);  // TODO: is this cross-platform?
  emitter << YAML::EndMap;

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Output OMPL Planning config files
// ******************************************************************************************
bool MoveItConfigData::outputOMPLPlanningYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Output every available planner ---------------------------------------------------
  emitter << YAML::Key << "planner_configs";

  emitter << YAML::Value << YAML::BeginMap;

  std::vector<OMPLPlannerDescription> planner_des = getOMPLPlanners();

  // Add Planners with parameter values
  std::vector<std::string> pconfigs;
  for (std::size_t i = 0; i < planner_des.size(); ++i)
  {
    std::string defaultconfig = planner_des[i].name_;
    emitter << YAML::Key << defaultconfig;
    emitter << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "type" << YAML::Value << "geometric::" + planner_des[i].name_;
    for (std::size_t j = 0; j < planner_des[i].parameter_list_.size(); j++)
    {
      emitter << YAML::Key << planner_des[i].parameter_list_[j].name;
      emitter << YAML::Value << planner_des[i].parameter_list_[j].value;
      emitter << YAML::Comment(planner_des[i].parameter_list_[j].comment.c_str());
    }
    emitter << YAML::EndMap;

    pconfigs.push_back(defaultconfig);
  }

  // End of every avail planner
  emitter << YAML::EndMap;

  // Output every group and the planners it can use ----------------------------------
  for (std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin(); group_it != srdf_->groups_.end();
       ++group_it)
  {
    emitter << YAML::Key << group_it->name_;
    emitter << YAML::Value << YAML::BeginMap;
    // Output associated planners
    emitter << YAML::Key << "default_planner_config" << YAML::Value
            << group_meta_data_[group_it->name_].default_planner_;
    emitter << YAML::Key << "planner_configs";
    emitter << YAML::Value << YAML::BeginSeq;
    for (std::size_t i = 0; i < pconfigs.size(); ++i)
      emitter << pconfigs[i];
    emitter << YAML::EndSeq;

    // Output projection_evaluator
    std::string projection_joints = decideProjectionJoints(group_it->name_);
    if (!projection_joints.empty())
    {
      emitter << YAML::Key << "projection_evaluator";
      emitter << YAML::Value << projection_joints;
      // OMPL collision checking discretization
      emitter << YAML::Key << "longest_valid_segment_fraction";
      emitter << YAML::Value << "0.005";
    }

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Output CHOMP Planning config files
// ******************************************************************************************
bool MoveItConfigData::outputCHOMPPlanningYAML(const std::string& file_path)
{
  YAML::Emitter emitter;

  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "planning_time_limit" << YAML::Value << "10.0";
  emitter << YAML::Key << "max_iterations" << YAML::Value << "200";
  emitter << YAML::Key << "max_iterations_after_collision_free" << YAML::Value << "5";
  emitter << YAML::Key << "smoothness_cost_weight" << YAML::Value << "0.1";
  emitter << YAML::Key << "obstacle_cost_weight" << YAML::Value << "1.0";
  emitter << YAML::Key << "learning_rate" << YAML::Value << "0.01";
  emitter << YAML::Key << "smoothness_cost_velocity" << YAML::Value << "0.0";
  emitter << YAML::Key << "smoothness_cost_acceleration" << YAML::Value << "1.0";
  emitter << YAML::Key << "smoothness_cost_jerk" << YAML::Value << "0.0";
  emitter << YAML::Key << "ridge_factor" << YAML::Value << "0.01";
  emitter << YAML::Key << "use_pseudo_inverse" << YAML::Value << "false";
  emitter << YAML::Key << "pseudo_inverse_ridge_factor" << YAML::Value << "1e-4";
  emitter << YAML::Key << "joint_update_limit" << YAML::Value << "0.1";
  emitter << YAML::Key << "collision_clearence" << YAML::Value << "0.2";
  emitter << YAML::Key << "collision_threshold" << YAML::Value << "0.07";
  emitter << YAML::Key << "use_stochastic_descent" << YAML::Value << "true";
  emitter << YAML::Key << "enable_failure_recovery" << YAML::Value << "true";
  emitter << YAML::Key << "max_recovery_attempts" << YAML::Value << "5";
  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Output kinematic config files
// ******************************************************************************************
bool MoveItConfigData::outputKinematicsYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Output every group and the kinematic solver it can use ----------------------------------
  for (std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin(); group_it != srdf_->groups_.end();
       ++group_it)
  {
    // Only save kinematic data if the solver is not "None"
    if (group_meta_data_[group_it->name_].kinematics_solver_.empty() ||
        group_meta_data_[group_it->name_].kinematics_solver_ == "None")
      continue;

    emitter << YAML::Key << group_it->name_;
    emitter << YAML::Value << YAML::BeginMap;

    // Kinematic Solver
    emitter << YAML::Key << "kinematics_solver";
    emitter << YAML::Value << group_meta_data_[group_it->name_].kinematics_solver_;

    // Search Resolution
    emitter << YAML::Key << "kinematics_solver_search_resolution";
    emitter << YAML::Value << group_meta_data_[group_it->name_].kinematics_solver_search_resolution_;

    // Solver Timeout
    emitter << YAML::Key << "kinematics_solver_timeout";
    emitter << YAML::Value << group_meta_data_[group_it->name_].kinematics_solver_timeout_;

    // Solver Attempts
    emitter << YAML::Key << "kinematics_solver_attempts";
    emitter << YAML::Value << group_meta_data_[group_it->name_].kinematics_solver_attempts_;

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Writes a Gazebo compatible robot URDF to gazebo_compatible_urdf_string_
// ******************************************************************************************
std::string MoveItConfigData::getGazeboCompatibleURDF()
{
  bool new_urdf_needed = false;
  TiXmlDocument urdf_document;

  // Used to convert XmlDocument to std string
  TiXmlPrinter printer;
  urdf_document.Parse((const char*)urdf_string_.c_str(), 0, TIXML_ENCODING_UTF8);
  try
  {
    for (TiXmlElement* link_element = urdf_document.RootElement()->FirstChildElement(); link_element != NULL;
         link_element = link_element->NextSiblingElement())
    {
      if (((std::string)link_element->Value()).find("link") == std::string::npos)
        continue;
      // Before adding inertial elements, make sure there is none and the link has collision element
      if (link_element->FirstChildElement("inertial") == NULL && link_element->FirstChildElement("collision") != NULL)
      {
        new_urdf_needed = true;
        TiXmlElement inertia_link("inertial");
        TiXmlElement mass("mass");
        TiXmlElement inertia_joint("inertia");

        mass.SetAttribute("value", "0.1");

        inertia_joint.SetAttribute("ixx", "0.03");
        inertia_joint.SetAttribute("iyy", "0.03");
        inertia_joint.SetAttribute("izz", "0.03");
        inertia_joint.SetAttribute("ixy", "0.0");
        inertia_joint.SetAttribute("ixz", "0.0");
        inertia_joint.SetAttribute("iyz", "0.0");

        inertia_link.InsertEndChild(mass);
        inertia_link.InsertEndChild(inertia_joint);

        link_element->InsertEndChild(inertia_link);
      }
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM_NAMED("moveit_config_data", e.what());
    return std::string("");
  }

  if (new_urdf_needed)
  {
    urdf_document.Accept(&printer);
    return std::string(printer.CStr());
  }

  return std::string("");
}

bool MoveItConfigData::outputFakeControllersYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  emitter << YAML::Key << "controller_list";
  emitter << YAML::Value << YAML::BeginSeq;

  // Loop through groups
  for (std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin(); group_it != srdf_->groups_.end();
       ++group_it)
  {
    // Get list of associated joints
    const robot_model::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group_it->name_);
    emitter << YAML::BeginMap;
    const std::vector<const robot_model::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
    emitter << YAML::Key << "name";
    emitter << YAML::Value << "fake_" + group_it->name_ + "_controller";
    emitter << YAML::Key << "joints";
    emitter << YAML::Value << YAML::BeginSeq;

    // Iterate through the joints
    for (const robot_model::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != NULL || joint->getType() == robot_model::JointModel::FIXED)
        continue;
      emitter << joint->getName();
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;
  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

std::vector<OMPLPlannerDescription> MoveItConfigData::getOMPLPlanners()
{
  std::vector<OMPLPlannerDescription> planner_des;

  OMPLPlannerDescription SBL("SBL", "geometric");
  SBL.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  planner_des.push_back(SBL);

  OMPLPlannerDescription EST("EST", "geometric");
  EST.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0 setup()");
  EST.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  planner_des.push_back(EST);

  OMPLPlannerDescription LBKPIECE("LBKPIECE", "geometric");
  LBKPIECE.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                        "setup()");
  LBKPIECE.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9");
  LBKPIECE.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(LBKPIECE);

  OMPLPlannerDescription BKPIECE("BKPIECE", "geometric");
  BKPIECE.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                       "setup()");
  BKPIECE.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9");
  BKPIECE.addParameter("failed_expansion_score_factor", "0.5", "When extending motion fails, scale score by factor. "
                                                               "default: 0.5");
  BKPIECE.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(BKPIECE);

  OMPLPlannerDescription KPIECE("KPIECE", "geometric");
  KPIECE.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                      "setup()");
  KPIECE.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  KPIECE.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9 (0.0,1.]");
  KPIECE.addParameter("failed_expansion_score_factor", "0.5", "When extending motion fails, scale score by factor. "
                                                              "default: 0.5");
  KPIECE.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(KPIECE);

  OMPLPlannerDescription RRT("RRT", "geometric");
  RRT.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  RRT.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  planner_des.push_back(RRT);

  OMPLPlannerDescription RRTConnect("RRTConnect", "geometric");
  RRTConnect.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                          "setup()");
  planner_des.push_back(RRTConnect);

  OMPLPlannerDescription RRTstar("RRTstar", "geometric");
  RRTstar.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                       "setup()");
  RRTstar.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  RRTstar.addParameter("delay_collision_checking", "1", "Stop collision checking as soon as C-free parent found. "
                                                        "default 1");
  planner_des.push_back(RRTstar);

  OMPLPlannerDescription TRRT("TRRT", "geometric");
  TRRT.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  TRRT.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  TRRT.addParameter("max_states_failed", "10", "when to start increasing temp. default: 10");
  TRRT.addParameter("temp_change_factor", "2.0", "how much to increase or decrease temp. default: 2.0");
  TRRT.addParameter("min_temperature", "10e-10", "lower limit of temp change. default: 10e-10");
  TRRT.addParameter("init_temperature", "10e-6", "initial temperature. default: 10e-6");
  TRRT.addParameter("frountier_threshold", "0.0", "dist new state to nearest neighbor to disqualify as frontier. "
                                                  "default: 0.0 set in setup()");
  TRRT.addParameter("frountierNodeRatio", "0.1", "1/10, or 1 nonfrontier for every 10 frontier. default: 0.1");
  TRRT.addParameter("k_constant", "0.0", "value used to normalize expresssion. default: 0.0 set in setup()");
  planner_des.push_back(TRRT);

  OMPLPlannerDescription PRM("PRM", "geometric");
  PRM.addParameter("max_nearest_neighbors", "10", "use k nearest neighbors. default: 10");
  planner_des.push_back(PRM);

  OMPLPlannerDescription PRMstar("PRMstar", "geometric");  // no declares in code
  planner_des.push_back(PRMstar);

  OMPLPlannerDescription FMT("FMT", "geometric");
  FMT.addParameter("num_samples", "1000", "number of states that the planner should sample. default: 1000");
  FMT.addParameter("radius_multiplier", "1.1", "multiplier used for the nearest neighbors search radius. default: 1.1");
  FMT.addParameter("nearest_k", "1", "use Knearest strategy. default: 1");
  FMT.addParameter("cache_cc", "1", "use collision checking cache. default: 1");
  FMT.addParameter("heuristics", "0", "activate cost to go heuristics. default: 0");
  FMT.addParameter("extended_fmt", "1", "activate the extended FMT*: adding new samples if planner does not finish "
                                        "successfully. default: 1");
  planner_des.push_back(FMT);

  OMPLPlannerDescription BFMT("BFMT", "geometric");
  BFMT.addParameter("num_samples", "1000", "number of states that the planner should sample. default: 1000");
  BFMT.addParameter("radius_multiplier", "1.0", "multiplier used for the nearest neighbors search radius. default: "
                                                "1.0");
  BFMT.addParameter("nearest_k", "1", "use the Knearest strategy. default: 1");
  BFMT.addParameter("balanced", "0", "exploration strategy: balanced true expands one tree every iteration. False will "
                                     "select the tree with lowest maximum cost to go. default: 1");
  BFMT.addParameter("optimality", "1", "termination strategy: optimality true finishes when the best possible path is "
                                       "found. Otherwise, the algorithm will finish when the first feasible path is "
                                       "found. default: 1");
  BFMT.addParameter("heuristics", "1", "activates cost to go heuristics. default: 1");
  BFMT.addParameter("cache_cc", "1", "use the collision checking cache. default: 1");
  BFMT.addParameter("extended_fmt", "1", "Activates the extended FMT*: adding new samples if planner does not finish "
                                         "successfully. default: 1");
  planner_des.push_back(BFMT);

  OMPLPlannerDescription PDST("PDST", "geometric");
  RRT.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  planner_des.push_back(PDST);

  OMPLPlannerDescription STRIDE("STRIDE", "geometric");
  STRIDE.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                      "setup()");
  STRIDE.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  STRIDE.addParameter("use_projected_distance", "0", "whether nearest neighbors are computed based on distances in a "
                                                     "projection of the state rather distances in the state space "
                                                     "itself. default: 0");
  STRIDE.addParameter("degree", "16", "desired degree of a node in the Geometric Near-neightbor Access Tree (GNAT). "
                                      "default: 16");
  STRIDE.addParameter("max_degree", "18", "max degree of a node in the GNAT. default: 12");
  STRIDE.addParameter("min_degree", "12", "min degree of a node in the GNAT. default: 12");
  STRIDE.addParameter("max_pts_per_leaf", "6", "max points per leaf in the GNAT. default: 6");
  STRIDE.addParameter("estimated_dimension", "0.0", "estimated dimension of the free space. default: 0.0");
  STRIDE.addParameter("min_valid_path_fraction", "0.2", "Accept partially valid moves above fraction. default: 0.2");
  planner_des.push_back(STRIDE);

  OMPLPlannerDescription BiTRRT("BiTRRT", "geometric");
  BiTRRT.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                      "setup()");
  BiTRRT.addParameter("temp_change_factor", "0.1", "how much to increase or decrease temp. default: 0.1");
  BiTRRT.addParameter("init_temperature", "100", "initial temperature. default: 100");
  BiTRRT.addParameter("frountier_threshold", "0.0", "dist new state to nearest neighbor to disqualify as frontier. "
                                                    "default: 0.0 set in setup()");
  BiTRRT.addParameter("frountier_node_ratio", "0.1", "1/10, or 1 nonfrontier for every 10 frontier. default: 0.1");
  BiTRRT.addParameter("cost_threshold", "1e300", "the cost threshold. Any motion cost that is not better will not be "
                                                 "expanded. default: inf");
  planner_des.push_back(BiTRRT);

  OMPLPlannerDescription LBTRRT("LBTRRT", "geometric");
  LBTRRT.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                      "setup()");
  LBTRRT.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  LBTRRT.addParameter("epsilon", "0.4", "optimality approximation factor. default: 0.4");
  planner_des.push_back(LBTRRT);

  OMPLPlannerDescription BiEST("BiEST", "geometric");
  BiEST.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                     "setup()");
  planner_des.push_back(BiEST);

  OMPLPlannerDescription ProjEST("ProjEST", "geometric");
  ProjEST.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                       "setup()");
  ProjEST.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  planner_des.push_back(ProjEST);

  OMPLPlannerDescription LazyPRM("LazyPRM", "geometric");
  LazyPRM.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                                       "setup()");
  planner_des.push_back(LazyPRM);

  OMPLPlannerDescription LazyPRMstar("LazyPRMstar", "geometric");  // no declares in code
  planner_des.push_back(LazyPRMstar);

  OMPLPlannerDescription SPARS("SPARS", "geometric");
  SPARS.addParameter("stretch_factor", "3.0", "roadmap spanner stretch factor. multiplicative upper bound on path "
                                              "quality. It does not make sense to make this parameter more than 3. "
                                              "default: 3.0");
  SPARS.addParameter("sparse_delta_fraction", "0.25", "delta fraction for connection distance. This value represents "
                                                      "the visibility range of sparse samples. default: 0.25");
  SPARS.addParameter("dense_delta_fraction", "0.001", "delta fraction for interface detection. default: 0.001");
  SPARS.addParameter("max_failures", "1000", "maximum consecutive failure limit. default: 1000");
  planner_des.push_back(SPARS);

  OMPLPlannerDescription SPARStwo("SPARStwo", "geometric");
  SPARStwo.addParameter("stretch_factor", "3.0", "roadmap spanner stretch factor. multiplicative upper bound on path "
                                                 "quality. It does not make sense to make this parameter more than 3. "
                                                 "default: 3.0");
  SPARStwo.addParameter("sparse_delta_fraction", "0.25",
                        "delta fraction for connection distance. This value represents "
                        "the visibility range of sparse samples. default: 0.25");
  SPARStwo.addParameter("dense_delta_fraction", "0.001", "delta fraction for interface detection. default: 0.001");
  SPARStwo.addParameter("max_failures", "5000", "maximum consecutive failure limit. default: 5000");
  planner_des.push_back(SPARStwo);

  return planner_des;
}

// ******************************************************************************************
// Helper function to write the FollowJointTrajectory for each planning group to ros_controller.yaml,
// and erases the controller that have been written, to avoid mixing between FollowJointTrajectory
// which are published under the namespace of 'controller_list' and other types of controllers.
// ******************************************************************************************
void MoveItConfigData::outputFollowJointTrajectoryYAML(YAML::Emitter& emitter,
                                                       std::vector<ROSControlConfig>& ros_controllers_config_output)
{
  // Write default controllers
  emitter << YAML::Key << "controller_list";
  emitter << YAML::Value << YAML::BeginSeq;
  {
    for (std::vector<ROSControlConfig>::iterator controller_it = ros_controllers_config_output.begin();
         controller_it != ros_controllers_config_output.end();)
    {
      // Depending on the controller type, fill the required data
      if (controller_it->type_ == "FollowJointTrajectory")
      {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "name";
        emitter << YAML::Value << controller_it->name_;
        emitter << YAML::Key << "action_ns";
        emitter << YAML::Value << "follow_joint_trajectory";
        emitter << YAML::Key << "default";
        emitter << YAML::Value << "True";
        emitter << YAML::Key << "type";
        emitter << YAML::Value << controller_it->type_;
        // Write joints
        emitter << YAML::Key << "joints";
        {
          if (controller_it->joints_.size() != 1)
          {
            emitter << YAML::Value << YAML::BeginSeq;

            // Iterate through the joints
            for (std::vector<std::string>::iterator joint_it = controller_it->joints_.begin();
                 joint_it != controller_it->joints_.end(); ++joint_it)
            {
              emitter << *joint_it;
            }
            emitter << YAML::EndSeq;
          }
          else
          {
            emitter << YAML::Value << YAML::BeginMap;
            emitter << controller_it->joints_[0];
            emitter << YAML::EndMap;
          }
        }
        ros_controllers_config_output.erase(controller_it);
        emitter << YAML::EndMap;
      }
      else
      {
        controller_it++;
      }
    }
    emitter << YAML::EndSeq;
  }
}

// ******************************************************************************************
// Output controllers config files
// ******************************************************************************************
bool MoveItConfigData::outputROSControllersYAML(const std::string& file_path)
{
  // Copy ros_control_config_ to a new vector to avoid modifying it
  std::vector<ROSControlConfig> ros_controllers_config_output(ros_controllers_config_);

  // Cache the joints' names.
  std::vector<std::vector<std::string>> planning_groups;
  std::vector<std::string> group_joints;

  // We are going to write the joints names many times.
  // Loop through groups to store the joints names in group_joints vector and reuse is.
  for (std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin(); group_it != srdf_->groups_.end();
       ++group_it)
  {
    // Get list of associated joints
    const robot_model::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group_it->name_);
    const std::vector<const robot_model::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
    // Iterate through the joints and push into group_joints vector.
    for (const robot_model::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != NULL || joint->getType() == robot_model::JointModel::FIXED)
        continue;
      else
        group_joints.push_back(joint->getName());
    }
    // Push all the group joints into planning_groups vector.
    planning_groups.push_back(group_joints);
    group_joints.clear();
  }

  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Start with the robot name  ---------------------------------------------------
  emitter << YAML::Key << srdf_->srdf_model_->getName();
  emitter << YAML::Value << YAML::BeginMap;

  {
    emitter << YAML::Comment("MoveIt-specific simulation settings");
    emitter << YAML::Key << "moveit_sim_hw_interface" << YAML::Value << YAML::BeginMap;
    // Moveit Simulation Controller settings for setting initial pose
    {
      emitter << YAML::Key << "joint_model_group";
      emitter << YAML::Value << "controllers_initial_group_";
      emitter << YAML::Key << "joint_model_group_pose";
      emitter << YAML::Value << "controllers_initial_pose_";
      emitter << YAML::EndMap;
    }
    // Settings for ros_control control loop
    emitter << YAML::Newline;
    emitter << YAML::Comment("Settings for ros_control control loop");
    emitter << YAML::Key << "generic_hw_control_loop" << YAML::Value << YAML::BeginMap;
    {
      emitter << YAML::Key << "loop_hz";
      emitter << YAML::Value << "300";
      emitter << YAML::Key << "cycle_time_error_threshold";
      emitter << YAML::Value << "0.01";
      emitter << YAML::EndMap;
    }
    // Settings for ros_control hardware interface
    emitter << YAML::Newline;
    emitter << YAML::Comment("Settings for ros_control hardware interface");
    emitter << YAML::Key << "hardware_interface" << YAML::Value << YAML::BeginMap;
    {
      // Get list of all joints for the robot
      const std::vector<const robot_model::JointModel*>& joint_models = getRobotModel()->getJointModels();

      emitter << YAML::Key << "joints";
      {
        if (joint_models.size() != 1)
        {
          emitter << YAML::Value << YAML::BeginSeq;
          // Iterate through the joints
          for (std::vector<const robot_model::JointModel*>::const_iterator joint_it = joint_models.begin();
               joint_it < joint_models.end(); ++joint_it)
          {
            if ((*joint_it)->isPassive() || (*joint_it)->getMimic() != NULL ||
                (*joint_it)->getType() == robot_model::JointModel::FIXED)
              continue;
            else
              emitter << (*joint_it)->getName();
          }
          emitter << YAML::EndSeq;
        }
        else
        {
          emitter << YAML::Value << YAML::BeginMap;
          emitter << joint_models[0]->getName();
          emitter << YAML::EndMap;
        }
      }
      emitter << YAML::Key << "sim_control_mode";
      emitter << YAML::Value << "1";
      emitter << YAML::Comment("0: position, 1: velocity");
      emitter << YAML::Newline;
      emitter << YAML::EndMap;
    }
    // Joint State Controller
    emitter << YAML::Comment("Publish all joint states");
    emitter << YAML::Newline << YAML::Comment("Creates the /joint_states topic necessary in ROS");
    emitter << YAML::Key << "joint_state_controller" << YAML::Value << YAML::BeginMap;
    {
      emitter << YAML::Key << "type";
      emitter << YAML::Value << "joint_state_controller/JointStateController";
      emitter << YAML::Key << "publish_rate";
      emitter << YAML::Value << "50";
      emitter << YAML::EndMap;
    }

    // Writes Follow Joint Trajectory ROS controllers to ros_controller.yaml
    outputFollowJointTrajectoryYAML(emitter, ros_controllers_config_output);

    for (std::vector<ROSControlConfig>::const_iterator controller_it = ros_controllers_config_output.begin();
         controller_it != ros_controllers_config_output.end(); ++controller_it)
    {
      emitter << YAML::Key << controller_it->name_;
      emitter << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "type";
      emitter << YAML::Value << controller_it->type_;

      // Write joints
      emitter << YAML::Key << "joints";
      {
        if (controller_it->joints_.size() != 1)
        {
          emitter << YAML::Value << YAML::BeginSeq;

          // Iterate through the joints
          for (std::vector<std::string>::const_iterator joint_it = controller_it->joints_.begin();
               joint_it != controller_it->joints_.end(); ++joint_it)
          {
            emitter << *joint_it;
          }
          emitter << YAML::EndSeq;
        }
        else
        {
          emitter << YAML::Value << YAML::BeginMap;
          emitter << controller_it->joints_[0];
          emitter << YAML::EndMap;
        }
      }
      // Write gains as they are required for vel and effort controllers
      emitter << YAML::Key << "gains";
      emitter << YAML::Value << YAML::BeginMap;
      {
        // Iterate through the joints
        for (std::vector<std::string>::const_iterator joint_it = controller_it->joints_.begin();
             joint_it != controller_it->joints_.end(); ++joint_it)
        {
          emitter << YAML::Key << *joint_it << YAML::Value << YAML::BeginMap;
          emitter << YAML::Key << "p";
          emitter << YAML::Value << "100";
          emitter << YAML::Key << "d";
          emitter << YAML::Value << "1";
          emitter << YAML::Key << "i";
          emitter << YAML::Value << "1";
          emitter << YAML::Key << "i_clamp";
          emitter << YAML::Value << "1" << YAML::EndMap;
        }
        emitter << YAML::EndMap;
      }
      emitter << YAML::EndMap;
    }
  }
  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for writing " << file_path);
    return false;
  }
  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Output 3D Sensor configuration file
// ******************************************************************************************
bool MoveItConfigData::output3DSensorPluginYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  emitter << YAML::Comment("The name of this file shouldn't be changed, or else the Setup Assistant won't detect it");
  emitter << YAML::Key << "sensors";
  emitter << YAML::Value << YAML::BeginSeq;

  // Can we have more than one plugin config?
  emitter << YAML::BeginMap;

  // Make sure sensors_plugin_config_parameter_list_ is not empty
  if (!sensors_plugin_config_parameter_list_.empty())
  {
    for (auto& parameter : sensors_plugin_config_parameter_list_[0])
    {
      emitter << YAML::Key << parameter.first;
      emitter << YAML::Value << parameter.second.getValue();
    }
  }

  emitter << YAML::EndMap;

  emitter << YAML::EndSeq;

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Output joint limits config files
// ******************************************************************************************
bool MoveItConfigData::outputJointLimitsYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  emitter << YAML::Key << "joint_limits";
  emitter << YAML::Value << YAML::BeginMap;

  // Union all the joints in groups. Uses a custom comparator to allow the joints to be sorted by name
  std::set<const robot_model::JointModel*, joint_model_compare> joints;

  // Loop through groups
  for (std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin(); group_it != srdf_->groups_.end();
       ++group_it)
  {
    // Get list of associated joints
    const robot_model::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group_it->name_);

    const std::vector<const robot_model::JointModel*>& joint_models = joint_model_group->getJointModels();

    // Iterate through the joints
    for (std::vector<const robot_model::JointModel*>::const_iterator joint_it = joint_models.begin();
         joint_it != joint_models.end(); ++joint_it)
    {
      // Check that this joint only represents 1 variable.
      if ((*joint_it)->getVariableCount() == 1)
        joints.insert(*joint_it);
    }
  }

  // Add joints to yaml file, if no more than 1 dof
  for (std::set<const robot_model::JointModel*>::iterator joint_it = joints.begin(); joint_it != joints.end();
       ++joint_it)
  {
    emitter << YAML::Key << (*joint_it)->getName();
    emitter << YAML::Value << YAML::BeginMap;

    const robot_model::VariableBounds& b = (*joint_it)->getVariableBounds()[0];

    // Output property
    emitter << YAML::Key << "has_velocity_limits";
    if (b.velocity_bounded_)
      emitter << YAML::Value << "true";
    else
      emitter << YAML::Value << "false";

    // Output property
    emitter << YAML::Key << "max_velocity";
    emitter << YAML::Value << std::min(fabs(b.max_velocity_), fabs(b.min_velocity_));

    // Output property
    emitter << YAML::Key << "has_acceleration_limits";
    if (b.acceleration_bounded_)
      emitter << YAML::Value << "true";
    else
      emitter << YAML::Value << "false";

    // Output property
    emitter << YAML::Key << "max_acceleration";
    emitter << YAML::Value << std::min(fabs(b.max_acceleration_), fabs(b.min_acceleration_));

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for writing " << file_path);
    return false;
  }
  // Add documentation into joint_limits.yaml
  output_stream << "# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or "
                   "augmented as needed"
                << std::endl;
  output_stream << "# Specific joint properties can be changed with the keys [max_position, min_position, "
                   "max_velocity, max_acceleration]"
                << std::endl;
  output_stream << "# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]" << std::endl;
  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Set list of collision link pairs in SRDF; sorted; with optional filter
// ******************************************************************************************

class SortableDisabledCollision
{
public:
  SortableDisabledCollision(const srdf::Model::DisabledCollision& dc)
    : dc_(dc), key_(dc.link1_ < dc.link2_ ? (dc.link1_ + "|" + dc.link2_) : (dc.link2_ + "|" + dc.link1_))
  {
  }
  operator const srdf::Model::DisabledCollision() const
  {
    return dc_;
  }
  bool operator<(const SortableDisabledCollision& other) const
  {
    return key_ < other.key_;
  }

private:
  const srdf::Model::DisabledCollision dc_;
  const std::string key_;
};

void MoveItConfigData::setCollisionLinkPairs(const moveit_setup_assistant::LinkPairMap& link_pairs, size_t skip_mask)
{
  // Create temp disabled collision
  srdf::Model::DisabledCollision dc;

  std::set<SortableDisabledCollision> disabled_collisions;
  disabled_collisions.insert(srdf_->disabled_collisions_.begin(), srdf_->disabled_collisions_.end());

  // copy the data in this class's LinkPairMap datastructure to srdf::Model::DisabledCollision format
  for (moveit_setup_assistant::LinkPairMap::const_iterator pair_it = link_pairs.begin(); pair_it != link_pairs.end();
       ++pair_it)
  {
    // Only copy those that are actually disabled
    if (pair_it->second.disable_check)
    {
      if ((1 << pair_it->second.reason) & skip_mask)
        continue;

      dc.link1_ = pair_it->first.first;
      dc.link2_ = pair_it->first.second;
      dc.reason_ = moveit_setup_assistant::disabledReasonToString(pair_it->second.reason);

      disabled_collisions.insert(SortableDisabledCollision(dc));
    }
  }

  srdf_->disabled_collisions_.assign(disabled_collisions.begin(), disabled_collisions.end());
}

// ******************************************************************************************
// Decide the best two joints to be used for the projection evaluator
// ******************************************************************************************
std::string MoveItConfigData::decideProjectionJoints(std::string planning_group)
{
  std::string joint_pair = "";

  // Retrieve pointer to the shared kinematic model
  const robot_model::RobotModelConstPtr& model = getRobotModel();

  // Error check
  if (!model->hasJointModelGroup(planning_group))
    return joint_pair;

  // Get the joint model group
  const robot_model::JointModelGroup* group = model->getJointModelGroup(planning_group);

  // get vector of joint names
  const std::vector<std::string> joints = group->getJointModelNames();

  if (joints.size() >= 2)
  {
    // Check that the first two joints have only 1 variable
    if (group->getJointModel(joints[0])->getVariableCount() == 1 &&
        group->getJointModel(joints[1])->getVariableCount() == 1)
    {
      // Just choose the first two joints.
      joint_pair = "joints(" + joints[0] + "," + joints[1] + ")";
    }
  }

  return joint_pair;
}

template <typename T>
bool parse(const YAML::Node& node, const std::string& key, T& storage, const T& default_value = T())
{
  const YAML::Node& n = node[key];
  bool valid = n;
  storage = valid ? n.as<T>() : default_value;
  return valid;
}

bool MoveItConfigData::inputOMPLYAML(const std::string& file_path)
{
  // Load file
  std::ifstream input_stream(file_path.c_str());
  if (!input_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for reading " << file_path);
    return false;
  }

  // Begin parsing
  try
  {
    YAML::Node doc = YAML::Load(input_stream);

    // Loop through all groups
    for (YAML::const_iterator group_it = doc.begin(); group_it != doc.end(); ++group_it)
    {
      // get group name
      const std::string group_name = group_it->first.as<std::string>();

      // compare group name found to list of groups in group_meta_data_
      std::map<std::string, GroupMetaData>::iterator group_meta_it;
      group_meta_it = group_meta_data_.find(group_name);
      if (group_meta_it != group_meta_data_.end())
      {
        std::string planner;
        parse(group_it->second, "default_planner_config", planner);
        std::size_t pos = planner.find("kConfigDefault");
        if (pos != std::string::npos)
        {
          planner = planner.substr(0, pos);
        }
        group_meta_data_[group_name].default_planner_ = planner;
      }
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  return true;
}

// ******************************************************************************************
// Input kinematics.yaml file
// ******************************************************************************************
bool MoveItConfigData::inputKinematicsYAML(const std::string& file_path)
{
  // Load file
  std::ifstream input_stream(file_path.c_str());
  if (!input_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for reading " << file_path);
    return false;
  }

  // Begin parsing
  try
  {
    YAML::Node doc = YAML::Load(input_stream);

    // Loop through all groups
    for (YAML::const_iterator group_it = doc.begin(); group_it != doc.end(); ++group_it)
    {
      const std::string& group_name = group_it->first.as<std::string>();
      const YAML::Node& group = group_it->second;

      // Create new meta data
      GroupMetaData meta_data;

      parse(group, "kinematics_solver", meta_data.kinematics_solver_);
      parse(group, "kinematics_solver_search_resolution", meta_data.kinematics_solver_search_resolution_,
            DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION_);
      parse(group, "kinematics_solver_timeout", meta_data.kinematics_solver_timeout_, DEFAULT_KIN_SOLVER_TIMEOUT_);
      parse(group, "kinematics_solver_attempts", meta_data.kinematics_solver_attempts_, DEFAULT_KIN_SOLVER_ATTEMPTS_);

      // Assign meta data to vector
      group_meta_data_[group_name] = meta_data;
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  return true;  // file created successfully
}

// ******************************************************************************************
// Helper function for parsing an individual ROSController from ros_controllers yaml file
// ******************************************************************************************
bool MoveItConfigData::parseROSController(const YAML::Node& controller)
{
  // Used in parsing ROS controllers
  ROSControlConfig control_setting;

  if (const YAML::Node& trajectory_controllers = controller)
  {
    for (std::size_t trajectory_id = 0; trajectory_id < trajectory_controllers.size(); ++trajectory_id)
    {
      // Controller node
      if (const YAML::Node& controller_node = trajectory_controllers[trajectory_id])
      {
        if (const YAML::Node& joints = controller_node["joints"])
        {
          control_setting.joints_.clear();
          for (YAML::const_iterator joint_it = joints.begin(); joint_it != joints.end(); ++joint_it)
          {
            control_setting.joints_.push_back(joint_it->as<std::string>());
          }
          if (!parse(controller_node, "name", control_setting.name_))
          {
            ROS_ERROR_STREAM_NAMED("ros_controller.yaml", "Couldn't parse ros_controllers.yaml");
            return false;
          }
          if (!parse(controller_node, "type", control_setting.type_))
          {
            ROS_ERROR_STREAM_NAMED("ros_controller.yaml", "Couldn't parse ros_controllers.yaml");
            return false;
          }
          // All required fields were parsed correctly
          ros_controllers_config_.push_back(control_setting);
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("ros_controller.yaml", "Couldn't parse ros_controllers.yaml");
          return false;
        }
      }
    }
  }
  return true;
}

// ******************************************************************************************
// Helper function for parsing ROSControllers from ros_controllers yaml file
// ******************************************************************************************
bool MoveItConfigData::processROSControllers(const YAML::Node& controllers)
{
  // Used in parsing ROS controllers
  ROSControlConfig control_setting;

  // Loop through all controllers
  for (YAML::const_iterator controller_it = controllers.begin(); controller_it != controllers.end(); ++controller_it)
  {
    // Follow Joint Trajectory action controllers
    if (controller_it->first.as<std::string>() == "controller_list")
    {
      if (!parseROSController(controller_it->second))
        return false;
    }
    // Other settings found in the ros_controllers file
    else
    {
      const std::string& controller_name = controller_it->first.as<std::string>();
      control_setting.joints_.clear();

      // Push joints if found in the controller
      if (const YAML::Node& joints = controller_it->second["joints"])
      {
        if (joints.IsSequence())
        {
          for (YAML::const_iterator joint_it = joints.begin(); joint_it != joints.end(); ++joint_it)
          {
            control_setting.joints_.push_back(joint_it->as<std::string>());
          }
        }
        else
        {
          control_setting.joints_.push_back(joints.as<std::string>());
        }
      }

      // If the setting has joints then it is a controller that needs to be parsed
      if (!control_setting.joints_.empty())
      {
        if (const YAML::Node& urdf_node = controller_it->second["type"])
        {
          control_setting.type_ = controller_it->second["type"].as<std::string>();
          control_setting.name_ = controller_name;
          ros_controllers_config_.push_back(control_setting);
          control_setting.joints_.clear();
        }
      }
    }
  }
  return true;
}

// ******************************************************************************************
// Input ros_controllers.yaml file
// ******************************************************************************************
bool MoveItConfigData::inputROSControllersYAML(const std::string& file_path)
{
  // Load file
  std::ifstream input_stream(file_path.c_str());
  if (!input_stream.good())
  {
    ROS_WARN_STREAM_NAMED("ros_controller.yaml", "Does not exist " << file_path);
    return false;
  }

  // Begin parsing
  try
  {
    YAML::Node doc = YAML::Load(input_stream);

    for (YAML::const_iterator doc_map_it = doc.begin(); doc_map_it != doc.end(); ++doc_map_it)
    {
      if (const YAML::Node& controllers = doc_map_it->second)
        processROSControllers(controllers);
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  return true;  // file created successfully
}

// ******************************************************************************************
// Add a Follow Joint Trajectory action Controller for each Planning Group
// ******************************************************************************************
bool MoveItConfigData::addDefaultControllers()
{
  if (srdf_->srdf_model_->getGroups().size() == 0)
    return false;
  // Loop through groups
  for (std::vector<srdf::Model::Group>::const_iterator group_it = srdf_->srdf_model_->getGroups().begin();
       group_it != srdf_->srdf_model_->getGroups().end(); ++group_it)
  {
    ROSControlConfig group_controller;
    // Get list of associated joints
    const robot_model::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group_it->name_);
    const std::vector<const robot_model::JointModel*>& joint_models = joint_model_group->getActiveJointModels();

    // Iterate through the joints
    for (const robot_model::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != NULL || joint->getType() == robot_model::JointModel::FIXED)
        continue;
      group_controller.joints_.push_back(joint->getName());
    }
    if (!group_controller.joints_.empty())
    {
      group_controller.name_ = group_it->name_ + "_controller";
      group_controller.type_ = "FollowJointTrajectory";
      addROSController(group_controller);
    }
  }
  return true;
}

// ******************************************************************************************
// Set package path; try to resolve path from package name if directory does not exist
// ******************************************************************************************
bool MoveItConfigData::setPackagePath(const std::string& pkg_path)
{
  std::string full_package_path;

  // check that the folder exists
  if (!fs::is_directory(pkg_path))
  {
    // does not exist, check if its a package
    full_package_path = ros::package::getPath(pkg_path);

    // check that the folder exists
    if (!fs::is_directory(full_package_path))
    {
      return false;
    }
  }
  else
  {
    // they inputted a full path
    full_package_path = pkg_path;
  }

  config_pkg_path_ = full_package_path;
  return true;
}

// ******************************************************************************************
// Resolve path to .setup_assistant file
// ******************************************************************************************

bool MoveItConfigData::getSetupAssistantYAMLPath(std::string& path)
{
  path = appendPaths(config_pkg_path_, ".setup_assistant");

  // Check if the old package is a setup assistant package
  return fs::is_regular_file(path);
}

// ******************************************************************************************
// Make the full URDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool MoveItConfigData::createFullURDFPath()
{
  boost::trim(urdf_pkg_name_);

  // Check if a package name was provided
  if (urdf_pkg_name_.empty() || urdf_pkg_name_ == "\"\"")
  {
    urdf_path_ = urdf_pkg_relative_path_;
    urdf_pkg_name_.clear();
  }
  else
  {
    // Check that ROS can find the package
    std::string robot_desc_pkg_path = ros::package::getPath(urdf_pkg_name_);

    if (robot_desc_pkg_path.empty())
    {
      urdf_path_.clear();
      return false;
    }

    // Append the relative URDF url path
    urdf_path_ = appendPaths(robot_desc_pkg_path, urdf_pkg_relative_path_);
  }

  // Check that this file exits -------------------------------------------------
  return fs::is_regular_file(urdf_path_);
}

// ******************************************************************************************
// Make the full SRDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool MoveItConfigData::createFullSRDFPath(const std::string& package_path)
{
  srdf_path_ = appendPaths(package_path, srdf_pkg_relative_path_);

  return fs::is_regular_file(srdf_path_);
}

// ******************************************************************************************
// Input .setup_assistant file - contains data used for the MoveIt! Setup Assistant
// ******************************************************************************************
bool MoveItConfigData::inputSetupAssistantYAML(const std::string& file_path)
{
  // Load file
  std::ifstream input_stream(file_path.c_str());
  if (!input_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for reading " << file_path);
    return false;
  }

  // Begin parsing
  try
  {
    const YAML::Node& doc = YAML::Load(input_stream);

    // Get title node
    if (const YAML::Node& title_node = doc["moveit_setup_assistant_config"])
    {
      // URDF Properties
      if (const YAML::Node& urdf_node = title_node["URDF"])
      {
        if (!parse(urdf_node, "package", urdf_pkg_name_))
          return false;  // if we do not find this value we cannot continue

        if (!parse(urdf_node, "relative_path", urdf_pkg_relative_path_))
          return false;  // if we do not find this value we cannot continue

        parse(urdf_node, "xacro_args", xacro_args_);
      }
      // SRDF Properties
      if (const YAML::Node& srdf_node = title_node["SRDF"])
      {
        if (!parse(srdf_node, "relative_path", srdf_pkg_relative_path_))
          return false;  // if we do not find this value we cannot continue
      }
      // Package generation time
      if (const YAML::Node& config_node = title_node["CONFIG"])
      {
        parse(config_node, "author_name", author_name_);
        parse(config_node, "author_email", author_email_);
        parse(config_node, "generated_timestamp", config_pkg_generated_timestamp_);
      }
      return true;
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM(e.what());
  }

  return false;  // if it gets to this point an error has occured
}

// ******************************************************************************************
// Input sensors_3d yaml file
// ******************************************************************************************
bool MoveItConfigData::input3DSensorsYAML(const std::string& default_file_path, const std::string& file_path)
{
  // Load default parameters file
  std::ifstream default_input_stream(default_file_path.c_str());
  if (!default_input_stream.good())
  {
    ROS_ERROR_STREAM_NAMED("sensors_3d.yaml", "Unable to open file for reading " << default_file_path);
    return false;
  }

  // Parse default parameters values
  try
  {
    const YAML::Node& doc = YAML::Load(default_input_stream);

    // Get sensors node
    if (const YAML::Node& sensors_node = doc["sensors"])
    {
      // Make sue that the sensors are written as a sequence
      if (sensors_node.IsSequence())
      {
        GenericParameter sensor_param;
        std::map<std::string, GenericParameter> sensor_map;

        // Loop over the sensors available in the file
        for (std::size_t i = 0; i < sensors_node.size(); ++i)
        {
          if (const YAML::Node& sensor_node = sensors_node[i])
          {
            for (YAML::const_iterator sensor_it = sensor_node.begin(); sensor_it != sensor_node.end(); ++sensor_it)
            {
              sensor_param.setName(sensor_it->first.as<std::string>());
              sensor_param.setValue(sensor_it->second.as<std::string>());

              // Set the key as the parameter name to make accessing it easier
              sensor_map[sensor_it->first.as<std::string>()] = sensor_param;
            }
            sensors_plugin_config_parameter_list_.push_back(sensor_map);
          }
        }
      }
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM("Error parsing default sensors yaml: " << e.what());
  }

  // Is there a sensors config in the package?
  if (file_path.empty())
  {
    return true;
  }

  // Load file
  std::ifstream input_stream(file_path.c_str());
  if (!input_stream.good())
  {
    ROS_ERROR_STREAM_NAMED("sensors_3d.yaml", "Unable to open file for reading " << file_path);
    return false;
  }

  // Begin parsing
  try
  {
    const YAML::Node& doc = YAML::Load(input_stream);

    // Get sensors node
    const YAML::Node& sensors_node = doc["sensors"];

    // Make sure that the sensors are written as a sequence
    if (sensors_node && sensors_node.IsSequence())
    {
      GenericParameter sensor_param;
      std::map<std::string, GenericParameter> sensor_map;
      bool empty_node = true;

      // Loop over the sensors available in the file
      for (std::size_t i = 0; i < sensors_node.size(); ++i)
      {
        if (const YAML::Node& sensor_node = sensors_node[i])
        {
          for (YAML::const_iterator sensor_it = sensor_node.begin(); sensor_it != sensor_node.end(); ++sensor_it)
          {
            empty_node = false;
            sensor_param.setName(sensor_it->first.as<std::string>());
            sensor_param.setValue(sensor_it->second.as<std::string>());

            // Set the key as the parameter name to make accessing it easier
            sensor_map[sensor_it->first.as<std::string>()] = sensor_param;
          }
          // Don't push empty nodes
          if (!empty_node)
            sensors_plugin_config_parameter_list_.push_back(sensor_map);
        }
      }
    }
    return true;
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM("Error parsing sensors yaml: " << e.what());
  }

  return false;  // if it gets to this point an error has occured
}

// ******************************************************************************************
// Helper Function for joining a file path and a file name, or two file paths, etc, in a cross-platform way
// ******************************************************************************************
std::string MoveItConfigData::appendPaths(const std::string& path1, const std::string& path2)
{
  fs::path result = path1;
  result /= path2;
  return result.make_preferred().native().c_str();
}

srdf::Model::Group* MoveItConfigData::findGroupByName(const std::string& name)
{
  // Find the group we are editing based on the goup name string
  srdf::Model::Group* searched_group = NULL;  // used for holding our search results

  for (std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin(); group_it != srdf_->groups_.end();
       ++group_it)
  {
    if (group_it->name_ == name)  // string match
    {
      searched_group = &(*group_it);  // convert to pointer from iterator
      break;                          // we are done searching
    }
  }

  // Check if subgroup was found
  if (searched_group == NULL)  // not found
    ROS_FATAL_STREAM("An internal error has occured while searching for groups. Group '" << name << "' was not found "
                                                                                                    "in the SRDF.");

  return searched_group;
}

// ******************************************************************************************
// Find ROS controller by name
// ******************************************************************************************
ROSControlConfig* MoveItConfigData::findROSControllerByName(const std::string& controller_name)
{
  // Find the ROSController we are editing based on the ROSController name string
  ROSControlConfig* searched_ros_controller = NULL;  // used for holding our search results

  for (std::vector<ROSControlConfig>::iterator controller_it = ros_controllers_config_.begin();
       controller_it != ros_controllers_config_.end(); ++controller_it)
  {
    if (controller_it->name_ == controller_name)  // string match
    {
      searched_ros_controller = &(*controller_it);  // convert to pointer from iterator
      break;                                        // we are done searching
    }
  }

  return searched_ros_controller;
}

// ******************************************************************************************
// Deletes a ROS controller by name
// ******************************************************************************************
bool MoveItConfigData::deleteROSController(const std::string& controller_name)
{
  for (std::vector<ROSControlConfig>::iterator controller_it = ros_controllers_config_.begin();
       controller_it != ros_controllers_config_.end(); ++controller_it)
  {
    if (controller_it->name_ == controller_name)  // string match
    {
      ros_controllers_config_.erase(controller_it);
      // we are done searching
      return true;
    }
  }
  return false;
}

// ******************************************************************************************
// Adds a ROS controller to ros_controllers_config_ vector
// ******************************************************************************************
bool MoveItConfigData::addROSController(const ROSControlConfig& new_controller)
{
  // Used for holding our search results
  ROSControlConfig* searched_ros_controller = NULL;

  // Find if there is an existing controller with the same name
  searched_ros_controller = findROSControllerByName(new_controller.name_);

  if (searched_ros_controller && searched_ros_controller->type_ == new_controller.type_)
    return false;

  ros_controllers_config_.push_back(new_controller);
  return true;
}

// ******************************************************************************************
// Gets ros_controllers_config_ vector
// ******************************************************************************************
std::vector<ROSControlConfig>& MoveItConfigData::getROSControllers()
{
  return ros_controllers_config_;
}

// ******************************************************************************************
// Used to add a sensor plugin configuation parameter to the sensor plugin configuration parameter list
// ******************************************************************************************
void MoveItConfigData::addGenericParameterToSensorPluginConfig(const std::string& name, const std::string& value,
                                                               const std::string& comment)
{
  // Use index 0 since we only write one plugin
  GenericParameter new_parameter;
  new_parameter.setName(name);
  new_parameter.setValue(value);
  sensors_plugin_config_parameter_list_[0][name] = new_parameter;
}

// ******************************************************************************************
// Used to get sensor plugin configuration parameter list
// ******************************************************************************************
std::vector<std::map<std::string, GenericParameter>> MoveItConfigData::getSensorPluginConfig()
{
  return sensors_plugin_config_parameter_list_;
}

// ******************************************************************************************
// Used to clear sensor plugin configuration parameter list
// ******************************************************************************************
void MoveItConfigData::clearSensorPluginConfig()
{
  for (size_t param_id = 0; param_id < sensors_plugin_config_parameter_list_.size(); ++param_id)
  {
    sensors_plugin_config_parameter_list_[param_id].clear();
  }
}

}  // namespace
