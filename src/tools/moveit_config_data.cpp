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

/* Author: Dave Coleman */

// ******************************************************************************************
/* DEVELOPER NOTES

   This class is shared with all widgets and contains the common configuration data
   needed for generating each robot's MoveIt configuration package. All SRDF data is
   contained in a subclass of this class - srdf_writer.cpp. This class also contains
   the functions for writing out the configuration files. Maybe it would have been best to
   keep the writing out functions in configuration_files_widget.cpp, but I am not sure.
*/
// ******************************************************************************************

#include <moveit/setup_assistant/tools/moveit_config_data.h>
// Reading/Writing Files
#include <iostream> // For writing yaml and launch files
#include <fstream>
#include <yaml-cpp/yaml.h> // outputing yaml config files
#include <boost/filesystem.hpp>  // for creating folders/files
#include <boost/algorithm/string.hpp> // for string find and replace in templates
// ROS
#include <ros/console.h>
#include <ros/package.h> // for getting file path for loading images

namespace moveit_setup_assistant
{

// File system
namespace fs = boost::filesystem;

// ******************************************************************************************
// Constructor
// ******************************************************************************************
MoveItConfigData::MoveItConfigData() :
  config_pkg_generated_timestamp_(0)
{
  // Create an instance of SRDF writer and URDF model for all widgets to share
  srdf_.reset( new SRDFWriter() );
  urdf_model_.reset( new urdf::Model() );

  // Not in debug mode
  debug_ = false;

  // Get MoveIt Setup Assistant package path
  setup_assistant_path_ = ros::package::getPath("moveit_setup_assistant");
  if( setup_assistant_path_.empty() )
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
// Provide a kinematic model. Load a new one if necessary
// ******************************************************************************************
robot_model::RobotModelConstPtr MoveItConfigData::getRobotModel()
{
  if( !kin_model_ )
  {
    // Initialize with a URDF Model Interface and a SRDF Model
    kin_model_.reset( new robot_model::RobotModel( urdf_model_, srdf_->srdf_model_ ) );
    kin_model_const_ = kin_model_;
  }

  return kin_model_const_;
}

// ******************************************************************************************
// Update the Kinematic Model with latest SRDF modifications
// ******************************************************************************************
void MoveItConfigData::updateRobotModel()
{
  ROS_INFO( "Updating kinematic model");

  // Tell SRDF Writer to create new SRDF Model, use original URDF model
  srdf_->updateSRDFModel( *urdf_model_ );

  // Create new kin model
  kin_model_.reset( new robot_model::RobotModel( urdf_model_, srdf_->srdf_model_ ) );
  kin_model_const_ = kin_model_;

  // Reset the planning scene
  planning_scene_.reset();
}

// ******************************************************************************************
// Provide a shared planning scene
// ******************************************************************************************
planning_scene::PlanningScenePtr MoveItConfigData::getPlanningScene()
{
  if( !planning_scene_ )
  {
    // make sure kinematic model exists
    getRobotModel();

    // Allocate an empty planning scene
    planning_scene_.reset(new planning_scene::PlanningScene(kin_model_));
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
  for( std::vector<srdf::Model::DisabledCollision>::const_iterator pair_it = srdf_->disabled_collisions_.begin();
       pair_it != srdf_->disabled_collisions_.end(); ++pair_it )
  {
    allowed_collision_matrix_.setEntry( pair_it->link1_, pair_it->link2_, true );
  }
}

// ******************************************************************************************
// Output MoveIt Setup Assistant hidden settings file
// ******************************************************************************************
bool MoveItConfigData::outputSetupAssistantFile( const std::string& file_path )
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
  emitter << YAML::EndMap;

  /// SRDF Path Location
  emitter << YAML::Key << "SRDF";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "relative_path" << YAML::Value << srdf_pkg_relative_path_;
  emitter << YAML::EndMap;

  /// Package generation time
  emitter << YAML::Key << "CONFIG";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "generated_timestamp" << YAML::Value << std::time(NULL); // TODO: is this cross-platform?
  emitter << YAML::EndMap;

  emitter << YAML::EndMap;

  std::ofstream output_stream( file_path.c_str(), std::ios_base::trunc );
  if( !output_stream.good() )
  {
    ROS_ERROR_STREAM( "Unable to open file for writing " << file_path );
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true; // file created successfully
}

// ******************************************************************************************
// Output OMPL Planning config files
// ******************************************************************************************
bool MoveItConfigData::outputOMPLPlanningYAML( const std::string& file_path )
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Output every available planner ---------------------------------------------------
  emitter << YAML::Key << "planner_configs";

  emitter << YAML::Value << YAML::BeginMap;

  // Add Planner
  emitter << YAML::Key << "SBLkConfigDefault";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "geometric::SBL";
  emitter << YAML::EndMap;

  // Add Planner
  emitter << YAML::Key << "LBKPIECEkConfigDefault";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "geometric::LBKPIECE";
  emitter << YAML::EndMap;

  // Add Planner
  emitter << YAML::Key << "RRTkConfigDefault";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "geometric::RRT";
  emitter << YAML::EndMap;

  // Add Planner
  emitter << YAML::Key << "RRTConnectkConfigDefault";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "geometric::RRTConnect";
  emitter << YAML::EndMap;

  // Add Planner
  emitter << YAML::Key << "LazyRRTkConfigDefault";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "geometric::LazyRRT";
  emitter << YAML::EndMap;

  // Add Planner
  emitter << YAML::Key << "ESTkConfigDefault";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "geometric::EST";
  emitter << YAML::EndMap;

  // Add Planner
  emitter << YAML::Key << "KPIECEkConfigDefault";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "geometric::KPIECE";
  emitter << YAML::EndMap;

  // Add Planner
  emitter << YAML::Key << "RRTStarkConfigDefault";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "geometric::RRTstar";
  emitter << YAML::EndMap;

  // Add Planner
  emitter << YAML::Key << "BKPIECEkConfigDefault";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "geometric::BKPIECE";
  emitter << YAML::EndMap;

  // End of every avail planner
  emitter << YAML::EndMap;

  // Output every group and the planners it can use ----------------------------------
  for( std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin();
       group_it != srdf_->groups_.end();  ++group_it )
  {
    emitter << YAML::Key << group_it->name_;
    emitter << YAML::Value << YAML::BeginMap;
    // Output associated planners
    emitter << YAML::Key << "planner_configs";
    emitter << YAML::Value << YAML::BeginSeq;
    emitter << "SBLkConfigDefault" << "LBKPIECEkConfigDefault" << "RRTkConfigDefault"
            << "RRTConnectkConfigDefault" << "ESTkConfigDefault" << "KPIECEkConfigDefault"
            << "BKPIECEkConfigDefault" << "RRTStarkConfigDefault" << YAML::EndSeq;

    // Output projection_evaluator
    std::string projection_joints = decideProjectionJoints( group_it->name_ );
    if( !projection_joints.empty() )
    {
      emitter << YAML::Key << "projection_evaluator";
      emitter << YAML::Value << projection_joints;
      emitter << YAML::Key << "longest_valid_segment_fraction";
      emitter << YAML::Value << "0.05";
    }

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream( file_path.c_str(), std::ios_base::trunc );
  if( !output_stream.good() )
  {
    ROS_ERROR_STREAM( "Unable to open file for writing " << file_path );
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true; // file created successfully
}

// ******************************************************************************************
// Output kinematic config files
// ******************************************************************************************
bool MoveItConfigData::outputKinematicsYAML( const std::string& file_path )
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Output every group and the kinematic solver it can use ----------------------------------
  for( std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin();
       group_it != srdf_->groups_.end();  ++group_it )
  {
    // Only save kinematic data if the solver is not "None"
    if( group_meta_data_[ group_it->name_ ].kinematics_solver_.empty() ||
        group_meta_data_[ group_it->name_ ].kinematics_solver_ == "None" )
      continue;

    emitter << YAML::Key << group_it->name_;
    emitter << YAML::Value << YAML::BeginMap;

    // Kinematic Solver
    emitter << YAML::Key << "kinematics_solver";
    emitter << YAML::Value << group_meta_data_[ group_it->name_ ].kinematics_solver_;

    // Search Resolution
    emitter << YAML::Key << "kinematics_solver_search_resolution";
    emitter << YAML::Value << group_meta_data_[ group_it->name_ ].kinematics_solver_search_resolution_;

    // Solver Timeout
    emitter << YAML::Key << "kinematics_solver_timeout";
    emitter << YAML::Value << group_meta_data_[ group_it->name_ ].kinematics_solver_timeout_;

    // Solver Attempts
    emitter << YAML::Key << "kinematics_solver_attempts";
    emitter << YAML::Value << group_meta_data_[ group_it->name_ ].kinematics_solver_attempts_;

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream( file_path.c_str(), std::ios_base::trunc );
  if( !output_stream.good() )
  {
    ROS_ERROR_STREAM( "Unable to open file for writing " << file_path );
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true; // file created successfully
}

// ******************************************************************************************
// Output joint limits config files
// ******************************************************************************************
bool MoveItConfigData::outputJointLimitsYAML( const std::string& file_path )
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  emitter << YAML::Key << "joint_limits";
  emitter << YAML::Value << YAML::BeginMap;

  // Union all the joints in groups
  std::set<std::string> joints;

  // Loop through groups
  for( std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin();
       group_it != srdf_->groups_.end();  ++group_it )
  {
    // Get list of associated joints
    const robot_model::JointModelGroup *joint_model_group =
      getRobotModel()->getJointModelGroup( group_it->name_ );

    std::vector<const robot_model::JointModel*> joint_models = joint_model_group->getJointModels();

    // Iterate through the joints
    for( std::vector<const robot_model::JointModel*>::const_iterator joint_it = joint_models.begin();
         joint_it < joint_models.end(); ++joint_it )
    {
      // Check that this joint only represents 1 variable.
      if( (*joint_it)->getVariableCount() == 1 )
      {
        joints.insert( (*joint_it)->getName() );
      }
    }
  }

  // Add joints to yaml file, if no more than 1 dof
  for ( std::set<std::string>::iterator joint_it = joints.begin() ; joint_it != joints.end() ; ++joint_it )
  {
    emitter << YAML::Key << *joint_it;
    emitter << YAML::Value << YAML::BeginMap;

    // Output property
    emitter << YAML::Key << "has_velocity_limits";
    emitter << YAML::Value << "true";

    // Output property
    emitter << YAML::Key << "max_velocity";
    emitter << YAML::Value << "1.0";

    // Output property
    emitter << YAML::Key << "has_acceleration_limits";
    emitter << YAML::Value << "true";

    // Output property
    emitter << YAML::Key << "max_acceleration";
    emitter << YAML::Value << "1.0";

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream( file_path.c_str(), std::ios_base::trunc );
  if( !output_stream.good() )
  {
    ROS_ERROR_STREAM( "Unable to open file for writing " << file_path );
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true; // file created successfully
}

// ******************************************************************************************
// Decide the best two joints to be used for the projection evaluator
// ******************************************************************************************
std::string MoveItConfigData::decideProjectionJoints(std::string planning_group)
{
  std::string joint_pair = "";

  // Retrieve pointer to the shared kinematic model
  const robot_model::RobotModelConstPtr &model = getRobotModel();

  // Error check
  if( !model->hasJointModelGroup( planning_group ) )
    return joint_pair;

  // Get the joint model group
  const robot_model::JointModelGroup* group = model->getJointModelGroup(planning_group);

  // get vector of joint names
  const std::vector<std::string> joints = group->getJointModelNames();

  if( joints.size() >= 2 )
  {
    // Check that the first two joints have only 1 variable
    if( group->getJointModel( joints[0] )->getVariableCount() == 1 &&
        group->getJointModel( joints[1] )->getVariableCount() == 1)
    {
      // Just choose the first two joints.
      joint_pair = "joints("+joints[0] + "," + joints[1]+")";
    }
  }

  return joint_pair;
}

// ******************************************************************************************
// Input kinematics.yaml file
// ******************************************************************************************
bool MoveItConfigData::inputKinematicsYAML( const std::string& file_path )
{
  // Load file
  std::ifstream input_stream( file_path.c_str() );
  if( !input_stream.good() )
  {
    ROS_ERROR_STREAM( "Unable to open file for reading " << file_path );
    return false;
  }

  // Begin parsing
  try {
    YAML::Parser parser(input_stream);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    // Loop through all groups
    for( YAML::Iterator group_it = doc.begin(); group_it != doc.end(); ++group_it )
    {
      std::string group_name;
      group_it.first() >> group_name;

      // Create new meta data
      GroupMetaData new_meta_data;

      // kinematics_solver
      if( const YAML::Node *prop_name = group_it.second().FindValue( "kinematics_solver" ) )
      {
        *prop_name >> new_meta_data.kinematics_solver_;
      }

      // kinematics_solver_search_resolution
      if( const YAML::Node *prop_name = group_it.second().FindValue( "kinematics_solver_search_resolution" ) )
      {
        *prop_name >> new_meta_data.kinematics_solver_search_resolution_;
      }
      else
      {
        new_meta_data.kinematics_solver_attempts_ = DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION_;
      }

      // kinematics_solver_timeout
      if( const YAML::Node *prop_name = group_it.second().FindValue( "kinematics_solver_timeout" ) )
      {
        *prop_name >> new_meta_data.kinematics_solver_timeout_;
      }
      else
      {
        new_meta_data.kinematics_solver_attempts_ = DEFAULT_KIN_SOLVER_TIMEOUT_;
      }

      // kinematics_solver_attempts
      if( const YAML::Node *prop_name = group_it.second().FindValue( "kinematics_solver_attempts" ) )
      {
        *prop_name >> new_meta_data.kinematics_solver_attempts_;
      }
      else
      {
        new_meta_data.kinematics_solver_attempts_ = DEFAULT_KIN_SOLVER_ATTEMPTS_;
      }

      // Assign meta data to vector
      group_meta_data_[ group_name ] = new_meta_data;
    }

  }
  catch(YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM( e.what() );
    return false;
  }

  return true; // file created successfully
}

// ******************************************************************************************
// Input .setup_assistant file - contains data used for the MoveIt Setup Assistant
// ******************************************************************************************
bool MoveItConfigData::inputSetupAssistantYAML( const std::string& file_path )
{
  // Load file
  std::ifstream input_stream( file_path.c_str() );
  if( !input_stream.good() )
  {
    ROS_ERROR_STREAM( "Unable to open file for reading " << file_path );
    return false;
  }

  // Begin parsing
  try {
    YAML::Parser parser(input_stream);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    // Get title node
    if( const YAML::Node *title_node =doc.FindValue( "moveit_setup_assistant_config" ) )
    {
      // URDF Properties
      if( const YAML::Node *urdf_node = title_node->FindValue( "URDF" ) )
      {
        // Load first property
        if( const YAML::Node *package_node = urdf_node->FindValue( "package" ) )
        {
          *package_node >> urdf_pkg_name_;
        }
        else
        {
          return false; // if we do not find this value we cannot continue
        }

        // Load second property
        if( const YAML::Node *relative_node = urdf_node->FindValue( "relative_path" ) )
        {
          *relative_node >> urdf_pkg_relative_path_;
        }
        else
        {
          return false; // if we do not find this value we cannot continue
        }
      }
      // SRDF Properties
      if( const YAML::Node *srdf_node = title_node->FindValue( "SRDF" ) )
      {
        // Load first property
        if( const YAML::Node *relative_node = srdf_node->FindValue( "relative_path" ) )
        {
          *relative_node >> srdf_pkg_relative_path_;
        }
        else
        {
          return false; // if we do not find this value we cannot continue
        }
      }
      // Package generation time
      if( const YAML::Node *config_node = title_node->FindValue( "CONFIG" ) )
      {
        // Load first property
        if( const YAML::Node *timestamp_node = config_node->FindValue( "generated_timestamp" ) )
        {
          *timestamp_node >> config_pkg_generated_timestamp_;
        }
        else
        {
          // if we do not find this value it is fine, not required
        }
      }
      return true;
    }
  }
  catch(YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM( e.what() );
  }

  return false; // if it gets to this point an error has occured
}

// ******************************************************************************************
// Helper Function for joining a file path and a file name, or two file paths, etc, in a cross-platform way
// ******************************************************************************************
std::string MoveItConfigData::appendPaths( const std::string &path1, const std::string &path2 )
{
  fs::path result = path1;
  result /= path2;
  return result.make_preferred().native().c_str();
}

srdf::Model::Group* MoveItConfigData::findGroupByName( const std::string &name )
{
  // Find the group we are editing based on the goup name string
  srdf::Model::Group *searched_group = NULL; // used for holding our search results

  for( std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin();
       group_it != srdf_->groups_.end(); ++group_it )
  {
    if( group_it->name_ == name ) // string match
    {
      searched_group = &(*group_it);  // convert to pointer from iterator
      break; // we are done searching
    }
  }

  // Check if subgroup was found
  if( searched_group == NULL ) // not found
    ROS_FATAL_STREAM("An internal error has occured while searching for groups. Group '" << name << "' was not found in the SRDF.");

  return searched_group;
}



} // namespace
