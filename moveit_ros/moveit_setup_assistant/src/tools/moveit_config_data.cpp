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

#include "moveit_setup_assistant/tools/moveit_config_data.h"
// Reading/Writing Files
#include <iostream> // For writing yaml and launch files
#include <fstream>
#include <yaml-cpp/yaml.h> // outputing yaml config files
#include <boost/filesystem.hpp>  // for creating folders/files
#include <boost/algorithm/string.hpp> // for string find and replace in templates
// ROS
#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loading images

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Constructor
// ******************************************************************************************
MoveItConfigData::MoveItConfigData()
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
    // use cout b/c we don't know ROS's status...
    std::cout << "Unable to get MoveIt Setup Assistant package path " << std::endl;
    exit(0);
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
planning_models::KinematicModelConstPtr MoveItConfigData::getKinematicModel()
{
  if( !kin_model_ )
  {
    // Initialize with a URDF Model Interface and a SRDF Model
    kin_model_.reset( new planning_models::KinematicModel( urdf_model_, srdf_->srdf_model_ ) );                                                            
  }
  
  return kin_model_;
}

// ******************************************************************************************
// Update the Kinematic Model with latest SRDF modifications
// ******************************************************************************************
void MoveItConfigData::updateKinematicModel()
{
  ROS_INFO( "Updating kinematic model");

  // Tell SRDF Writer to create new SRDF Model, use original URDF model
  srdf_->updateSRDFModel( *urdf_model_ );

  // Create new kin model
  kin_model_.reset( new planning_models::KinematicModel( urdf_model_, srdf_->srdf_model_ ) );                                                            
}

// ******************************************************************************************
// Provide a shared planning scene
// ******************************************************************************************
planning_scene::PlanningScenePtr MoveItConfigData::getPlanningScene()
{
  if( !planning_scene_ )
  {
    // Allocate an empty planning scene
    planning_scene_.reset(new planning_scene::PlanningScene( ));

    // Configure planning scene
    planning_scene_->configure( urdf_model_, srdf_->srdf_model_ );
  }
  return planning_scene_;
}

// ******************************************************************************************
// Output basic package files
// ******************************************************************************************
bool MoveItConfigData::outputPackageFiles( const std::string& template_package_path,
                                           const std::string& new_package_path,
                                           const std::string& new_package_name  )
{
  // Copy CMakeLists.txt
  std::string template_path = template_package_path + "CMakeLists.txt";
  std::string file_path = new_package_path + "CMakeLists.txt";

  // Use generic template copy function
  if( !copyTemplate( template_path, file_path, new_package_name ) )
    return false;

  // Copy Makefile
  template_path = template_package_path + "Makefile";
  file_path = new_package_path + "Makefile";

  // Use generic template copy function
  if( !copyTemplate( template_path, file_path, new_package_name ) )
    return false;

  // Copy manifest.xml
  template_path = template_package_path + "manifest.xml";
  file_path = new_package_path + "manifest.xml";

  // Use generic template copy function
  if( !copyTemplate( template_path, file_path, new_package_name ) )
    return false;

  return true; // success
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
  /*emitter << YAML::Key << "SRDF";
    emitter << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "path" << YAML::Value << srdf_path_;
    emitter << YAML::EndMap; */
  
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
  emitter << YAML::Key << "SBLkConfig1";
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

  namespace pm = planning_models;

  // Loop through groups
  for( std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin(); 
       group_it != srdf_->groups_.end();  ++group_it )
  {  
    // Get list of associated joints  
    const pm::KinematicModel::JointModelGroup *joint_model_group = 
      getKinematicModel()->getJointModelGroup( group_it->name_ );

    std::vector<const pm::KinematicModel::JointModel*> joint_models = joint_model_group->getJointModels();
  
    // Iterate through the joints
    for( std::vector<const pm::KinematicModel::JointModel*>::const_iterator joint_it = joint_models.begin();
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
// Output benchmark server launch file
// ******************************************************************************************
bool MoveItConfigData::outputBenchmarkServerLaunch( const std::string& file_path )
{
  // TODO: identify what goes in this launch file

  return true; // file created successfully
}

// ******************************************************************************************
// Output move group launch file
// ******************************************************************************************
bool MoveItConfigData::outputMoveGroupLaunch( const std::string& file_path,
                                              const std::string& template_package_path,
                                              const std::string& new_package_name  )
{
  // Path
  const std::string template_path = template_package_path + "launch/move_group.launch";

  // Use generic template copy function
  return copyTemplate( template_path, file_path, new_package_name );
}

// ******************************************************************************************
// Output OMPL Planner launch file
// ******************************************************************************************
bool MoveItConfigData::outputOMPLPlannerLaunch( const std::string& file_path,
                                                const std::string& template_package_path,
                                                const std::string& new_package_name  )
{
  // File system
  namespace fs = boost::filesystem;

  // Path
  const std::string template_path = template_package_path + "launch/ompl_planner.launch";

  // Only generate this file if it does not exist. This allows user to customize the urdf launch part
  if( fs::is_regular_file( file_path ) )
  {
    return true; // do nothing
  }  

  // Use generic template copy function
  return copyTemplate( template_path, file_path, new_package_name );
}

// ******************************************************************************************
// Output planning context launch file
// ******************************************************************************************
bool MoveItConfigData::outputPlanningContextLaunch( const std::string& file_path,
                                                    const std::string& template_package_path,
                                                    const std::string& new_package_name )
{
  // File system
  namespace fs = boost::filesystem;

  // Path
  const std::string template_path = template_package_path + "launch/planning_context.launch";

  // Use generic template copy function
  return copyTemplate( template_path, file_path, new_package_name );
}

// ******************************************************************************************
// Output warehouse launch file
// ******************************************************************************************
bool MoveItConfigData::outputWarehouseLaunch( const std::string& file_path )
{
  // TODO: identify what goes in this launch file

  return true; // file created successfully
}

// ******************************************************************************************
// Output warehouse settings launch file
// ******************************************************************************************
bool MoveItConfigData::outputWarehouseSettingsLaunch( const std::string& file_path )
{
  // TODO: identify what goes in this launch file

  return true; // file created successfully
}

// ******************************************************************************************
// Copy a template from location <template_path> to location <output_path> and replace package name
// ******************************************************************************************
bool MoveItConfigData::copyTemplate( const std::string& template_path, const std::string& output_path,
                                     const std::string& new_package_name  )
{
  // File system
  namespace fs = boost::filesystem;

  // Error check file
  if( ! fs::is_regular_file( template_path ) )
  {
    ROS_ERROR_STREAM( "Unable to find template file " << template_path );
    return false;
  }
  
  // Load file
  std::ifstream template_stream( template_path.c_str() );
  if( !template_stream.good() ) // File not found
  {
    ROS_ERROR_STREAM( "Unable to load file " << template_path );
    return false;
  }

  // Load the file to a string using an efficient memory allocation technique
  std::string template_string;
  template_stream.seekg(0, std::ios::end);   
  template_string.reserve(template_stream.tellg());
  template_stream.seekg(0, std::ios::beg);
  template_string.assign( (std::istreambuf_iterator<char>(template_stream)), std::istreambuf_iterator<char>() );  
  template_stream.close();

  // Replace keywords in string ------------------------------------------------------------
  boost::replace_all( template_string, "[GENERATED_PACKAGE_NAME]", new_package_name );

  // Save string to new location -----------------------------------------------------------
  std::ofstream output_stream( output_path.c_str(), std::ios_base::trunc );
  if( !output_stream.good() )
  {
    ROS_ERROR_STREAM( "Unable to open file for writing " << output_path );
    return false;
  }

  output_stream << template_string.c_str();
  output_stream.close();

  return true; // file created successfully
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
      if( const YAML::Node *urdf_node = title_node->FindValue( "URDF" ) ) 
      {
        if( const YAML::Node *package_node = urdf_node->FindValue( "package" ) ) 
        {
          *package_node >> urdf_pkg_name_;
        } 
        return true;
      }
    }
    
    /*
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

      // Assign meta data to vector
      group_meta_data_[ group_name ] = new_meta_data;
    }
    */

  } 
  catch(YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM( e.what() );
  }

  return false; // if it gets to this point an error has occured
}




} // namespace
