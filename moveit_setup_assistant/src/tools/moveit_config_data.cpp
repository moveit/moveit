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

#include "moveit_setup_assistant/tools/moveit_config_data.h"
// For writing yaml and launch files
#include <iostream> 
#include <fstream>

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
  emitter << YAML::Key << "path" << YAML::Value << urdf_path_;
  emitter << YAML::EndMap;

  /// SRDF Path Location
  emitter << YAML::Key << "SRDF";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "path" << YAML::Value << srdf_path_;
  emitter << YAML::EndMap;
  
  emitter << YAML::EndMap;
  std::ofstream outf( file_path.c_str(), std::ios_base::trunc );
  
  outf << emitter.c_str();

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
  std::ofstream outf( file_path.c_str(), std::ios_base::trunc );
  
  outf << emitter.c_str();

  return true; // file created successfully
}

// ******************************************************************************************
// Output kinematic config files
// ******************************************************************************************
bool MoveItConfigData::outputKinematicsYAML( const std::string& file_path )
{




  return true; // file created successfully
}

// ******************************************************************************************
// Output benchmark server launch file
// ******************************************************************************************
bool MoveItConfigData::outputBenchmarkServerLaunch( const std::string& file_path )
{




  return true; // file created successfully
}

// ******************************************************************************************
// Output move group launch file
// ******************************************************************************************
bool MoveItConfigData::outputMoveGroupLaunch( const std::string& file_path )
{




  return true; // file created successfully
}

// ******************************************************************************************
// Output OMPL Planner launch file
// ******************************************************************************************
bool MoveItConfigData::outputOMPLPlannerLaunch( const std::string& file_path )
{




  return true; // file created successfully
}

// ******************************************************************************************
// Output planning context launch file
// ******************************************************************************************
bool MoveItConfigData::outputPlanningContextLaunch( const std::string& file_path )
{




  return true; // file created successfully
}

// ******************************************************************************************
// Output warehouse launch file
// ******************************************************************************************
bool MoveItConfigData::outputWarehouseLaunch( const std::string& file_path )
{




  return true; // file created successfully
}

// ******************************************************************************************
// Output warehouse settings launch file
// ******************************************************************************************
bool MoveItConfigData::outputWarehouseSettingsLaunch( const std::string& file_path )
{




  return true; // file created successfully
}


} // namespace
