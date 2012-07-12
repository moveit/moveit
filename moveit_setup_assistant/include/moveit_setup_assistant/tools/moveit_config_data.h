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

#ifndef MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_TOOLS_MOVEIT_CONFIG_DATA_
#define MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_TOOLS_MOVEIT_CONFIG_DATA_

#include <boost/shared_ptr.hpp>
#include <srdfdom/model.h> // use their struct datastructures
#include <urdf/model.h> // to share throughout app
#include <moveit_setup_assistant/tools/srdf_writer.h> // for writing srdf data
#include <planning_scene/planning_scene.h> // for getting kinematic model
//#include <planning_scene_monitor/planning_scene_monitor.h> // for getting monitor
//#include <planning_models_loader/kinematic_model_loader.h>

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Constants
// ******************************************************************************************

// Used for loading kinematic model
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string MOVEIT_PLANNING_SCENE="moveit_planning_scene";

// ******************************************************************************************
// Structs
// ******************************************************************************************

/** 
 * Planning groups extra data not found in srdf but used in config files
 */
struct GroupMetaData
{
  std::string kinematics_solver_; // Name of kinematics plugin to use
  double kinematics_solver_search_resolution_; // resolution to use with solver
};

// ******************************************************************************************
// Class
// ******************************************************************************************
class MoveItConfigData
{
public:
  MoveItConfigData();
  ~MoveItConfigData();

  // All of the data needed for creating a MoveIt Configuration Files

  // SRDF Data and Writer
  SRDFWriterPtr srdf_;

  // URDF robot model
  boost::shared_ptr<urdf::Model> urdf_model_;
  
  /// Planning groups extra data not found in srdf but used in config files
  std::map<std::string, GroupMetaData> group_meta_data_;

  // Paths
  std::string urdf_path_;
  std::string srdf_path_;
  std::string setup_assistant_path_; // Remember Setup Assistants package's path for when we use its templates
  
  // Is this application in debug mode?
  bool debug_;

  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Provide a shared kinematic model loader
  planning_models::KinematicModelConstPtr getKinematicModel();

  /// Update the Kinematic Model with latest SRDF modifications
  void updateKinematicModel();

  /// Provide a shared planning scene
  planning_scene::PlanningScenePtr getPlanningScene();


  // ******************************************************************************************
  // Public Functions for outputting configuration and setting files
  // ******************************************************************************************
  bool outputPackageFiles( const std::string& template_package_path, 
                           const std::string& new_package_path,
                           const std::string& new_package_name );
  bool outputSetupAssistantFile( const std::string& file_path );
  bool outputOMPLPlanningYAML( const std::string& file_path );
  bool outputKinematicsYAML( const std::string& file_path );
  bool outputJointLimitsYAML( const std::string& file_path );
  bool outputBenchmarkServerLaunch( const std::string& file_path );
  bool outputMoveGroupLaunch( const std::string& file_path, 
                              const std::string& template_package_path, 
                              const std::string& new_package_name );
  bool outputOMPLPlannerLaunch( const std::string& file_path );
  bool outputPlanningContextLaunch( const std::string& file_path );
  bool outputWarehouseLaunch( const std::string& file_path );
  bool outputWarehouseSettingsLaunch( const std::string& file_path );

  /** 
   * Copy a template from location <template_path> to location <output_path> and replace package name
   * 
   * @param template_path path to template file
   * @param output_path desired path to copy to
   * @param new_package_name name of the new package being created, to replace key word in template
   * 
   * @return bool if the template was copied correctly
   */
  bool copyTemplate( const std::string& template_path, const std::string& output_path, 
                     const std::string& new_package_name );

  /** 
   * Input kinematics.yaml file for editing its values
   * @param file_path path to kinematics.yaml in the input package
   * @return bool if the file was read correctly
   */  
  bool inputKinematicsYAML( const std::string& file_path );


private:

  // ******************************************************************************************
  // Private Vars
  // ******************************************************************************************

  // Shared kinematic model
  planning_models::KinematicModelConstPtr kin_model_;

  // Shared planning scene
  planning_scene::PlanningScenePtr planning_scene_;
};

// ******************************************************************************************
// Boost Pointers
// ******************************************************************************************

/// Create a shared pointer for passing this data object between widgets
typedef boost::shared_ptr<MoveItConfigData> MoveItConfigDataPtr;


} // namespace

#endif
