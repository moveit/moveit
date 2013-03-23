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

#ifndef MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_MOVEIT_CONFIG_DATA_
#define MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_MOVEIT_CONFIG_DATA_

#include <boost/shared_ptr.hpp>
#include <srdfdom/model.h> // use their struct datastructures
#include <urdf/model.h> // to share throughout app
#include <moveit/setup_assistant/tools/srdf_writer.h> // for writing srdf data
#include <moveit/planning_scene/planning_scene.h> // for getting kinematic model
#include <moveit/collision_detection/collision_matrix.h> // for figuring out if robot is in collision

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Constants
// ******************************************************************************************
 
// Used for loading kinematic model
static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string MOVEIT_ROBOT_STATE = "moveit_robot_state";

// Default kin solver values
static const double DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION_ = 0.005;
static const double DEFAULT_KIN_SOLVER_TIMEOUT_ = 0.005;
static const int    DEFAULT_KIN_SOLVER_ATTEMPTS_ = 3;


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
  double kinematics_solver_timeout_; // solver timeout
  int kinematics_solver_attempts_; // solver attempts
};

// ******************************************************************************************
// Forward Declarations
// ******************************************************************************************
//class urdf::Model;

// ******************************************************************************************
// Class
// ******************************************************************************************
class MoveItConfigData
{
public:
  MoveItConfigData();
  ~MoveItConfigData();

  // All of the data needed for creating a MoveIt Configuration Files

  // ******************************************************************************************
  // URDF Data
  // ******************************************************************************************

  /// Full file-system path to urdf
  std::string urdf_path_; 

  /// Name of package containig urdf (note: this may be empty b/c user may not have urdf in pkg)
  std::string urdf_pkg_name_; 

  /// Path relative to urdf package (note: this may be same as urdf_path_)
  std::string urdf_pkg_relative_path_; 

  /// URDF robot model
  boost::shared_ptr<urdf::Model> urdf_model_;

  // ******************************************************************************************
  // SRDF Data
  // ******************************************************************************************

  /// Full file-system path to srdf
  std::string srdf_path_; 

  /// Path relative to loaded configuration package
  std::string srdf_pkg_relative_path_;
 
  /// SRDF Data and Writer
  SRDFWriterPtr srdf_;

  // ******************************************************************************************
  // Other Data
  // ******************************************************************************************

  /// Planning groups extra data not found in srdf but used in config files
  std::map<std::string, GroupMetaData> group_meta_data_;

  /// Setup Assistants package's path for when we use its templates
  std::string setup_assistant_path_; 

  /// Loaded configuration package path - if an existing package was loaded, holds that path
  std::string config_pkg_path_;

  /// Location that moveit_setup_assistant stores its templates
  std::string template_package_path_;
  
  /// Is this application in debug mode?
  bool debug_;

  /// Allowed collision matrix for robot poses
  collision_detection::AllowedCollisionMatrix allowed_collision_matrix_;

  /// Timestamp when configuration package was generated, if it was previously generated
  std::time_t config_pkg_generated_timestamp_;

  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Provide a shared kinematic model loader
  robot_model::RobotModelConstPtr getRobotModel();

  /// Update the Kinematic Model with latest SRDF modifications
  void updateRobotModel();

  /// Provide a shared planning scene
  planning_scene::PlanningScenePtr getPlanningScene();
  
  /** 
   * Find the associated group by name
   * 
   * @param name - name of data to find in datastructure
   * @return pointer to data in datastructure
   */
  srdf::Model::Group* findGroupByName( const std::string &name );
  
  /// Load the allowed collision matrix from the SRDF's list of link pairs
  void loadAllowedCollisionMatrix();

  // ******************************************************************************************
  // Public Functions for outputting configuration and setting files
  // ******************************************************************************************
  bool outputSetupAssistantFile( const std::string& file_path );
  bool outputOMPLPlanningYAML( const std::string& file_path );
  bool outputKinematicsYAML( const std::string& file_path );
  bool outputJointLimitsYAML( const std::string& file_path );

  /**
   * \brief Decide the best two joints to be used for the projection evaluator
   * \param planning_group name of group to use
   * \return string - value to insert into yaml file
   */
  std::string decideProjectionJoints(std::string planning_group);
     
  /** 
   * Input kinematics.yaml file for editing its values
   * @param file_path path to kinematics.yaml in the input package
   * @return bool if the file was read correctly
   */  
  bool inputKinematicsYAML( const std::string& file_path );

  /** 
   * Input .setup_assistant file - contains data used for the MoveIt Setup Assistant
   * 
   * @param file_path path to .setup_assistant file
   * @return bool if the file was read correctly
   */
  bool inputSetupAssistantYAML( const std::string& file_path );

  /**
   * Helper Function for joining a file path and a file name, or two file paths, etc, 
   * in a cross-platform way
   *
   * @param path1 first half of path
   * @param path2 second half of path, or filename
   * @return string resulting combined paths
   */
  std::string appendPaths( const std::string &path1, const std::string &path2 );

private:

  // ******************************************************************************************
  // Private Vars
  // ******************************************************************************************

  // Shared kinematic model
  robot_model::RobotModelPtr kin_model_;
  robot_model::RobotModelConstPtr kin_model_const_;

  // Shared planning scene
  planning_scene::PlanningScenePtr planning_scene_;
};

/// Create a shared pointer for passing this data object between widgets
typedef boost::shared_ptr<MoveItConfigData> MoveItConfigDataPtr;


} // namespace 

#endif
