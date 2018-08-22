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

#ifndef MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_MOVEIT_CONFIG_DATA_
#define MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_MOVEIT_CONFIG_DATA_

#include <boost/shared_ptr.hpp>
#include <srdfdom/model.h>        // use their struct datastructures
#include <srdfdom/srdf_writer.h>  // for writing srdf data
#include <urdf/model.h>           // to share throughout app
#include <yaml-cpp/yaml.h>        // outputing yaml config files
#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene/planning_scene.h>                     // for getting kinematic model
#include <moveit/collision_detection/collision_matrix.h>              // for figuring out if robot is in collision
#include <moveit/setup_assistant/tools/compute_default_collisions.h>  // for LinkPairMap

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
static const int DEFAULT_KIN_SOLVER_ATTEMPTS_ = 3;

// ******************************************************************************************
// Structs
// ******************************************************************************************

/**
 * Planning groups extra data not found in srdf but used in config files
 */
struct GroupMetaData
{
  std::string kinematics_solver_;               // Name of kinematics plugin to use
  double kinematics_solver_search_resolution_;  // resolution to use with solver
  double kinematics_solver_timeout_;            // solver timeout
  int kinematics_solver_attempts_;              // solver attempts
  std::string default_planner_;                 // Name of the default planner to use
};

/**
 * ROS Controllers settings which may be set in the config files
 */
struct ROSControlConfig
{
  std::string name_;                 // controller name
  std::string type_;                 // controller type
  std::vector<std::string> joints_;  // joints controller by this controller
};

/**
 * Planning parameters which may be set in the config files
 */
struct OmplPlanningParameter
{
  std::string name;     // name of parameter
  std::string value;    // value parameter will receive (but as a string)
  std::string comment;  // comment briefly describing what this parameter does
};

/** \brief This class describes the OMPL planners by name, type, and parameter list, used to create the
 * ompl_planning.yaml file */
class OMPLPlannerDescription
{
public:
  /** \brief Constructor
   *  @param name: name of planner
   *  @parameter type: type of planner
   */
  OMPLPlannerDescription(const std::string& name, const std::string& type)
  {
    name_ = name;
    type_ = type;
  };
  /** \brief Destructor */
  ~OMPLPlannerDescription()
  {
    parameter_list_.clear();
  };
  /** \brief adds a parameter to the planner description
   * @param name: name of parameter to add
   * @parameter: value: value of parameter as a string
   *  @parameter: value: value of parameter as a string
   */
  void addParameter(const std::string& name, const std::string& value = "", const std::string& comment = "")
  {
    OmplPlanningParameter temp;
    temp.name = name;
    temp.value = value;
    temp.comment = comment;
    parameter_list_.push_back(temp);
  }
  std::vector<OmplPlanningParameter> parameter_list_;
  std::string name_;  // name of planner
  std::string type_;  // type of planner (geometric)
};

/**
 * Reusable parameter class which may be used in reading and writing configuration files
 */
class GenericParameter
{
public:
  GenericParameter()
  {
    comment_ = "";
  };

  void setName(std::string name)
  {
    name_ = name;
  };
  void setValue(std::string value)
  {
    value_ = value;
  };
  void setComment(std::string comment)
  {
    comment_ = comment;
  };
  std::string getName()
  {
    return name_;
  };
  std::string getValue()
  {
    return value_;
  };
  std::string getComment()
  {
    return comment_;
  };

private:
  std::string name_;     // name of parameter
  std::string value_;    // value parameter will receive (but as a string)
  std::string comment_;  // comment briefly describing what this parameter does
};

MOVEIT_CLASS_FORWARD(MoveItConfigData);

/** \brief This class is shared with all widgets and contains the common configuration data
    needed for generating each robot's MoveIt! configuration package.

    All SRDF data is contained in a subclass of this class -
    srdf_writer.cpp. This class also contains the functions for writing
    out the configuration files. */
class MoveItConfigData
{
public:
  MoveItConfigData();
  ~MoveItConfigData();

  // bits of information that can be entered in Setup Assistant
  enum InformationFields
  {
    COLLISIONS = 1 << 1,
    VIRTUAL_JOINTS = 1 << 2,
    GROUPS = 1 << 3,
    GROUP_CONTENTS = 1 << 4,
    GROUP_KINEMATICS = 1 << 5,
    POSES = 1 << 6,
    END_EFFECTORS = 1 << 7,
    PASSIVE_JOINTS = 1 << 8,
    AUTHOR_INFO = 1 << 9,
    SENSORS_CONFIG = 1 << 10,
    SRDF = COLLISIONS | VIRTUAL_JOINTS | GROUPS | GROUP_CONTENTS | POSES | END_EFFECTORS | PASSIVE_JOINTS
  };
  unsigned long changes;  // bitfield of changes (composed of InformationFields)

  // All of the data needed for creating a MoveIt! Configuration Files

  // ******************************************************************************************
  // URDF Data
  // ******************************************************************************************

  /// Full file-system path to urdf
  std::string urdf_path_;

  /// Name of package containig urdf (note: this may be empty b/c user may not have urdf in pkg)
  std::string urdf_pkg_name_;

  /// Path relative to urdf package (note: this may be same as urdf_path_)
  std::string urdf_pkg_relative_path_;

  /// Flag indicating whether the URDF was loaded from .xacro format
  bool urdf_from_xacro_;
  /// xacro arguments
  std::string xacro_args_;

  /// URDF robot model
  urdf::ModelSharedPtr urdf_model_;

  /// URDF robot model string
  std::string urdf_string_;

  // ******************************************************************************************
  // SRDF Data
  // ******************************************************************************************

  /// Full file-system path to srdf
  std::string srdf_path_;

  /// Path relative to loaded configuration package
  std::string srdf_pkg_relative_path_;

  /// SRDF Data and Writer
  srdf::SRDFWriterPtr srdf_;

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

  /// Name of the author of this config
  std::string author_name_;

  /// Email of the author of this config
  std::string author_email_;

  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Load a robot model
  void setRobotModel(robot_model::RobotModelPtr robot_model);

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
  srdf::Model::Group* findGroupByName(const std::string& name);

  /// Load the allowed collision matrix from the SRDF's list of link pairs
  void loadAllowedCollisionMatrix();

  // ******************************************************************************************
  // Public Functions for outputting configuration and setting files
  // ******************************************************************************************
  std::vector<OMPLPlannerDescription> getOMPLPlanners();
  bool outputSetupAssistantFile(const std::string& file_path);
  bool outputOMPLPlanningYAML(const std::string& file_path);
  bool outputCHOMPPlanningYAML(const std::string& file_path);
  bool outputKinematicsYAML(const std::string& file_path);
  bool outputJointLimitsYAML(const std::string& file_path);
  bool outputFakeControllersYAML(const std::string& file_path);

  /**
   * Helper function for writing follow joint trajectory ROS controllers to ros_controllers.yaml
   * @param YAML Emitter - yaml emitter used to write the config to the ROS controllers yaml file
   * @param vector<ROSControlConfig> - a copy of ROS controllers config which will be modified in the function
   */
  void outputFollowJointTrajectoryYAML(YAML::Emitter& emitter,
                                       std::vector<ROSControlConfig>& ros_controllers_config_output);

  bool outputROSControllersYAML(const std::string& file_path);
  bool output3DSensorPluginYAML(const std::string& file_path);

  /**
   * \brief Parses the existing urdf and constructs a string from it with the elements required by gazebo simulator
   * added
   * \return gazebo compatible urdf or empty if error encountered
   */
  std::string getGazeboCompatibleURDF();

  /**
   * \brief Set list of collision link pairs in SRDF; sorted; with optional filter
   * \param link_pairs list of collision link pairs
   * \param skip_mask mask of shifted moveit_setup_assistant::DisabledReason values that will be skipped
   */
  void setCollisionLinkPairs(const moveit_setup_assistant::LinkPairMap& link_pairs, size_t skip_mask = 0);

  /**
   * \brief Decide the best two joints to be used for the projection evaluator
   * \param planning_group name of group to use
   * \return string - value to insert into yaml file
   */
  std::string decideProjectionJoints(std::string planning_group);

  /**
   * Input ompl_planning.yaml file for editing its values
   * @param file_path path to ompl_planning.yaml in the input package
   * @return true if the file was read correctly
   */
  bool inputOMPLYAML(const std::string& file_path);

  /**
   * Input kinematics.yaml file for editing its values
   * @param file_path path to kinematics.yaml in the input package
   * @return true if the file was read correctly
   */
  bool inputKinematicsYAML(const std::string& file_path);

  /**
   * Helper function for parsing ros_controllers.yaml file
   * @param YAML::Node - individual controller to be parsed
   * @return true if the file was read correctly
   */
  bool parseROSController(const YAML::Node& controller);

  /**
   * Helper function for parsing ros_controllers.yaml file
   * @param YAML::Node - controllers to be parsed
   * @return true if the file was read correctly
   */
  bool processROSControllers(const YAML::Node& controllers);

  /**
   * Input ros_controllers.yaml file for editing its values
   * @param file_path path to ros_controllers.yaml in the input package
   * @return true if the file was read correctly
   */
  bool inputROSControllersYAML(const std::string& file_path);

  /**
   * \brief Add a Follow Joint Trajectory action Controller for each Planning Group
   * \return true if controllers were added to the ros_controllers_config_ data structure
   */
  bool addDefaultControllers();

  /**
   * Set package path; try to resolve path from package name if directory does not exist
   * @param pkg_path path to package or package name
   * @return true if the path was set
   */
  bool setPackagePath(const std::string& pkg_path);

  /**
   * Resolve path to .setup_assistant file
   * @param path resolved path
   * @return true if the path could be resolved
   */
  bool getSetupAssistantYAMLPath(std::string& path);

  /// Make the full URDF path using the loaded .setup_assistant data
  bool createFullURDFPath();

  /// Make the full SRDF path using the loaded .setup_assistant data
  bool createFullSRDFPath(const std::string& package_path);

  /**
   * Input .setup_assistant file - contains data used for the MoveIt! Setup Assistant
   *
   * @param file_path path to .setup_assistant file
   * @return true if the file was read correctly
   */
  bool inputSetupAssistantYAML(const std::string& file_path);

  /**
   * Input sensors_3d file - contains 3d sensors config data
   *
   * @param default_file_path path to sensors_3d yaml file which contains default parameter values
   * @param file_path path to sensors_3d yaml file in the config package
   * @return true if the file was read correctly
   */
  bool input3DSensorsYAML(const std::string& default_file_path, const std::string& file_path = "");

  /**
   * Helper Function for joining a file path and a file name, or two file paths, etc,
   * in a cross-platform way
   *
   * @param path1 first half of path
   * @param path2 second half of path, or filename
   * @return string resulting combined paths
   */
  std::string appendPaths(const std::string& path1, const std::string& path2);

  /**
   * \brief Adds a ROS controller to ros_controllers_config_ vector
   * \param new_controller a new ROS Controller to add
   * \return true if inserted correctly
   */
  bool addROSController(const ROSControlConfig& new_controller);

  /**
   * \brief Gets ros_controllers_config_ vector
   * \return pointer to ros_controllers_config_
   */
  std::vector<ROSControlConfig>& getROSControllers();

  /**
   * Find the associated ROS controller by name
   *
   * @param controller_name - name of ROS controller to find in datastructure
   * @return pointer to data in datastructure
   */
  ROSControlConfig* findROSControllerByName(const std::string& controller_name);

  /**
   * Delete ROS controller by name
   *
   * @param controller_name - name of ROS controller to delete
   * @return true if deleted, false if not found
   */
  bool deleteROSController(const std::string& controller_name);

  /**
   * \brief Used for adding a sensor plugin configuation prameter to the sensor plugin configuration parameter list
   */
  void addGenericParameterToSensorPluginConfig(const std::string& name, const std::string& value = "",
                                               const std::string& comment = "");

  /**
   * \brief Clear the sensor plugin configuration parameter list
   */
  void clearSensorPluginConfig();

  /**
   * \brief Used for adding a sensor plugin configuation parameter to the sensor plugin configuration parameter list
   */
  std::vector<std::map<std::string, GenericParameter> > getSensorPluginConfig();

  /**
   * \brief Custom std::set comparator, used for sorting the joint_limits.yaml file into alphabetical order
   * \param jm1 - a pointer to the first joint model to compare
   * \param jm2 - a pointer to the second joint model to compare
   * \return bool of alphabetical sorting comparison
   */
  struct joint_model_compare
  {
    bool operator()(const robot_model::JointModel* jm1, const robot_model::JointModel* jm2) const
    {
      return jm1->getName() < jm2->getName();
    }
  };

private:
  // ******************************************************************************************
  // Private Vars
  // ******************************************************************************************

  /// Sensor plugin configuration parameter list, each sensor plugin type is a map
  std::vector<std::map<std::string, GenericParameter> > sensors_plugin_config_parameter_list_;

  /// Shared kinematic model
  robot_model::RobotModelPtr robot_model_;

  /// ROS Controllers config data
  std::vector<ROSControlConfig> ros_controllers_config_;

  /// Shared planning scene
  planning_scene::PlanningScenePtr planning_scene_;
};

}  // namespace

#endif
