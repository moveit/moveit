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
#include <iostream> // For writing yaml and launch files
#include <fstream>
#include <yaml-cpp/yaml.h> // outputing yaml config files
#include <boost/filesystem.hpp>  // for creating folders/files
#include <boost/algorithm/string.hpp> // for string find and replace in templates

#ifdef HAVE_NEW_YAMLCPP
#include <boost/optional.hpp> // optional
#endif

// ROS
#include <ros/console.h>
#include <ros/package.h> // for getting file path for loading images

#ifdef HAVE_NEW_YAMLCPP
namespace YAML {
  // Create a legacy Iterator that can be used with the yaml-cpp 0.3 API.
  class Iterator
  {
  public:
    typedef YAML::iterator iterator_t;
    typedef YAML::const_iterator const_iterator_t;

    Iterator (iterator_t iter) : iter_ (iter)
    {
    }

    const Node& first () const
    {
      return iter_->first;
    }

    const Node& second () const
    {
      return iter_->second;
    }

    detail::iterator_value operator*() { return *iter_; }

    Iterator operator++() { return Iterator (++iter_); }

    bool operator== (iterator_t const& other) const
    {
      return iter_ == other;
    }

    bool operator!= (iterator_t const& other) const
    {
      return iter_ != other;
    }

  private:
    iterator_t iter_;
  };
}
#endif

namespace moveit_setup_assistant
{

// File system
namespace fs = boost::filesystem;


#ifdef HAVE_NEW_YAMLCPP
typedef boost::optional<YAML::Node> yaml_node_t;

// Helper function to find a value (yaml-cpp 0.5)
template <typename T>
yaml_node_t findValue (const YAML::Node& node, const T& key)
{
  if (node[key]) return node[key];
  return yaml_node_t ();
}

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

#else
typedef const YAML::Node* yaml_node_t;

// Helper function to find a value (yaml-cpp 0.3)
template <typename T>
yaml_node_t findValue (const YAML::Node& node, const T& key)
{
  return node.FindValue (key);
}
#endif


// yaml-cpp 0.5 also changed how you load the YAML document.  This
// function hides the changes.
void loadYaml(std::istream& in_stream, YAML::Node& doc_out)
{
#ifdef HAVE_NEW_YAMLCPP
  doc_out = YAML::Load(in_stream);
#else
  YAML::Parser parser(in_stream);
  parser.GetNextDocument(doc_out);
#endif
}

// ******************************************************************************************
// Constructor
// ******************************************************************************************
MoveItConfigData::MoveItConfigData() :
  config_pkg_generated_timestamp_(0)
{
  // Create an instance of SRDF writer and URDF model for all widgets to share
  srdf_.reset( new srdf::SRDFWriter() );
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
  if( !robot_model_ )
  {
    // Initialize with a URDF Model Interface and a SRDF Model
    robot_model_.reset( new robot_model::RobotModel( urdf_model_, srdf_->srdf_model_ ) );
    robot_model_const_ = robot_model_;
  }

  return robot_model_const_;
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
  robot_model_.reset( new robot_model::RobotModel( urdf_model_, srdf_->srdf_model_ ) );
  robot_model_const_ = robot_model_;

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
  emitter << YAML::Key << "author_name" << YAML::Value << author_name_;
  emitter << YAML::Key << "author_email" << YAML::Value << author_email_;
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

  std::vector<OMPLPlannerDescription> planner_des;

  OMPLPlannerDescription SBL("SBL","geometric");
  SBL.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  planner_des.push_back(SBL);

  OMPLPlannerDescription EST("EST", "geometric");
  EST.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0 setup()" );
  EST.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  planner_des.push_back(EST);

  OMPLPlannerDescription LBKPIECE("LBKPIECE", "geometric");
  LBKPIECE.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  LBKPIECE.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9");
  LBKPIECE.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(LBKPIECE);

  OMPLPlannerDescription BKPIECE("BKPIECE", "geometric");
  BKPIECE.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  BKPIECE.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9");
  BKPIECE.addParameter("failed_expansion_score_factor", "0.5", "When extending motion fails, scale score by factor. default: 0.5");
  BKPIECE.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(BKPIECE);

  OMPLPlannerDescription KPIECE("KPIECE", "geometric");
  KPIECE.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  KPIECE.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05 ");
  KPIECE.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9 (0.0,1.]");
  KPIECE.addParameter("failed_expansion_score_factor", "0.5", "When extending motion fails, scale score by factor. default: 0.5");
  KPIECE.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(KPIECE);

  OMPLPlannerDescription RRT("RRT", "geometric");
  RRT.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  RRT.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  planner_des.push_back(RRT);

  OMPLPlannerDescription RRTConnect("RRTConnect", "geometric");
  RRTConnect.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  planner_des.push_back(RRTConnect);

  OMPLPlannerDescription RRTstar("RRTstar", "geometric");
  RRTstar.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  RRTstar.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  RRTstar.addParameter("delay_collision_checking", "1", "Stop collision checking as soon as C-free parent found. default 1");
  planner_des.push_back(RRTstar);

  OMPLPlannerDescription TRRT("TRRT", "geometric");
  TRRT.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  TRRT.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  TRRT.addParameter("max_states_failed", "10", "when to start increasing temp. default: 10");
  TRRT.addParameter("temp_change_factor", "2.0", "how much to increase or decrease temp. default: 2.0");
  TRRT.addParameter("min_temperature", "10e-10", "lower limit of temp change. default: 10e-10");
  TRRT.addParameter("init_temperature", "10e-6", "initial temperature. default: 10e-6");
  TRRT.addParameter("frountier_threshold", "0.0", "dist new state to nearest neighbor to disqualify as frontier. default: 0.0 set in setup() ");
  TRRT.addParameter("frountierNodeRatio", "0.1", "1/10, or 1 nonfrontier for every 10 frontier. default: 0.1");
  TRRT.addParameter("k_constant", "0.0", "value used to normalize expresssion. default: 0.0 set in setup()");
  planner_des.push_back(TRRT);

  OMPLPlannerDescription PRM("PRM", "geometric");
  PRM.addParameter("max_nearest_neighbors", "10", "use k nearest neighbors. default: 10");
  planner_des.push_back(PRM);

  OMPLPlannerDescription PRMstar("PRMstar", "geometric"); // no delcares in code
  planner_des.push_back(PRMstar);

  // Add Planners with parameter values 
  std::vector<std::string> pconfigs;
  for (std::size_t i = 0 ; i < planner_des.size() ; ++i)
  {
    std::string defaultconfig = planner_des[i].name_ + "kConfigDefault";
    emitter << YAML::Key << defaultconfig;
    emitter << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "type" << YAML::Value << "geometric::" + planner_des[i].name_;
    for(std::size_t  j=0; j<planner_des[i].parameter_list_.size(); j++) 
      {
	emitter << YAML::Key  << planner_des[i].parameter_list_[j].name;
	emitter << YAML::Value << planner_des[i].parameter_list_[j].value;
	emitter << YAML::Comment(planner_des[i].parameter_list_[j].comment.c_str());
      }
    emitter << YAML::EndMap;

    pconfigs.push_back(defaultconfig);
  }

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
    for (std::size_t i = 0 ; i < pconfigs.size() ; ++i)
      emitter << pconfigs[i];
    emitter << YAML::EndSeq;

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

bool MoveItConfigData::outputFakeControllersYAML( const std::string& file_path )
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  emitter << YAML::Key << "controller_list";
  emitter << YAML::Value << YAML::BeginSeq;

  // Union all the joints in groups
  std::set<const robot_model::JointModel*> joints;

  // Loop through groups
  for (std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin();
       group_it != srdf_->groups_.end();  ++group_it)
  {
    // Get list of associated joints
    const robot_model::JointModelGroup *joint_model_group =
      getRobotModel()->getJointModelGroup( group_it->name_ );
    emitter << YAML::BeginMap;
    const std::vector<const robot_model::JointModel*> &joint_models = joint_model_group->getActiveJointModels();
    emitter << YAML::Key << "name";
    emitter << YAML::Value << "fake_" + group_it->name_ + "_controller";
    emitter << YAML::Key << "joints";
    emitter << YAML::Value << YAML::BeginSeq;

    // Iterate through the joints
    for (std::vector<const robot_model::JointModel*>::const_iterator joint_it = joint_models.begin();
         joint_it != joint_models.end(); ++joint_it)
    {
      emitter << (*joint_it)->getName();
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;
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

  // Union all the joints in groups. Uses a custom comparator to allow the joints to be sorted by name
  std::set<const robot_model::JointModel*, joint_model_compare> joints;

  // Loop through groups
  for (std::vector<srdf::Model::Group>::iterator group_it = srdf_->groups_.begin();
       group_it != srdf_->groups_.end();  ++group_it)
  {
    // Get list of associated joints
    const robot_model::JointModelGroup *joint_model_group =
      getRobotModel()->getJointModelGroup( group_it->name_ );

    const std::vector<const robot_model::JointModel*> &joint_models = joint_model_group->getJointModels();

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
  for ( std::set<const robot_model::JointModel*>::iterator joint_it = joints.begin() ; joint_it != joints.end() ; ++joint_it )
  {
    emitter << YAML::Key << (*joint_it)->getName();
    emitter << YAML::Value << YAML::BeginMap;

    const robot_model::VariableBounds &b = (*joint_it)->getVariableBounds()[0];

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

  std::ofstream output_stream( file_path.c_str(), std::ios_base::trunc );
  if( !output_stream.good() )
  {
    ROS_ERROR_STREAM( "Unable to open file for writing " << file_path );
    return false;
  }
  // Add documentation into joint_limits.yaml
  output_stream << "# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed" << std::endl;
  output_stream << "# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]" << std::endl;
  output_stream << "# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]" << std::endl;
  output_stream << emitter.c_str();
  output_stream.close();

  return true; // file created successfully
}

// ******************************************************************************************
// Set list of collision link pairs in SRDF; sorted; with optional filter
// ******************************************************************************************


class SortableDisabledCollision
{
public:
  SortableDisabledCollision(const srdf::Model::DisabledCollision &dc) :
     dc_(dc), key_( dc.link1_ < dc.link2_ ? (dc.link1_ + "|" + dc.link2_) : (dc.link2_ + "|" + dc.link1_) )
  {
  }
  operator const srdf::Model::DisabledCollision () const
  {
    return dc_;

  }
  bool operator < (const SortableDisabledCollision &other) const
  {
    return key_ < other.key_;
  }
private:
  const srdf::Model::DisabledCollision dc_;
  const std::string key_;
};

void MoveItConfigData::setCollisionLinkPairs(const moveit_setup_assistant::LinkPairMap &link_pairs, size_t skip_mask){
  // Create temp disabled collision
  srdf::Model::DisabledCollision dc;

  std::set<SortableDisabledCollision> disabled_collisions;
  disabled_collisions.insert(srdf_->disabled_collisions_.begin(), srdf_->disabled_collisions_.end());

  // copy the data in this class's LinkPairMap datastructure to srdf::Model::DisabledCollision format
  for ( moveit_setup_assistant::LinkPairMap::const_iterator pair_it = link_pairs.begin();
        pair_it != link_pairs.end(); ++pair_it)
  {
    // Only copy those that are actually disabled
    if(pair_it->second.disable_check)
    {
      if((1 << pair_it->second.reason) & skip_mask) continue;

      dc.link1_ = pair_it->first.first;
      dc.link2_ = pair_it->first.second;
      dc.reason_ = moveit_setup_assistant::disabledReasonToString( pair_it->second.reason );

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
    YAML::Node doc;
    loadYaml(input_stream, doc);

    yaml_node_t prop_name;

    // Loop through all groups
#ifdef HAVE_NEW_YAMLCPP
    for( YAML::const_iterator group_it = doc.begin(); group_it != doc.end(); ++group_it )
#else
    for( YAML::Iterator group_it = doc.begin(); group_it != doc.end(); ++group_it )
#endif
    {
#ifdef HAVE_NEW_YAMLCPP
      const std::string group_name = group_it->first.as<std::string>();
#else
      std::string group_name;
      group_it.first() >> group_name;
#endif

      // Create new meta data
      GroupMetaData new_meta_data;

      // kinematics_solver
#ifdef HAVE_NEW_YAMLCPP
      if( prop_name = findValue( group_it->second, "kinematics_solver" ) )
#else
      if( prop_name = findValue( group_it.second(), "kinematics_solver" ) )
#endif
      {
        *prop_name >> new_meta_data.kinematics_solver_;
      }

      // kinematics_solver_search_resolution
#ifdef HAVE_NEW_YAMLCPP
      if( prop_name = findValue( group_it->second, "kinematics_solver_search_resolution" ) )
#else
      if( prop_name = findValue( group_it.second(), "kinematics_solver_search_resolution" ) )
#endif
      {
        *prop_name >> new_meta_data.kinematics_solver_search_resolution_;
      }
      else
      {
        new_meta_data.kinematics_solver_attempts_ = DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION_;
      }

      // kinematics_solver_timeout
#ifdef HAVE_NEW_YAMLCPP
      if( prop_name = findValue( group_it->second, "kinematics_solver_timeout" ) )
#else
      if( prop_name = findValue( group_it.second(), "kinematics_solver_timeout" ) )
#endif
      {
        *prop_name >> new_meta_data.kinematics_solver_timeout_;
      }
      else
      {
        new_meta_data.kinematics_solver_attempts_ = DEFAULT_KIN_SOLVER_TIMEOUT_;
      }

      // kinematics_solver_attempts
#ifdef HAVE_NEW_YAMLCPP
      if( prop_name = findValue( group_it->second, "kinematics_solver_attempts" ) )
#else
      if( prop_name = findValue( group_it.second(), "kinematics_solver_attempts" ) )
#endif
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
// Set package path; try to resolve path from package name if directory does not exist
// ******************************************************************************************
bool MoveItConfigData::setPackagePath( const std::string& pkg_path )
{
  std::string full_package_path;

  // check that the folder exists
  if( !fs::is_directory( pkg_path ) )
  {
    // does not exist, check if its a package
    full_package_path = ros::package::getPath( pkg_path );

    // check that the folder exists
    if( !fs::is_directory( full_package_path ) )
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

bool MoveItConfigData::getSetupAssistantYAMLPath( std::string& path )
{
  path = appendPaths(config_pkg_path_, ".setup_assistant");

  // Check if the old package is a setup assistant package
  return fs::is_regular_file( path );
}

// ******************************************************************************************
// Make the full URDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool MoveItConfigData::createFullURDFPath()
{
  boost::trim(urdf_pkg_name_);

  // Check if a package name was provided
  if( urdf_pkg_name_.empty() || urdf_pkg_name_ == "\"\"" )
  {
    urdf_path_ = urdf_pkg_relative_path_;
    urdf_pkg_name_.clear();
  }
  else
  {
    // Check that ROS can find the package
    std::string robot_desc_pkg_path = ros::package::getPath( urdf_pkg_name_ );

    if( robot_desc_pkg_path.empty() )
    {
      urdf_path_.clear();
      return false;
    }

    // Append the relative URDF url path
    urdf_path_ = appendPaths(robot_desc_pkg_path, urdf_pkg_relative_path_);
  }

  // Check that this file exits -------------------------------------------------
  return fs::is_regular_file( urdf_path_ );
}

// ******************************************************************************************
// Make the full SRDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool MoveItConfigData::createFullSRDFPath( const std::string& package_path )
{
  srdf_path_ = appendPaths(package_path, srdf_pkg_relative_path_);

  return fs::is_regular_file( srdf_path_ );
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
    YAML::Node doc;
    loadYaml(input_stream, doc);

    yaml_node_t title_node, urdf_node, package_node, srdf_node,
                relative_node, config_node, timestamp_node, author_name_node, author_email_node;

    // Get title node
    if( title_node = findValue( doc, "moveit_setup_assistant_config" ) )
    {
      // URDF Properties
      if( urdf_node = findValue( *title_node, "URDF" ) )
      {
        // Load first property
        if( package_node = findValue( *urdf_node, "package" ) )
        {
          *package_node >> urdf_pkg_name_;
        }
        else
        {
          return false; // if we do not find this value we cannot continue
        }

        // Load second property
        if( relative_node = findValue( *urdf_node, "relative_path" ) )
        {
          *relative_node >> urdf_pkg_relative_path_;
        }
        else
        {
          return false; // if we do not find this value we cannot continue
        }
      }
      // SRDF Properties
      if( srdf_node = findValue( *title_node, "SRDF" ) )
      {
        // Load first property
        if( relative_node = findValue( *srdf_node, "relative_path" ) )
        {
          *relative_node >> srdf_pkg_relative_path_;
        }
        else
        {
          return false; // if we do not find this value we cannot continue
        }
      }
      // Package generation time
      if( config_node = findValue( *title_node, "CONFIG" ) )
      {
        // Load author contact details
        if( author_name_node = findValue( *config_node, "author_name" ) )
        {
          *author_name_node >> author_name_;
        }
        if( author_email_node = findValue( *config_node, "author_email" ) )
        {
          *author_email_node >> author_email_;
        }
        // Load first property
        if( timestamp_node = findValue( *config_node, "generated_timestamp" ) )
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
