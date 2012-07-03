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
#include <ros/ros.h> // for the node handle
#include <moveit_setup_assistant/tools/srdf_writer.h> // for writing srdf data
#include <planning_scene/planning_scene.h> // for getting kinematic model
#include <planning_scene_monitor/planning_scene_monitor.h> // for getting monitor
#include <planning_models_loader/kinematic_model_loader.h>

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Constants
// ******************************************************************************************

// Used for loading kinematic model
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string MOVEIT_PLANNING_SCENE="moveit_planning_scene";

// ******************************************************************************************
// Class
// ******************************************************************************************
class MoveItConfigData
{
public:
  MoveItConfigData();
  ~MoveItConfigData();

  // All of the data needed for creating a MoveIt Configuration Files

  // Paths
  std::string urdf_path_;
  std::string srdf_path_;

  // URDF robot model
  urdf::Model urdf_model_;

  // SRDF Data and Writer
  SRDFWriterPtr srdf_;

  // Is this application in debug mode?
  bool debug_;

  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Provide a shared kinematic model loader
  planning_models_loader::KinematicModelLoaderPtr getKinematicModelLoader();

  /// Provide a shared planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor();

  /// Provide a kinematic model. Load a new one if necessary
  const planning_models::KinematicModelConstPtr& getKinematicModel();

  /// Share the same node handle throughout the application
  ros::NodeHandle& getNodeHandle();


private:

  // ******************************************************************************************
  // Private Vars
  // ******************************************************************************************

  // Shared kinematic model loader
  planning_models_loader::KinematicModelLoaderPtr kin_model_loader_;

  // Shared planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Shared kinematic model
  planning_models::KinematicModelConstPtr kin_model_;

  // Shared node handle
  ros::NodeHandle *nh_;
  
};

// ******************************************************************************************
// Boost Pointers
// ******************************************************************************************

/// Create a shared pointer for passing this data object between widgets
typedef boost::shared_ptr<MoveItConfigData> MoveItConfigDataPtr;

}

#endif
