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

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Constructor
// ******************************************************************************************
MoveItConfigData::MoveItConfigData()
{
  // Create an instance of SRDF writer for all widgets to share
  srdf_.reset( new SRDFWriter() );

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
// Provide a shared kinematic model loader
// ******************************************************************************************
planning_models_loader::KinematicModelLoaderPtr MoveItConfigData::getKinematicModelLoader()
{
  if( !kin_model_loader_ )
  {
    // Customize the loader with options
    planning_models_loader::KinematicModelLoader::Options opt(ROBOT_DESCRIPTION);
    opt.load_kinematics_solvers_ = false;

    // Initialize
    kin_model_loader_.reset(new planning_models_loader::KinematicModelLoader(opt));
  }

  return kin_model_loader_;
}

// ******************************************************************************************
// Provide a shared planning scene
// ******************************************************************************************
planning_scene_monitor::PlanningSceneMonitorPtr MoveItConfigData::getPlanningSceneMonitor()
{
  if( !planning_scene_monitor_ )
  {
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor( getKinematicModelLoader() ));
  }
  return planning_scene_monitor_;
}

// ******************************************************************************************
// Provide a kinematic model. Load a new one if necessary
// ******************************************************************************************
const planning_models::KinematicModelConstPtr& MoveItConfigData::getKinematicModel()
{
  if (!kin_model_)
  {
    kin_model_ = getKinematicModelLoader()->getModel();
  }
  return kin_model_;
}


}
