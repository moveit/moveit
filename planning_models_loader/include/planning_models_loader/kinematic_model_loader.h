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

/* Author: Ioan Sucan */

#ifndef MOVEIT_PLANNING_MODELS_LOADER_KINEMATIC_MODEL_LOADER_
#define MOVEIT_PLANNING_MODELS_LOADER_KINEMATIC_MODEL_LOADER_

#include <planning_models/kinematic_model.h>
#include <robot_model_loader/robot_model_loader.h>
#include <kinematics_plugin_loader/kinematics_plugin_loader.h>

namespace planning_models_loader
{

/** @class KinematicModelLoader */
class KinematicModelLoader
{
public:
  
  /** @brief Default constructor
   *  @param robot_description The string name corresponding to the ROS param where the URDF is loaded; Using the same parameter name plus the "_planning" suffix, additional configuration can be specified (e.g., additional joint limits)
   *  @param root_link The name of the link to consider as root of the model. By default (\e root_link is empty) the root will be the one specified in the URDF. However, it is possible to re-parent the tree using this argument. */
  KinematicModelLoader(const std::string &robot_description = "robot_description", const std::string &root_link = "");
  
  /** @brief Get the constructed planning_models::KinematicModel */
  const planning_models::KinematicModelPtr& getModel(void) const
  {
    return model_;
  }
  
  /** @brief Get the resolved parameter name for the robot description */
  const std::string& getRobotDescription(void) const
  {
    return robot_model_loader_->getRobotDescription();
  }
  
  /** @brief Get the parsed URDF model*/
  const boost::shared_ptr<urdf::Model>& getURDF(void) const
  {
    return robot_model_loader_->getURDF();
  }
  
  /** @brief Get the parsed SRDF model*/
  const boost::shared_ptr<srdf::Model>& getSRDF(void) const
  {
    return robot_model_loader_->getSRDF();
  }

  const robot_model_loader::RobotModelLoaderPtr& getRobotModelLoader(void) const
  {
    return robot_model_loader_;
  }

  /** \brief Get the kinematics solvers plugin loader. 
      \note This instance needs to be kept in scope, otherwise kinematics solver plugins may get unloaded. */
  const kinematics_plugin_loader::KinematicsPluginLoaderPtr& getKinematicsPluginLoader(void) const
  {
    return kinematics_loader_;
  }

  std::map<std::string, kinematics::KinematicsBasePtr> generateKinematicsSolversMap(void) const;
  
private:
  
  planning_models::KinematicModelPtr model_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_; 
  kinematics_plugin_loader::KinematicsPluginLoaderPtr kinematics_loader_;

};

typedef boost::shared_ptr<KinematicModelLoader> KinematicModelLoaderPtr;
typedef boost::shared_ptr<const KinematicModelLoader> KinematicModelLoaderConstPtr;

}
#endif
