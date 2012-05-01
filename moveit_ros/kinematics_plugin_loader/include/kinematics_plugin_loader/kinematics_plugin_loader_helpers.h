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

/* Author: E. Gil Jones */

#ifndef MOVEIT_KINEMATICS_PLUGIN_LOADER_HELPERS_
#define MOVEIT_KINEMATICS_PLUGIN_LOADER_HELPERS_

#include <kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <srdf/model.h>
#include <planning_models/kinematic_model.h>

namespace kinematics_plugin_loader
{

static inline void generateKinematicsLoaderMap(const planning_models::KinematicModelConstPtr& kinematic_model,
                                               const boost::shared_ptr<const srdf::Model>& srdf_model, 
                                               const boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader>& kinematics_plugin_loader,
                                               std::map<std::string, kinematics::KinematicsBasePtr>& solver_map) 
{
  const std::vector<srdf::Model::Group>& srdf_groups = srdf_model->getGroups();

  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_plugin_loader->getLoaderFunction();
  
  for(unsigned int i = 0; i < srdf_groups.size(); i++) {
    if(srdf_groups[i].subgroups_.size() == 0 &&
       srdf_groups[i].chains_.size() == 1) {
      if(!kinematics_plugin_loader->isGroupKnown(srdf_groups[i].name_)) {
        ROS_WARN_STREAM("Really should have loader for " << srdf_groups[i].name_);
        continue;
      }
      const planning_models::KinematicModel::JointModelGroup* jmg
        = kinematic_model->getJointModelGroup(srdf_groups[i].name_); 

      solver_map[srdf_groups[i].name_] = kinematics_allocator(jmg);
    }
  }
}
}

#endif
