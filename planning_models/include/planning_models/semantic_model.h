/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef PLANNING_MODELS_SEMANTIC_MODEL_
#define PLANNING_MODELS_SEMANTIC_MODEL_

#include <planning_models/kinematic_model.h>

namespace planning_models
{

struct SemanticGroup {

  SemanticGroup(const std::string& group_name) :
    is_arm_(false),
    has_end_effector_(false),
    is_end_effector_(false),
    is_multi_arm_(false),
    name_(group_name)
  {
  };

  bool is_arm_;
  bool has_end_effector_;
  bool is_end_effector_;
  bool is_multi_arm_;
  
  std::string name_;
  std::string end_effector_name_;
  std::string base_link_;
  std::string tip_link_;
  std::string attach_link_;

  std::vector<std::string> sub_arm_names_;

};

class SemanticModel {

public:
  
  SemanticModel(const KinematicModelConstPtr& kmodel,
                const boost::shared_ptr<const srdf::Model>& srdf_model);
  
  std::vector<std::string> getEndEffectorLinks(const std::string& group_name) const;
  std::vector<std::string> getGroupLinks(const std::string& group_name) const;
  std::vector<std::string> getGroupJoints(const std::string& group_name) const;
  std::string getAttachLink(const std::string& group_name) const;
  std::string getTipLink(const std::string& group_name) const;
  std::string getBaseLink(const std::string& group_name) const;
  std::string getEndEffector(const std::string& group_name) const;

  bool isArm(const std::string& group_name) const;
  bool isEndEffector(const std::string& group_name) const;
  bool hasEndEffector(const std::string& group_name) const;

  const planning_models::SemanticGroup* getGroup(const std::string& group_name ) const;

  std::string getModelName() const {
    return kmodel_->getName();
  }

protected:
  
  KinematicModelConstPtr kmodel_;

  std::map<std::string, SemanticGroup> semantic_group_map_;
};

typedef boost::shared_ptr<SemanticModel> SemanticModelPtr;
typedef boost::shared_ptr<const SemanticModel> SemanticModelConstPtr;

}

#endif
