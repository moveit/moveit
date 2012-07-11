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

#include <planning_scene_distance_field/planning_scene_distance_field.h>

namespace planning_scene {

PlanningSceneDistanceField::PlanningSceneDistanceField() 
  :PlanningScene()
{
}

PlanningSceneDistanceField::PlanningSceneDistanceField(const PlanningSceneConstPtr &parent) 
  :PlanningScene(parent)
{
  ROS_INFO_STREAM("Making diff");
}

bool PlanningSceneDistanceField::configure(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                                           const boost::shared_ptr<const srdf::Model> &srdf_model,
                                           const planning_models::KinematicModelPtr &kmodel)
{
  bool configured = PlanningScene::configure(urdf_model, srdf_model, kmodel);
  if(!parent_) {
    crobot_distance_.reset(new collision_distance_field::CollisionRobotDistanceField(kmodel));
    if(!cworld_distance_) {
      cworld_distance_.reset(new collision_distance_field::CollisionWorldDistanceField());
    }
  } else if(parent_->isConfigured()) {
    cworld_distance_.reset(new collision_distance_field::CollisionWorldDistanceField());
    cworld_distance_->recordChanges(true);
  } 
  ROS_INFO_STREAM("Calling configure");
  return configured;
}

planning_scene::PlanningScenePtr PlanningSceneDistanceField::diff(void) const
{
  return PlanningScenePtr(new PlanningSceneDistanceField(shared_from_this()));
}

void PlanningSceneDistanceField::addToObject(const std::string &id,
                                             const std::vector<shapes::ShapeConstPtr> &shapes,
                                             const std::vector<Eigen::Affine3d> &poses)
{
  PlanningScene::addToObject(id, shapes, poses);
  cworld_distance_->addToObject(id, shapes, poses);
}

void PlanningSceneDistanceField::addToObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  PlanningScene::addToObject(id, shape, pose);
  cworld_distance_->addToObject(id, shape, pose);
}

void PlanningSceneDistanceField::removeObject(const std::string& id){
  PlanningScene::removeObject(id);
  cworld_distance_->removeObject(id);
}


}
