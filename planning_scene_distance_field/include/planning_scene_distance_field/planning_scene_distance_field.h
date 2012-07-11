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

#ifndef MOVEIT_PLANNING_SCENE_DISTANCE_FIELD_
#define MOVEIT_PLANNING_SCENE_DISTANCE_FIELD_

#include <planning_scene/planning_scene.h>
#include <collision_distance_field/collision_robot_distance_field.h>
#include <collision_distance_field/collision_world_distance_field.h>

namespace planning_scene {

class PlanningSceneDistanceField : public PlanningScene {

public:

  PlanningSceneDistanceField(void);

  virtual ~PlanningSceneDistanceField(void) {
  }

  virtual PlanningScenePtr diff(void) const;

  const boost::shared_ptr<const collision_distance_field::CollisionWorldDistanceField> getCollisionWorldDistanceField() const {
    return cworld_distance_;
  }

  const boost::shared_ptr<const collision_distance_field::CollisionRobotDistanceField> getCollisionRobotDistanceField() const {
    if(parent_distance_field_) {
      return parent_distance_field_->getCollisionRobotDistanceField();
    }
    return crobot_distance_;
  }
  
  virtual bool configure(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                         const boost::shared_ptr<const srdf::Model> &srdf_model,
                         const planning_models::KinematicModelPtr &kmodel);
  
  virtual void moveShapeInObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose);

  virtual void addToObject(const std::string &id,
                           const std::vector<shapes::ShapeConstPtr> &shapes,
                           const std::vector<Eigen::Affine3d> &poses);

  virtual void addToObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose);

  virtual void removeObject(const std::string& id);

  virtual void removeAllObjects();

protected:

  PlanningSceneDistanceField(const PlanningSceneConstPtr &parent);

  std::map<std::string, std::vector<collision_distance_field::CollisionSphere> > coll_spheres_;

  boost::shared_ptr<const PlanningSceneDistanceField> parent_distance_field_;

  boost::shared_ptr<collision_distance_field::CollisionRobotDistanceField> crobot_distance_;
  boost::shared_ptr<collision_distance_field::CollisionWorldDistanceField> cworld_distance_;

};

}

#endif
