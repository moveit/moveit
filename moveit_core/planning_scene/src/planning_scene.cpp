/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_world.h>
#include <moveit/collision_detection_fcl/collision_robot.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_state/conversions.h>
#include <octomap_msgs/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <set>

namespace planning_scene
{
const std::string PlanningScene::COLLISION_MAP_NS = "<collision_map>";
const std::string PlanningScene::OCTOMAP_NS = "<octomap>";
const std::string PlanningScene::DEFAULT_SCENE_NAME = "(noname)";
}

bool planning_scene::PlanningScene::isEmpty(const moveit_msgs::PlanningScene &msg)
{
  return msg.name.empty() && msg.fixed_frame_transforms.empty() && msg.allowed_collision_matrix.entry_names.empty() &&
    msg.link_padding.empty() && msg.link_scale.empty() && isEmpty(msg.robot_state) && isEmpty(msg.world);
}

bool planning_scene::PlanningScene::isEmpty(const moveit_msgs::RobotState &msg)
{
  return msg.multi_dof_joint_state.joint_names.empty() && msg.joint_state.name.empty() && msg.attached_collision_objects.empty();
}

bool planning_scene::PlanningScene::isEmpty(const moveit_msgs::PlanningSceneWorld &msg)
{
  return msg.collision_objects.empty() && msg.octomap.octomap.data.empty() && msg.collision_map.boxes.empty();
}

planning_scene::PlanningScenePtr planning_scene::PlanningScene::clone(const planning_scene::PlanningSceneConstPtr &scene)
{
  PlanningScenePtr result = scene->diff();
  result->decoupleParent();
  result->setName(scene->getName());
  return result;
}

planning_scene::PlanningScenePtr planning_scene::PlanningScene::diff(void) const
{
  return PlanningScenePtr(new PlanningScene(shared_from_this()));
}

planning_scene::PlanningScenePtr planning_scene::PlanningScene::diff(const moveit_msgs::PlanningScene &msg) const
{
  planning_scene::PlanningScenePtr result = diff();
  result->setPlanningSceneDiffMsg(msg);
  return result;
}

planning_scene::PlanningScene::PlanningScene(void) : configured_(false)
{
  name_ = DEFAULT_SCENE_NAME;
  setCollisionDetectionTypes<collision_detection::CollisionWorldFCL, collision_detection::CollisionRobotFCL>();
}

planning_scene::PlanningScene::PlanningScene(const PlanningSceneConstPtr &parent) : parent_(parent), configured_(false)
{
  if (parent_)
  {
    collision_detection_allocator_.reset(parent_->collision_detection_allocator_->clone());
    if (parent_->isConfigured())
      configure(parent_->getKinematicModel()->getURDF(), parent_->getKinematicModel()->getSRDF());
    if (!parent_->getName().empty())
      name_ = parent_->getName() + "+";
  }
  else
  {
    logError("NULL parent scene specified. Ignoring.");
    name_ = DEFAULT_SCENE_NAME; 
    setCollisionDetectionTypes<collision_detection::CollisionWorldFCL, collision_detection::CollisionRobotFCL>();
  }
}

bool planning_scene::PlanningScene::configure(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                                              const boost::shared_ptr<const srdf::Model> &srdf_model,
                                              const std::string &root_link)
{   
  if (!urdf_model || !srdf_model)
  {
    configured_ = false;
    return false;
  }

  if (!parent_)
  {
    bool same = configured_ && kmodel_->getURDF() == urdf_model && kmodel_->getSRDF() == srdf_model;
    if (!same || !kmodel_ || kmodel_->getRootLinkName() != root_link)
    {
      kinematic_model::KinematicModelPtr newModel;
      if (root_link.empty())
        newModel.reset(new kinematic_model::KinematicModel(urdf_model, srdf_model));
      else
      {   
        const urdf::Link *root_link_ptr = urdf_model->getLink(root_link).get();
        if (root_link_ptr)
          newModel.reset(new kinematic_model::KinematicModel(urdf_model, srdf_model, root_link));
        else
        {
          logError("Link '%s' (to be used as root) was not found in model '%s'. Attempting to construct model with default root link instead.",
                   root_link.c_str(), urdf_model->getName().c_str());
          newModel.reset(new kinematic_model::KinematicModel(urdf_model, srdf_model));
        }
      }
      return configure(newModel);
    }
  }
  else
    return configure(kinematic_model::KinematicModelPtr());
  return isConfigured();
}

bool planning_scene::PlanningScene::configure(const kinematic_model::KinematicModelPtr &kmodel)
{
  if (!kmodel && !parent_)
  {
    configured_ = false;
    return false;
  }
  
  if (!parent_)
  {
    // nothing other than perhaps the root link has changed since the last call to configure()
    bool same = configured_ && kmodel_ == kmodel;
    if (!same)
    {
      kmodel_ = kmodel;
      kmodel_const_ = kmodel_;
      ftf_.reset(new kinematic_state::Transforms(kmodel_->getModelFrame()));
      ftf_const_ = ftf_;
      
      if (kstate_)
      {
        // keep the same joint values, update the transforms if needed
        std::map<std::string, double> jsv;
        kstate_->getStateValues(jsv);
        kstate_.reset(new kinematic_state::KinematicState(kmodel_));
        kstate_->setStateValues(jsv);
      }
      else
      {
        kstate_.reset(new kinematic_state::KinematicState(kmodel_));
        kstate_->setToDefaultValues();
      }
      
      // no need to reset this if the scene was previously configured
      if (!acm_)
      {
        acm_.reset(new collision_detection::AllowedCollisionMatrix());
        // Use default collision operations in the SRDF to setup the acm
        acm_->setEntry(getKinematicModel()->getLinkModelNamesWithCollisionGeometry(),
                      getKinematicModel()->getLinkModelNamesWithCollisionGeometry(), false);
  
        // allow collisions for pairs that have been disabled
        const std::vector<srdf::Model::DisabledCollision> &dc = getKinematicModel()->getSRDF()->getDisabledCollisionPairs();
        for (std::size_t i = 0 ; i < dc.size() ; ++i)
        {
          acm_->setEntry(dc[i].link1_, dc[i].link2_, true);
        }
        
      }
            
      crobot_ = collision_detection_allocator_->allocateRobot(kmodel_);
      crobot_unpadded_ = collision_detection_allocator_->allocateRobot(kmodel_);
      crobot_const_ = crobot_;
      crobot_unpadded_const_ = crobot_unpadded_;
      
      // no need to change the world if it was previously configured;
      // there is a catch though: the frame for planning may have changed, if a different root link was specified;
      // however, this is direcly requested by the user
      if (!cworld_)
      {
        cworld_ = collision_detection_allocator_->allocateWorld();
        cworld_const_ = cworld_;
        
        colors_.reset(new std::map<std::string, std_msgs::ColorRGBA>());
      }
      
      configured_ = true;
    }
  }
  else
  {
    if (parent_->isConfigured())
    {
      // even if we have a parent, we do maintain a separate world representation, one that records changes
      // this is cheap however, because the worlds share the world representation
      cworld_ = collision_detection_allocator_->allocateWorld(parent_->getCollisionWorld());
      cworld_->recordChanges(true);
      cworld_const_ = cworld_;
      configured_ = true;
    }
    else
      logError("Parent is not configured yet");
  }

  return isConfigured();
}

const kinematic_model::KinematicModelPtr& planning_scene::PlanningScene::getKinematicModelNonConst(void)
{
  if (kmodel_ || !parent_)
    return kmodel_;
  const PlanningScene *p = parent_.get();
  const kinematic_model::KinematicModelPtr *r = NULL;
  while (p && (!r || !*r))
  {
    r = &p->kmodel_;
    p = p->parent_.get();
  }
  return *r;
}

void planning_scene::PlanningScene::clearDiffs(void)
{
  if (!parent_)
    return;

  // clear everything, reset the world
  cworld_ = collision_detection_allocator_->allocateWorld(parent_->getCollisionWorld());
  cworld_->recordChanges(true);
  cworld_const_ = cworld_;

  kmodel_.reset();
  kmodel_const_.reset();
  ftf_.reset();
  ftf_const_.reset();
  kstate_.reset();
  acm_.reset();
  crobot_.reset();
  crobot_const_.reset();
  crobot_unpadded_.reset();
  crobot_unpadded_const_.reset();
  colors_.reset();
}

void planning_scene::PlanningScene::pushDiffs(const PlanningScenePtr &scene)
{
  if (!parent_)
    return;

  if (ftf_)
    *scene->getTransforms() = *ftf_;

  if (kstate_)
    scene->getCurrentState() = *kstate_;

  if (acm_)
    scene->getAllowedCollisionMatrix() = *acm_;

  if (crobot_)
  {
    scene->getCollisionRobot()->setLinkPadding(crobot_->getLinkPadding());
    scene->getCollisionRobot()->setLinkScale(crobot_->getLinkScale());
  }

  if (cworld_->isRecordingChanges())
  {
    const std::vector<collision_detection::CollisionWorld::Change> &changes = cworld_->getChanges();
    if (!changes.empty())
    {
      collision_detection::CollisionWorldPtr w = scene->getCollisionWorld();
      for (std::size_t i = 0 ; i < changes.size() ; ++i)
        if (changes[i].type_ == collision_detection::CollisionWorld::Change::ADD)
        {
          collision_detection::CollisionWorld::ObjectConstPtr obj = cworld_->getObject(changes[i].id_);
          if (hasColor(changes[i].id_))
            scene->setColor(changes[i].id_, getColor(changes[i].id_));
          w->addToObject(obj->id_, obj->shapes_, obj->shape_poses_);
        }
        else
          if (changes[i].type_ == collision_detection::CollisionWorld::Change::REMOVE)
          {
            w->removeObject(changes[i].id_);
            scene->removeColor(changes[i].id_);
          }
          else
            logError("Unknown change on collision world");
    }
  }
}

double planning_scene::PlanningScene::distanceToCollisionUnpadded(const kinematic_state::KinematicState &kstate) const
{
  return getCollisionWorld()->distanceRobot(*getCollisionRobotUnpadded(), kstate);
}

double planning_scene::PlanningScene::distanceToCollisionUnpadded(const kinematic_state::KinematicState &kstate, const collision_detection::AllowedCollisionMatrix& acm) const
{
  return getCollisionWorld()->distanceRobot(*getCollisionRobotUnpadded(), kstate, acm);
}

double planning_scene::PlanningScene::distanceToCollision(const kinematic_state::KinematicState &kstate) const
{
  return getCollisionWorld()->distanceRobot(*getCollisionRobot(), kstate);
}

double planning_scene::PlanningScene::distanceToCollision(const kinematic_state::KinematicState &kstate, const collision_detection::AllowedCollisionMatrix& acm) const
{
  return getCollisionWorld()->distanceRobot(*getCollisionRobot(), kstate, acm);
}

void planning_scene::PlanningScene::checkCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult &res) const
{
  checkCollision(req, res, getCurrentState());
}

void planning_scene::PlanningScene::checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult &res) const
{
  checkSelfCollision(req, res, getCurrentState());
}

void planning_scene::PlanningScene::checkCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult &res,
                                                   const kinematic_state::KinematicState &kstate) const
{
  // check collision with the world using the padded version
  if (parent_)
    getCollisionWorld()->checkRobotCollision(req, res, *getCollisionRobot(), kstate, getAllowedCollisionMatrix());
  else
    getCollisionWorld()->checkRobotCollision(req, res, *crobot_, kstate, *acm_);
  
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
  {
    // do self-collision checking with the unpadded version of the robot
    if (parent_)
      getCollisionRobotUnpadded()->checkSelfCollision(req, res, kstate, getAllowedCollisionMatrix());
    else
      getCollisionRobotUnpadded()->checkSelfCollision(req, res, kstate, *acm_);
  }
}

void planning_scene::PlanningScene::checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult &res,
                                                       const kinematic_state::KinematicState &kstate) const
{
  // do self-collision checking with the unpadded version of the robot
  getCollisionRobotUnpadded()->checkSelfCollision(req, res, kstate, getAllowedCollisionMatrix());
}

void planning_scene::PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                                   collision_detection::CollisionResult &res,
                                                   const kinematic_state::KinematicState &kstate,
                                                   const collision_detection::AllowedCollisionMatrix& acm) const
{
  // check collision with the world using the padded version
  getCollisionWorld()->checkRobotCollision(req, res, *getCollisionRobot(), kstate, acm);
  
  // do self-collision checking with the unpadded version of the robot
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    getCollisionRobotUnpadded()->checkSelfCollision(req, res, kstate, acm);
}

void planning_scene::PlanningScene::checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
							   collision_detection::CollisionResult &res) const
{
  return checkCollisionUnpadded(req, res, getCurrentState(), getAllowedCollisionMatrix());
}

void planning_scene::PlanningScene::checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
							   collision_detection::CollisionResult &res,
							   const kinematic_state::KinematicState &kstate) const
{
  return checkCollisionUnpadded(req, res, kstate, getAllowedCollisionMatrix());
}

void planning_scene::PlanningScene::checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                                                           collision_detection::CollisionResult &res,
                                                           const kinematic_state::KinematicState &kstate,
                                                           const collision_detection::AllowedCollisionMatrix& acm) const
{
  // check collision with the world using the padded version
  getCollisionWorld()->checkRobotCollision(req, res, *getCollisionRobotUnpadded(), kstate, acm);

  // do self-collision checking with the unpadded version of the robot
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
  {
    getCollisionRobotUnpadded()->checkSelfCollision(req, res, kstate, acm);
  }
}

void planning_scene::PlanningScene::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                       collision_detection::CollisionResult &res,
                                                       const kinematic_state::KinematicState &kstate,
                                                       const collision_detection::AllowedCollisionMatrix& acm) const
{
  // do self-collision checking with the unpadded version of the robot
  getCollisionRobotUnpadded()->checkSelfCollision(req, res, kstate, acm);
}

void planning_scene::PlanningScene::getCollidingLinks(std::vector<std::string> &links) const
{
  getCollidingLinks(links, getCurrentState());
}


void planning_scene::PlanningScene::getCollidingLinks(std::vector<std::string> &links,
                                                      const kinematic_state::KinematicState &kstate) const
{
  getCollidingLinks(links, kstate, getAllowedCollisionMatrix());
}


void planning_scene::PlanningScene::getCollidingLinks(std::vector<std::string> &links,
                                                      const kinematic_state::KinematicState &kstate,
                                                      const collision_detection::AllowedCollisionMatrix& acm) const
{ 
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = getKinematicModel()->getLinkModelsWithCollisionGeometry().size() + 1;
  req.max_contacts_per_pair = 1;
  collision_detection::CollisionResult res;
  checkCollision(req, res, kstate, acm);
  links.clear();
  for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin() ; it != res.contacts.end() ; ++it)
    for (std::size_t j = 0 ; j < it->second.size() ; ++j)
    {
      if (it->second[j].body_type_1 == collision_detection::BodyTypes::ROBOT_LINK)
        links.push_back(it->second[j].body_name_1);
      if (it->second[j].body_type_2 == collision_detection::BodyTypes::ROBOT_LINK)
        links.push_back(it->second[j].body_name_2);
    }
}

const collision_detection::CollisionRobotPtr& planning_scene::PlanningScene::getCollisionRobot(void)
{
  if (!crobot_)
  {
    crobot_ = collision_detection_allocator_->allocateRobot(parent_->getCollisionRobot());
    crobot_const_ = crobot_;
  }
  return crobot_;
}

kinematic_state::KinematicState& planning_scene::PlanningScene::getCurrentState(void)
{
  if (!kstate_)
    kstate_.reset(new kinematic_state::KinematicState(parent_->getCurrentState()));
  return *kstate_;
}

collision_detection::AllowedCollisionMatrix& planning_scene::PlanningScene::getAllowedCollisionMatrix(void)
{
  if (!acm_)
    acm_.reset(new collision_detection::AllowedCollisionMatrix(parent_->getAllowedCollisionMatrix()));
  return *acm_;
}

const kinematic_state::TransformsPtr& planning_scene::PlanningScene::getTransforms(void)
{
  if (!ftf_)
  {
    ftf_.reset(new kinematic_state::Transforms(*parent_->getTransforms()));
    ftf_const_ = ftf_;
  }
  return ftf_;
}

void planning_scene::PlanningScene::getPlanningSceneDiffMsg(moveit_msgs::PlanningScene &scene) const
{
  scene.name = name_;
  scene.robot_model_root = getKinematicModel()->getRootLinkName();
  scene.robot_model_name = getKinematicModel()->getName();
  scene.is_diff = true;
  
  if (ftf_)
    ftf_->getTransforms(scene.fixed_frame_transforms);
  else
    scene.fixed_frame_transforms.clear();

  if (kstate_)
    kinematic_state::kinematicStateToRobotState(*kstate_, scene.robot_state);
  else
    scene.robot_state = moveit_msgs::RobotState();

  if (acm_)
    acm_->getMessage(scene.allowed_collision_matrix);
  else
    scene.allowed_collision_matrix = moveit_msgs::AllowedCollisionMatrix();

  if (crobot_)
  {
    crobot_->getPadding(scene.link_padding);
    crobot_->getScale(scene.link_scale);
  }
  else
  {
    scene.link_padding.clear();
    scene.link_scale.clear();
  }
  
  scene.object_colors.clear();
  if (colors_)
  {
    unsigned int i = 0;
    scene.object_colors.resize(colors_->size());
    for (ColorMap::const_iterator it = colors_->begin() ; it != colors_->end() ; ++it, ++i)
    {
      scene.object_colors[i].id = it->first;
      scene.object_colors[i].color = it->second;
    }
  }
  
  scene.world.collision_objects.clear();
  scene.world.collision_map = moveit_msgs::CollisionMap();
  scene.world.octomap = octomap_msgs::OctomapWithPose();
    
  if (cworld_->isRecordingChanges())
  {
    bool do_cmap = false;
    bool do_omap = false;
    const std::vector<collision_detection::CollisionWorld::Change> &changes = cworld_->getChanges();
    for (std::size_t i = 0 ; i < changes.size() ; ++i)
      if (changes[i].id_ == COLLISION_MAP_NS)
        do_cmap = true;
      else
        if (changes[i].id_ == OCTOMAP_NS)
          do_omap = true;
        else
        {
          if (changes[i].type_ == collision_detection::CollisionWorld::Change::ADD)
          {
            getPlanningSceneMsgCollisionObject(scene, changes[i].id_);
          }
          else
            if (changes[i].type_ == collision_detection::CollisionWorld::Change::REMOVE)
            {
              moveit_msgs::CollisionObject co;
              co.header.frame_id = getPlanningFrame();
              co.id = changes[i].id_;
              co.operation = moveit_msgs::CollisionObject::REMOVE;
              scene.world.collision_objects.push_back(co);
            }
            else
              logError("Unknown change on collision world");
        }
    if (do_cmap)
      getPlanningSceneMsgCollisionMap(scene);
    if (do_omap)
      getPlanningSceneMsgOctomap(scene);
  }
}

namespace planning_scene
{
namespace
{
class ShapeVisitorAddToCollisionObject : public boost::static_visitor<void>
{
public:
  
  ShapeVisitorAddToCollisionObject(moveit_msgs::CollisionObject *obj) :
    boost::static_visitor<void>(), obj_(obj)
  {
  }

  void setPoseMessage(const geometry_msgs::Pose *pose)
  {
    pose_ = pose;
  }
    
  void operator()(const shape_msgs::Plane &shape_msg) const
  {
    obj_->planes.push_back(shape_msg);   
    obj_->plane_poses.push_back(*pose_);
  }
  
  void operator()(const shape_msgs::Mesh &shape_msg) const
  {
    obj_->meshes.push_back(shape_msg);
    obj_->mesh_poses.push_back(*pose_);
  }
  
  void operator()(const shape_msgs::SolidPrimitive &shape_msg) const
  {
    obj_->primitives.push_back(shape_msg);
    obj_->primitive_poses.push_back(*pose_);
  }
  
private:
  
  moveit_msgs::CollisionObject *obj_;
  const geometry_msgs::Pose *pose_;
};
}
}

void planning_scene::PlanningScene::getPlanningSceneMsgCollisionObject(moveit_msgs::PlanningScene &scene, const std::string &ns) const
{ 
  moveit_msgs::CollisionObject co;
  co.header.frame_id = getPlanningFrame();
  co.id = ns;
  co.operation = moveit_msgs::CollisionObject::ADD;
  collision_detection::CollisionWorld::ObjectConstPtr obj = getCollisionWorld()->getObject(ns);
  if (!obj) 
    return;  
  ShapeVisitorAddToCollisionObject sv(&co);
  for (std::size_t j = 0 ; j < obj->shapes_.size() ; ++j)
  {
    shapes::ShapeMsg sm;
    if (constructMsgFromShape(obj->shapes_[j].get(), sm))
    {
      geometry_msgs::Pose p;
      tf::poseEigenToMsg(obj->shape_poses_[j], p);
      sv.setPoseMessage(&p);
      boost::apply_visitor(sv, sm);
    }
  }
  if (!co.primitives.empty() || !co.meshes.empty() || !co.planes.empty())
    scene.world.collision_objects.push_back(co);
}

void planning_scene::PlanningScene::getPlanningSceneMsgCollisionObjects(moveit_msgs::PlanningScene &scene) const
{
  scene.world.collision_objects.clear();
  const std::vector<std::string> &ns = getCollisionWorld()->getObjectIds();
  for (std::size_t i = 0 ; i < ns.size() ; ++i)
    if (ns[i] != COLLISION_MAP_NS && ns[i] != OCTOMAP_NS)
      getPlanningSceneMsgCollisionObject(scene, ns[i]);
}

void planning_scene::PlanningScene::getPlanningSceneMsgCollisionMap(moveit_msgs::PlanningScene &scene) const
{
  scene.world.collision_map.header.frame_id = getPlanningFrame();
  scene.world.collision_map.boxes.clear();
  if (getCollisionWorld()->hasObject(COLLISION_MAP_NS))
  {
    collision_detection::CollisionWorld::ObjectConstPtr map = getCollisionWorld()->getObject(COLLISION_MAP_NS);
    for (std::size_t i = 0 ; i < map->shapes_.size() ; ++i)
    {
      if (map->shapes_[i]->type != shapes::BOX)
      {
        logError("Shape that is not a box was found in '%s' object. Skipping shape.", COLLISION_MAP_NS.c_str());
        continue;
      }
      const shapes::Box *b = static_cast<const shapes::Box*>(map->shapes_[i].get());
      moveit_msgs::OrientedBoundingBox obb;
      obb.extents.x = b->size[0]; obb.extents.y = b->size[1]; obb.extents.z = b->size[2];
      tf::poseEigenToMsg(map->shape_poses_[i], obb.pose);
      scene.world.collision_map.boxes.push_back(obb);
    }
  }
}

void planning_scene::PlanningScene::getPlanningSceneMsgOctomap(moveit_msgs::PlanningScene &scene) const
{  
  scene.world.octomap.header.frame_id = getPlanningFrame();
  scene.world.octomap.octomap = octomap_msgs::Octomap();
  
  if (getCollisionWorld()->hasObject(OCTOMAP_NS))
  {
    collision_detection::CollisionWorld::ObjectConstPtr map = getCollisionWorld()->getObject(OCTOMAP_NS);
    if (map->shapes_.size() == 1)
    {
      const shapes::OcTree *o = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
      octomap_msgs::fullMapToMsg(*o->octree, scene.world.octomap.octomap);
      tf::poseEigenToMsg(map->shape_poses_[0], scene.world.octomap.origin);
    }
    else
      logError("Unexpected number of shapes in octomap collision object. Not including '%s' object", OCTOMAP_NS.c_str());
  }
}

void planning_scene::PlanningScene::getPlanningSceneMsg(moveit_msgs::PlanningScene &scene) const
{
  scene.name = name_;
  scene.is_diff = false;
  scene.robot_model_root = getKinematicModel()->getRootLinkName();
  scene.robot_model_name = getKinematicModel()->getName();
  getTransforms()->getTransforms(scene.fixed_frame_transforms);

  kinematic_state::kinematicStateToRobotState(getCurrentState(), scene.robot_state);
  getAllowedCollisionMatrix().getMessage(scene.allowed_collision_matrix);
  getCollisionRobot()->getPadding(scene.link_padding);
  getCollisionRobot()->getScale(scene.link_scale);
  scene.object_colors.clear();

  unsigned int i = 0;
  ColorMap cmap;
  getKnownColors(cmap);
  scene.object_colors.resize(cmap.size());
  for (ColorMap::const_iterator it = cmap.begin() ; it != cmap.end() ; ++it, ++i)
  {
    scene.object_colors[i].id = it->first;
    scene.object_colors[i].color = it->second;
  }
  
  // add collision objects
  getPlanningSceneMsgCollisionObjects(scene);

  // get the octomap
  getPlanningSceneMsgOctomap(scene);

  // get the collision map
  getPlanningSceneMsgCollisionMap(scene);
}

void planning_scene::PlanningScene::saveGeometryToStream(std::ostream &out) const
{
  out << name_ << std::endl;
  const std::vector<std::string> &ns = getCollisionWorld()->getObjectIds();
  for (std::size_t i = 0 ; i < ns.size() ; ++i)
    if (ns[i] != COLLISION_MAP_NS && ns[i] != OCTOMAP_NS)
    {
      collision_detection::CollisionWorld::ObjectConstPtr obj = getCollisionWorld()->getObject(ns[i]);
      if (obj)
      {
        out << "* " << ns[i] << std::endl;
        out << obj->shapes_.size() << std::endl;
        for (std::size_t j = 0 ; j < obj->shapes_.size() ; ++j)
        {
          shapes::saveAsText(obj->shapes_[j].get(), out);
          out << obj->shape_poses_[j].translation().x() << " " << obj->shape_poses_[j].translation().y() << " " << obj->shape_poses_[j].translation().z() << std::endl;
          Eigen::Quaterniond r(obj->shape_poses_[j].rotation());
          out << r.x() << " " << r.y() << " " << r.z() << " " << r.w() << std::endl;
          if (hasColor(ns[i]))
          {
            const std_msgs::ColorRGBA &c = getColor(ns[i]);
            out << c.r << " " << c.g << " " << c.b << " " << c.a << std::endl;
          }
          else
            out << "0 0 0 0" << std::endl;
        }
      }
    }
  out << "." << std::endl;
}

void planning_scene::PlanningScene::loadGeometryFromStream(std::istream &in)
{
  if (!in.good() || in.eof())
    return;  
  std::getline(in, name_);
  do
  {
    std::string marker;
    in >> marker;
    if (!in.good() || in.eof())
      return;
    if (marker == "*")
    {
      std::string ns;
      std::getline(in, ns);
      if (!in.good() || in.eof())
        return;  
      unsigned int shape_count;
      in >> shape_count;
      for (std::size_t i = 0 ; i < shape_count && in.good() && !in.eof() ; ++i)
      {
        shapes::Shape *s = shapes::constructShapeFromText(in);
        double x, y, z, rx, ry, rz, rw;
        in >> x >> y >> z;
        in >> rx >> ry >> rz >> rw;
        float r, g, b, a;
        in >> r >> g >> b >> a;
        if (s)
        {
          Eigen::Affine3d pose = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(rw, rx, ry, rz);          
          getCollisionWorld()->addToObject(ns, shapes::ShapePtr(s), pose);
          if (r > 0.0f || g > 0.0f || b > 0.0f || a > 0.0f)
          {
            std_msgs::ColorRGBA color;
            color.r = r; color.g = g; color.b = b; color.a = a;
            setColor(ns, color);
          }
        }
      }
    }
    else
      break;
  } while (true);
}

void planning_scene::PlanningScene::setCurrentState(const moveit_msgs::RobotState &state)
{
  if (parent_)
  {
    if (!kstate_)
      kstate_.reset(new kinematic_state::KinematicState(parent_->getCurrentState()));
    else
      // attached bodies are sent fully, so we need to clear old ones, if any
      kstate_->clearAttachedBodies();
    kinematic_state::robotStateToKinematicState(*getTransforms(), state, *kstate_);
  }
  else
  {   
    // attached bodies are sent fully, so we need to clear old ones, if any
    kstate_->clearAttachedBodies();
    kinematic_state::robotStateToKinematicState(*ftf_, state, *kstate_);
  }
}

void planning_scene::PlanningScene::setCurrentState(const kinematic_state::KinematicState &state)
{
  if (!kstate_)
    kstate_.reset(new kinematic_state::KinematicState(getKinematicModel()));
  *kstate_ = state;
}

void planning_scene::PlanningScene::decoupleParent(void)
{
  if (!parent_)
    return;
  if (parent_->isConfigured())
  {
    kmodel_ = getKinematicModelNonConst();
    kmodel_const_ = kmodel_;

    if (!ftf_)
    {
      ftf_.reset(new kinematic_state::Transforms(*parent_->getTransforms()));
      ftf_const_ = ftf_;
    }

    if (!kstate_)
      kstate_.reset(new kinematic_state::KinematicState(parent_->getCurrentState()));

    if (!acm_)
      acm_.reset(new collision_detection::AllowedCollisionMatrix(parent_->getAllowedCollisionMatrix()));

    if (!crobot_unpadded_)
    {
      crobot_unpadded_ = collision_detection_allocator_->allocateRobot(parent_->getCollisionRobotUnpadded());
      crobot_unpadded_const_ = crobot_unpadded_;
    }
    if (!crobot_)
    {
      crobot_ = collision_detection_allocator_->allocateRobot(parent_->getCollisionRobot());
      crobot_const_ = crobot_;
    }

    if (!cworld_)
    {
      cworld_ = collision_detection_allocator_->allocateWorld(parent_->getCollisionWorld());
      cworld_const_ = cworld_;
    }
    else
    {
      cworld_->recordChanges(false);
      cworld_->clearChanges();
    }

    if (!colors_)
    {
      std::map<std::string, std_msgs::ColorRGBA> kc;
      parent_->getKnownColors(kc);
      colors_.reset(new std::map<std::string, std_msgs::ColorRGBA>(kc));
    }
    else
    {
      std::map<std::string, std_msgs::ColorRGBA> kc;
      parent_->getKnownColors(kc);
      for (std::map<std::string, std_msgs::ColorRGBA>::const_iterator it = kc.begin() ; it != kc.end() ; ++it)
        if (colors_->find(it->first) == colors_->end())
          (*colors_)[it->first] = it->second;
    }
        
    configured_ = true;
  }
  else
    logError("Decoupling scene from a non-configured parent will cause problems");

  parent_.reset();
}

void planning_scene::PlanningScene::setPlanningSceneDiffMsg(const moveit_msgs::PlanningScene &scene)
{
  logDebug("Adding planning scene diff");
  if (!scene.name.empty())
    name_ = scene.name;
  
  if (!scene.robot_model_name.empty() && scene.robot_model_name != getKinematicModel()->getName())
    logWarn("Setting the scene for model '%s' but model '%s' is loaded.", scene.robot_model_name.c_str(), getKinematicModel()->getName().c_str());
  
  if (!scene.robot_model_root.empty() && scene.robot_model_root != getKinematicModel()->getRootLinkName())
    logWarn("Setting scene with robot model root '%s' but the current planning scene uses link '%s' as root.", scene.robot_model_root.c_str(), getKinematicModel()->getRootLinkName().c_str());
  
  // there is at least one transform in the list of fixed transform: from model frame to itself;
  // if the list is empty, then nothing has been set
  if (!scene.fixed_frame_transforms.empty())
  {
    if (!ftf_)
    {
      ftf_.reset(new kinematic_state::Transforms(getKinematicModel()->getModelFrame()));
      ftf_const_ = ftf_;
    }
    ftf_->setTransforms(scene.fixed_frame_transforms);
  }

  // if at least some joints have been specified, we set them
  if (!scene.robot_state.multi_dof_joint_state.joint_names.empty() ||
      !scene.robot_state.joint_state.name.empty() ||
      !scene.robot_state.attached_collision_objects.empty())
    setCurrentState(scene.robot_state);

  // if at least some links are mentioned in the allowed collision matrix, then we have an update
  if (!scene.allowed_collision_matrix.entry_names.empty())
    acm_.reset(new collision_detection::AllowedCollisionMatrix(scene.allowed_collision_matrix));

  if (!scene.link_padding.empty() || !scene.link_scale.empty())
  {
    if (!crobot_)
    { // this means we have a parent too
      crobot_ = collision_detection_allocator_->allocateRobot(parent_->getCollisionRobot());
      crobot_const_ = crobot_;
    }
    crobot_->setPadding(scene.link_padding);
    crobot_->setScale(scene.link_scale);
  }  
  
  // if any colors have been specified, replace the ones we have with the specified ones
  if (!scene.object_colors.empty())
  {
    colors_.reset();
    for (std::size_t i = 0 ; i < scene.object_colors.size() ; ++i)
      setColor(scene.object_colors[i].id, scene.object_colors[i].color);
  }
  
  // process collision object updates
  for (std::size_t i = 0 ; i < scene.world.collision_objects.size() ; ++i)
    processCollisionObjectMsg(scene.world.collision_objects[i]);

  // if an octomap was specified, replace the one we have with that one
  if (!scene.world.octomap.octomap.data.empty())
    processOctomapMsg(scene.world.octomap);
  
  // if a collision map was specified, replace the one we have with that one
  if (!scene.world.collision_map.boxes.empty())
    processCollisionMapMsg(scene.world.collision_map);
}

void planning_scene::PlanningScene::setPlanningSceneMsg(const moveit_msgs::PlanningScene &scene)
{
  logDebug("Setting new planning scene: '%s'", scene.name.c_str());
  name_ = scene.name;
  
  if (!scene.robot_model_name.empty() && scene.robot_model_name != getKinematicModel()->getName())
    logWarn("Setting the scene for model '%s' but model '%s' is loaded.", scene.robot_model_name.c_str(), getKinematicModel()->getName().c_str());

  if (parent_)
  {
    // if we have a parent, but we set a new planning scene, then we do not care about the parent any more
    // and we no longer represent the scene as a diff
    kmodel_ = getKinematicModelNonConst();
    kmodel_const_ = kmodel_;

    if (!ftf_)
    {
      ftf_.reset(new kinematic_state::Transforms(kmodel_->getModelFrame()));
      ftf_const_ = ftf_;
    }

    if (!kstate_)
      kstate_.reset(new kinematic_state::KinematicState(kmodel_));

    if (!crobot_)
    {
      crobot_ = collision_detection_allocator_->allocateRobot(kmodel_);
      crobot_const_ = crobot_;
    }
    crobot_unpadded_ = collision_detection_allocator_->allocateRobot(kmodel_);
    crobot_unpadded_const_ = crobot_unpadded_;

    cworld_->recordChanges(false);
    cworld_->clearChanges();

    configured_ = true;
    parent_.reset();
  }
  // re-parent the robot model if needed
  if (!scene.robot_model_root.empty() && scene.robot_model_root != getKinematicModel()->getRootLinkName())
    configure(getKinematicModel()->getURDF(), getKinematicModel()->getSRDF(), scene.robot_model_root);
  
  ftf_->setTransforms(scene.fixed_frame_transforms);
  kstate_->clearAttachedBodies();
  setCurrentState(scene.robot_state);
  acm_.reset(new collision_detection::AllowedCollisionMatrix(scene.allowed_collision_matrix));
  crobot_->setPadding(scene.link_padding);
  crobot_->setScale(scene.link_scale);
  colors_.reset(new std::map<std::string, std_msgs::ColorRGBA>());
  for (std::size_t i = 0 ; i < scene.object_colors.size() ; ++i)
    setColor(scene.object_colors[i].id, scene.object_colors[i].color);
  cworld_->clearObjects();
  processPlanningSceneWorldMsg(scene.world);
}

void planning_scene::PlanningScene::processPlanningSceneWorldMsg(const moveit_msgs::PlanningSceneWorld &world)
{ 
  for (std::size_t i = 0 ; i < world.collision_objects.size() ; ++i)
    processCollisionObjectMsg(world.collision_objects[i]);
  processOctomapMsg(world.octomap);
  processCollisionMapMsg(world.collision_map);
}

void planning_scene::PlanningScene::usePlanningSceneMsg(const moveit_msgs::PlanningScene &scene)
{
  if (scene.is_diff)
    setPlanningSceneDiffMsg(scene);
  else
    setPlanningSceneMsg(scene);
}

void planning_scene::PlanningScene::processCollisionMapMsg(const moveit_msgs::CollisionMap &map)
{
  // each collision map replaces any previous one
  cworld_->removeObject(COLLISION_MAP_NS);
  
  if (map.boxes.empty())
    return;
  const Eigen::Affine3d &t = getTransforms()->getTransform(getCurrentState(), map.header.frame_id);
  for (std::size_t i = 0 ; i < map.boxes.size() ; ++i)
  {
    Eigen::Affine3d p;
    tf::poseMsgToEigen(map.boxes[i].pose, p);
    shapes::Shape *s = new shapes::Box(map.boxes[i].extents.x, map.boxes[i].extents.y, map.boxes[i].extents.z);
    cworld_->addToObject(COLLISION_MAP_NS, shapes::ShapeConstPtr(s), t * p);
  }
}

void planning_scene::PlanningScene::processOctomapMsg(const octomap_msgs::Octomap &map)
{
  // each octomap replaces any previous one
  cworld_->removeObject(OCTOMAP_NS);

  if (map.data.empty())
    return;
  
  if (map.id != "OcTree")
  {
    logError("Received ocomap is of type '%s' but type 'OcTree' is expected.", map.id.c_str());
    return;
  }
  
  boost::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map)));
  if (!map.header.frame_id.empty())
  {
    const Eigen::Affine3d &t = getTransforms()->getTransform(getCurrentState(), map.header.frame_id);
    cworld_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), t);
  }
  else
  {
    cworld_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), Eigen::Affine3d::Identity());
  }
}

void planning_scene::PlanningScene::processOctomapMsg(const octomap_msgs::OctomapWithPose &map)
{
  // each octomap replaces any previous one
  cworld_->removeObject(OCTOMAP_NS);

  if (map.octomap.data.empty())
    return;
  
  if (map.octomap.id != "OcTree")
  {
    logError("Received ocomap is of type '%s' but type 'OcTree' is expected.", map.octomap.id.c_str());
    return;
  }

  boost::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map.octomap)));
  const Eigen::Affine3d &t = getTransforms()->getTransform(getCurrentState(), map.header.frame_id);
  Eigen::Affine3d p;
  tf::poseMsgToEigen(map.origin, p);
  p = t * p;
  cworld_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), p);
}

void planning_scene::PlanningScene::processOctomapPtr(const boost::shared_ptr<const octomap::OcTree> &octree, const Eigen::Affine3d &t)
{ 
  if (getCollisionWorld()->hasObject(OCTOMAP_NS))
  {
    collision_detection::CollisionWorld::ObjectConstPtr map = getCollisionWorld()->getObject(OCTOMAP_NS);
    if (map->shapes_.size() == 1)
    {
      // check to see if we have the same octree pointer & pose.
      const shapes::OcTree *o = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
      if (o->octree == octree)
      {
        // if the pose changed, we update it
        if (map->shape_poses_[0].isApprox(t, std::numeric_limits<double>::epsilon() * 100.0))
        {
          cworld_->changeRemoveObject(OCTOMAP_NS);
          cworld_->changeAddObject(OCTOMAP_NS);
        }
        else
        { 
          shapes::ShapeConstPtr shape = map->shapes_[0];
          map.reset(); // reset this pointer first so that caching optimizations can be used in CollisionWorld
          cworld_->moveShapeInObject(OCTOMAP_NS, shape, t);
        }
        return;
      }
    }
  }
  // if the octree pointer changed, update the structure
  cworld_->removeObject(OCTOMAP_NS); 
  cworld_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(octree)), t);
}

bool planning_scene::PlanningScene::processAttachedCollisionObjectMsg(const moveit_msgs::AttachedCollisionObject &object)
{
  if (!getKinematicModel()->hasLinkModel(object.link_name))
  {
    if (object.object.operation == moveit_msgs::CollisionObject::ADD)
      logError("Unable to attach a body to link '%s' (link not found)", object.link_name.c_str());
    else
      logError("Unable to detach body from link '%s' (link not found)", object.link_name.c_str());
    return false;
  }

  if (object.object.id == COLLISION_MAP_NS)
  {
    logError("The ID '%s' cannot be used for collision objects (name reserved)", COLLISION_MAP_NS.c_str());
    return false;
  }

  if (object.object.id == OCTOMAP_NS)
  {
    logError("The ID '%s' cannot be used for collision objects (name reserved)", OCTOMAP_NS.c_str());
    return false;
  }

  if (!kstate_) // there must be a parent in this case
    kstate_.reset(new kinematic_state::KinematicState(parent_->getCurrentState()));

  if (object.object.operation == moveit_msgs::CollisionObject::ADD)
  {
    if (object.object.primitives.size() != object.object.primitive_poses.size())
    {
      logError("Number of primitive shapes does not match number of poses in attached collision object message");
      return false;
    }

    if (object.object.meshes.size() != object.object.mesh_poses.size())
    {
      logError("Number of meshes does not match number of poses in attached collision object message");
      return false;
    }

    if (object.object.planes.size() != object.object.plane_poses.size())
    {
      logError("Number of planes does not match number of poses in attached collision object message");
      return false;
    }

    kinematic_state::LinkState *ls = kstate_->getLinkState(object.link_name);
    if (ls)
    {
      std::vector<shapes::ShapeConstPtr> shapes;
      EigenSTL::vector_Affine3d poses;

      // we need to add some shapes; if the message is empty, maybe the object is already in the world
      if (object.object.primitives.empty() && object.object.meshes.empty() && object.object.planes.empty())
      {
        if (cworld_->hasObject(object.object.id))
        {
          logInform("Attaching world object '%s' to link '%s'", object.object.id.c_str(), object.link_name.c_str());

          // extract the shapes from the world
          collision_detection::CollisionWorld::ObjectConstPtr obj = cworld_->getObject(object.object.id);
          shapes = obj->shapes_;
          poses = obj->shape_poses_;
          // remove the pointer to the objects from the collision world
          cworld_->removeObject(object.object.id);
          
          // need to transform poses to the link frame
          const Eigen::Affine3d &i_t = ls->getGlobalLinkTransform().inverse();
          for (std::size_t i = 0 ; i < poses.size() ; ++i)
            poses[i] = i_t * poses[i];
        }
        else
        {
          logError("Attempting to attach object '%s' to link '%s' but no geometry specified and such an object does not exist in the collision world",
                    object.object.id.c_str(), object.link_name.c_str());
          return false;
        }
      }
      else
      {
        // we clear the world objects with the same name, since we got an update on their geometry
        if (cworld_->hasObject(object.object.id))
        {
          logInform("Removing world object with the same name as newly attached object: '%s'", object.object.id.c_str());
          cworld_->removeObject(object.object.id);
        }

        for (std::size_t i = 0 ; i < object.object.primitives.size() ; ++i)
        {
          shapes::Shape *s = shapes::constructShapeFromMsg(object.object.primitives[i]);
          if (s)
          {
            Eigen::Affine3d p;
            tf::poseMsgToEigen(object.object.primitive_poses[i], p);
            shapes.push_back(shapes::ShapeConstPtr(s));
            poses.push_back(p);
          }
        }       
        for (std::size_t i = 0 ; i < object.object.meshes.size() ; ++i)
        {
          shapes::Shape *s = shapes::constructShapeFromMsg(object.object.meshes[i]);
          if (s)
          {
            Eigen::Affine3d p;
            tf::poseMsgToEigen(object.object.mesh_poses[i], p);
            shapes.push_back(shapes::ShapeConstPtr(s));
            poses.push_back(p);
          }
        }
        for (std::size_t i = 0 ; i < object.object.planes.size() ; ++i)
        {
          shapes::Shape *s = shapes::constructShapeFromMsg(object.object.planes[i]);
          if (s)
          {
            Eigen::Affine3d p;
            tf::poseMsgToEigen(object.object.plane_poses[i], p);
            shapes.push_back(shapes::ShapeConstPtr(s));
            poses.push_back(p);
          }
        }

        // transform poses to link frame
        if (object.object.header.frame_id != object.link_name)
        {
          const Eigen::Affine3d &t = ls->getGlobalLinkTransform().inverse() * getTransforms()->getTransform(*kstate_, object.object.header.frame_id);
          for (std::size_t i = 0 ; i < poses.size() ; ++i)
            poses[i] = t * poses[i];
        }
      }

      if (shapes.empty())
      {
        logError("There is no geometry to attach to link '%s' as part of attached body '%s'", object.link_name.c_str(), object.object.id.c_str());
        return false;
      }

      // there should not exist an attached object with this name
      if (ls->clearAttachedBody(object.object.id))
        logInform("The kinematic state already had an object named '%s' attached to link '%s'. The object was replaced.",
                  object.object.id.c_str(), object.link_name.c_str());
      ls->attachBody(object.object.id, shapes, poses, object.touch_links);      
      logInform("Attached object '%s' to link '%s'", object.object.id.c_str(), object.link_name.c_str());
      return true;
    }
    else
      logError("Kinematic state is not compatible with kinematic model. This could be fatal.");
  }
  else
    if (object.object.operation == moveit_msgs::CollisionObject::REMOVE)
    {
      kinematic_state::LinkState *ls = kstate_->getLinkState(object.link_name);
      if (ls)
      {
        const kinematic_state::AttachedBody *ab = ls->getAttachedBody(object.object.id);
        if (ab)
        {
          std::vector<shapes::ShapeConstPtr> shapes = ab->getShapes();
          EigenSTL::vector_Affine3d poses = ab->getGlobalCollisionBodyTransforms();
          ls->clearAttachedBody(object.object.id);
          
          if (cworld_->hasObject(object.object.id))
            logWarn("The collision world already has an object with the same name as the body about to be detached. NOT adding the detached body '%s' to the collision world.", object.object.id.c_str());
          else
          {
            cworld_->addToObject(object.object.id, shapes, poses);
            logInform("Detached object '%s' from link '%s' and added it back in the collision world", object.object.id.c_str(), object.link_name.c_str());
          }
          
          return true;
        }
        else
          logError("No object named '%s' is attached to link '%s'", object.object.id.c_str(), object.link_name.c_str());
      }
      else
        logError("Kinematic state is not compatible with kinematic model. This could be fatal.");
    }
    else
      logError("Unknown collision object operation: %d", object.object.operation);
  return false;
}

bool planning_scene::PlanningScene::processCollisionObjectMsg(const moveit_msgs::CollisionObject &object)
{
  if (object.id == COLLISION_MAP_NS)
  {
    logError("The ID '%s' cannot be used for collision objects (name reserved)", COLLISION_MAP_NS.c_str());
    return false;
  }
  if (object.id == OCTOMAP_NS)
  {
    logError("The ID '%s' cannot be used for collision objects (name reserved)", OCTOMAP_NS.c_str());
    return false;
  }

  if (object.operation == moveit_msgs::CollisionObject::ADD)
  {
    if (object.primitives.empty() && object.meshes.empty() && object.planes.empty())
    {
      logError("There are no shapes specified in the collision object message");
      return false;
    }
    
    if (object.primitives.size() != object.primitive_poses.size())
    {
      logError("Number of primitive shapes does not match number of poses in collision object message");
      return false;
    }

    if (object.meshes.size() != object.mesh_poses.size())
    {
      logError("Number of meshes does not match number of poses in collision object message");
      return false;
    }

    if (object.planes.size() != object.plane_poses.size())
    {
      logError("Number of planes does not match number of poses in collision object message");
      return false;
    }
    
    const Eigen::Affine3d &t = getTransforms()->getTransform(getCurrentState(), object.header.frame_id);
    for (std::size_t i = 0 ; i < object.primitives.size() ; ++i)
    {
      shapes::Shape *s = shapes::constructShapeFromMsg(object.primitives[i]);
      if (s)
      {
        Eigen::Affine3d p; 
        tf::poseMsgToEigen(object.primitive_poses[i], p);
        cworld_->addToObject(object.id, shapes::ShapeConstPtr(s), t * p);
      }
    }
    for (std::size_t i = 0 ; i < object.meshes.size() ; ++i)
    {
      shapes::Shape *s = shapes::constructShapeFromMsg(object.meshes[i]);
      if (s)
      {
        Eigen::Affine3d p; 
        tf::poseMsgToEigen(object.mesh_poses[i], p);
        cworld_->addToObject(object.id, shapes::ShapeConstPtr(s), t * p);
      }
    }
    for (std::size_t i = 0 ; i < object.planes.size() ; ++i)
    {
      shapes::Shape *s = shapes::constructShapeFromMsg(object.planes[i]);
      if (s)
      {
        Eigen::Affine3d p; 
        tf::poseMsgToEigen(object.plane_poses[i], p);
        cworld_->addToObject(object.id, shapes::ShapeConstPtr(s), t * p);
      }
    }
    return true;
  }
  else
    if (object.operation == moveit_msgs::CollisionObject::REMOVE)
    {
      cworld_->removeObject(object.id);
      return true;
    }
    else
      logError("Unknown collision object operation: %d", object.operation);
  return false;
}

bool planning_scene::PlanningScene::hasColor(const std::string &id) const
{
  if (colors_)
    if (colors_->find(id) != colors_->end())
      return true;
  if (parent_)
    return parent_->hasColor(id);
  return false;
}

const std_msgs::ColorRGBA& planning_scene::PlanningScene::getColor(const std::string &id) const
{
  if (colors_)
  {
    std::map<std::string, std_msgs::ColorRGBA>::const_iterator it = colors_->find(id);
    if (it != colors_->end())
      return it->second;
  }
  if (parent_)
    return parent_->getColor(id);
  static const std_msgs::ColorRGBA empty;
  return empty;
}

void planning_scene::PlanningScene::getKnownColors(std::map<std::string, std_msgs::ColorRGBA> &kc) const
{
  kc.clear();
  if (parent_)
    parent_->getKnownColors(kc);
  if (colors_)
    for (std::map<std::string, std_msgs::ColorRGBA>::const_iterator it = colors_->begin(); it != colors_->end() ; ++it)
      kc[it->first] = it->second;
}

void planning_scene::PlanningScene::setColor(const std::string &id, const std_msgs::ColorRGBA &color)
{
  if (!colors_)
    colors_.reset(new std::map<std::string, std_msgs::ColorRGBA>());
  (*colors_)[id] = color;
}

void planning_scene::PlanningScene::removeColor(const std::string &id)
{
  if (colors_)
    colors_->erase(id);
}

bool planning_scene::PlanningScene::isStateColliding(const moveit_msgs::RobotState &state, const std::string &group, bool verbose) const
{
  kinematic_state::KinematicState s(getCurrentState());
  kinematic_state::robotStateToKinematicState(*getTransforms(), state, s);
  return isStateColliding(s, group, verbose);
}

bool planning_scene::PlanningScene::isStateColliding(const std::string &group, bool verbose) const
{
  return isStateColliding(getCurrentState(), group, verbose);
}

bool planning_scene::PlanningScene::isStateColliding(const kinematic_state::KinematicState &state, const std::string &group, bool verbose) const
{ 
  collision_detection::CollisionRequest req;
  req.verbose = verbose;
  req.group_name = group;
  collision_detection::CollisionResult  res;
  checkCollision(req, res, state);
  return res.collision;
}

bool planning_scene::PlanningScene::isStateFeasible(const moveit_msgs::RobotState &state, bool verbose) const
{
  if (state_feasibility_)
  {
    kinematic_state::KinematicState s(getCurrentState());
    kinematic_state::robotStateToKinematicState(*getTransforms(), state, s);
    return state_feasibility_(s, verbose);
  }
  return true;
}

bool planning_scene::PlanningScene::isStateFeasible(const kinematic_state::KinematicState &state, bool verbose) const
{
  if (state_feasibility_)
    return state_feasibility_(state, verbose);
  return true;
}

bool planning_scene::PlanningScene::isStateConstrained(const moveit_msgs::RobotState &state, const moveit_msgs::Constraints &constr, bool verbose) const
{   
  kinematic_state::KinematicState s(getCurrentState());
  kinematic_state::robotStateToKinematicState(*getTransforms(), state, s);
  return isStateConstrained(s, constr, verbose);
}


bool planning_scene::PlanningScene::isStateConstrained(const kinematic_state::KinematicState &state, const moveit_msgs::Constraints &constr, bool verbose) const
{
  kinematic_constraints::KinematicConstraintSetPtr ks(new kinematic_constraints::KinematicConstraintSet(getKinematicModel(), getTransforms()));
  ks->add(constr);
  if (ks->empty())
    return true;
  else
    return isStateConstrained(state, *ks, verbose);
}

bool planning_scene::PlanningScene::isStateConstrained(const moveit_msgs::RobotState &state, const kinematic_constraints::KinematicConstraintSet &constr, bool verbose) const
{
  kinematic_state::KinematicState s(getCurrentState());
  kinematic_state::robotStateToKinematicState(*getTransforms(), state, s);
  return isStateConstrained(s, constr, verbose);
}

bool planning_scene::PlanningScene::isStateConstrained(const kinematic_state::KinematicState &state,  const kinematic_constraints::KinematicConstraintSet &constr, bool verbose) const
{ 
  return constr.decide(state, verbose).satisfied;
}

bool planning_scene::PlanningScene::isStateValid(const kinematic_state::KinematicState &state, const std::string &group, bool verbose) const
{
  static const moveit_msgs::Constraints emp_constraints;
  return isStateValid(state, emp_constraints, group, verbose);
}

bool planning_scene::PlanningScene::isStateValid(const moveit_msgs::RobotState &state, const std::string &group, bool verbose) const
{
  static const moveit_msgs::Constraints emp_constraints;
  return isStateValid(state, emp_constraints, group, verbose);
}

bool planning_scene::PlanningScene::isStateValid(const moveit_msgs::RobotState &state, const moveit_msgs::Constraints &constr, const std::string &group, bool verbose) const
{
  kinematic_state::KinematicState s(getCurrentState());
  kinematic_state::robotStateToKinematicState(*getTransforms(), state, s);
  return isStateValid(s, constr, group, verbose);
}

bool planning_scene::PlanningScene::isStateValid(const kinematic_state::KinematicState &state, const moveit_msgs::Constraints &constr, const std::string &group, bool verbose) const
{
  if (isStateColliding(state, group, verbose))
    return false;
  if (!isStateFeasible(state, verbose))
    return false;
  return isStateConstrained(state, constr, verbose);
}

bool planning_scene::PlanningScene::isStateValid(const kinematic_state::KinematicState &state, const kinematic_constraints::KinematicConstraintSet &constr, const std::string &group, bool verbose) const
{
  if (isStateColliding(state, group, verbose))
    return false;
  if (!isStateFeasible(state, verbose))
    return false;
  return isStateConstrained(state, constr, verbose);
}

bool planning_scene::PlanningScene::isPathValid(const moveit_msgs::RobotState &start_state, 
                                                const moveit_msgs::RobotTrajectory &trajectory,
                                                const std::string &group, bool verbose,
                                                std::vector<std::size_t> *invalid_index) const
{
  static const moveit_msgs::Constraints emp_constraints;
  static const std::vector<moveit_msgs::Constraints> emp_constraints_vector;
  return isPathValid(start_state, trajectory, emp_constraints, emp_constraints_vector, group, verbose, invalid_index);
}

bool planning_scene::PlanningScene::isPathValid(const moveit_msgs::RobotState &start_state,
                                                const moveit_msgs::RobotTrajectory &trajectory,
                                                const moveit_msgs::Constraints& path_constraints,
                                                const std::string &group, bool verbose,
                                                std::vector<std::size_t> *invalid_index) const
{  
  static const std::vector<moveit_msgs::Constraints> emp_constraints_vector;
  return isPathValid(start_state, trajectory, path_constraints, emp_constraints_vector, group, verbose, invalid_index);
}

bool planning_scene::PlanningScene::isPathValid(const moveit_msgs::RobotState &start_state,
                                                const moveit_msgs::RobotTrajectory &trajectory,
                                                const moveit_msgs::Constraints& path_constraints,
                                                const moveit_msgs::Constraints& goal_constraints,
                                                const std::string &group, bool verbose,
                                                std::vector<std::size_t> *invalid_index) const
{
  std::vector<moveit_msgs::Constraints> goal_constraints_vector(1, goal_constraints);
  return isPathValid(start_state, trajectory, path_constraints, goal_constraints_vector, group, verbose, invalid_index);
}

bool planning_scene::PlanningScene::isPathValid(const moveit_msgs::RobotState &start_state,
                                                const moveit_msgs::RobotTrajectory &trajectory,
                                                const moveit_msgs::Constraints& path_constraints,
                                                const std::vector<moveit_msgs::Constraints>& goal_constraints,
                                                const std::string &group, bool verbose,
                                                std::vector<std::size_t> *invalid_index) const
{  
  kinematic_state::KinematicState start(getCurrentState());
  kinematic_state::robotStateToKinematicState(*getTransforms(), start_state, start);
  return isPathValid(start, trajectory, path_constraints, goal_constraints, group, verbose, invalid_index);
}

bool planning_scene::PlanningScene::isPathValid(const kinematic_state::KinematicState &start,
                                                const moveit_msgs::RobotTrajectory &trajectory,
                                                const std::string &group, bool verbose,
                                                std::vector<std::size_t> *invalid_index) const
{
  static const moveit_msgs::Constraints emp_constraints;
  static const std::vector<moveit_msgs::Constraints> emp_constraints_vector;
  return isPathValid(start, trajectory, emp_constraints, emp_constraints_vector, group, verbose, invalid_index);
}

bool planning_scene::PlanningScene::isPathValid(const kinematic_state::KinematicState &start,
                                                const moveit_msgs::RobotTrajectory &trajectory,
                                                const moveit_msgs::Constraints& path_constraints,
                                                const std::string &group, bool verbose,
                                                std::vector<std::size_t> *invalid_index) const
{ 
  static const std::vector<moveit_msgs::Constraints> emp_constraints_vector;
  return isPathValid(start, trajectory, path_constraints, emp_constraints_vector, group, verbose, invalid_index);
}

bool planning_scene::PlanningScene::isPathValid(const kinematic_state::KinematicState &start,
                                                const moveit_msgs::RobotTrajectory &trajectory,
                                                const moveit_msgs::Constraints& path_constraints,
                                                const moveit_msgs::Constraints& goal_constraints,
                                                const std::string &group, bool verbose,
                                                std::vector<std::size_t> *invalid_index) const
{
  std::vector<moveit_msgs::Constraints> goal_constraints_vector(1, goal_constraints);
  return isPathValid(start, trajectory, path_constraints, goal_constraints_vector, group, verbose, invalid_index);
}

bool planning_scene::PlanningScene::isPathValid(const kinematic_state::KinematicState &start,
                                                const moveit_msgs::RobotTrajectory &trajectory,
                                                const moveit_msgs::Constraints& path_constraints,
                                                const std::vector<moveit_msgs::Constraints>& goal_constraints,
                                                const std::string &group, bool verbose,
                                                std::vector<std::size_t> *invalid_index) const
{
  bool result = true;
  if (invalid_index)
    invalid_index->clear();
  std::size_t state_count = trajectory_processing::trajectoryPointCount(trajectory);
  kinematic_constraints::KinematicConstraintSet ks_p(getKinematicModel(), getTransforms());
  ks_p.add(path_constraints);
  for (std::size_t i = 0 ; i < state_count ; ++i)
  {
    moveit_msgs::RobotState rs;
    trajectory_processing::robotTrajectoryPointToRobotState(trajectory, i, rs);
    kinematic_state::KinematicStatePtr st(new kinematic_state::KinematicState(start));
    kinematic_state::robotStateToKinematicState(*getTransforms(), rs, *st);

    bool this_state_valid = true;
    if (isStateColliding(*st, group, verbose))
      this_state_valid = false;
    if (!isStateFeasible(*st, verbose))
      this_state_valid = false;
    if (!ks_p.empty() && !ks_p.decide(*st, verbose).satisfied)
      this_state_valid = false;
    
    if (!this_state_valid)
    {
      if (invalid_index)
        invalid_index->push_back(i);
      else
        return false;
      result = false;
    }
    
    // check goal for last state
    if (i + 1 == state_count && !goal_constraints.empty())
    {
      bool found = false;
      for (std::size_t k = 0 ; k < goal_constraints.size() ; ++k)
      {
        if (isStateConstrained(*st, goal_constraints[k]))
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        if (verbose)
          logInform("Goal not satisfied");
        if (invalid_index)
          invalid_index->push_back(i);
        result = false;
      }
    }
  }
  return result;
} 

bool planning_scene::PlanningScene::isPathValid(const kinematic_state::KinematicTrajectory &trajectory,
                                                const moveit_msgs::Constraints& path_constraints,
                                                const std::vector<moveit_msgs::Constraints>& goal_constraints,
                                                const std::string &group, bool verbose, std::vector<std::size_t> *invalid_index) const
{
  bool result = true;
  if (invalid_index)
    invalid_index->clear();
  kinematic_constraints::KinematicConstraintSet ks_p(getKinematicModel(), getTransforms());
  ks_p.add(path_constraints);
  for (std::size_t i = 0 ; i < trajectory.size() ; ++i)
  {
    const kinematic_state::KinematicStatePtr &st = trajectory[i];
    
    bool this_state_valid = true;
    if (isStateColliding(*st, group, verbose))
      this_state_valid = false;
    if (!isStateFeasible(*st, verbose))
      this_state_valid = false;
    if (!ks_p.empty() && !ks_p.decide(*st, verbose).satisfied)
      this_state_valid = false;
    
    if (!this_state_valid)
    {
      if (invalid_index)
        invalid_index->push_back(i);
      else
        return false;
      result = false;
    }
    
    // check goal for last state
    if (i + 1 == trajectory.size() && !goal_constraints.empty())
    {
      bool found = false;
      for (std::size_t k = 0 ; k < goal_constraints.size() ; ++k)
      {
        if (isStateConstrained(*st, goal_constraints[k]))
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        if (verbose)
          logInform("Goal not satisfied");
        if (invalid_index)
          invalid_index->push_back(i);
        result = false;
      }
    }
  }
  return result;
}

bool planning_scene::PlanningScene::isPathValid(const kinematic_state::KinematicTrajectory &trajectory,
                                                const moveit_msgs::Constraints& path_constraints,
                                                const moveit_msgs::Constraints& goal_constraints,
                                                const std::string &group, bool verbose, std::vector<std::size_t> *invalid_index) const
{
  std::vector<moveit_msgs::Constraints> goal_constraints_vector(1, goal_constraints);
  return isPathValid(trajectory, path_constraints, goal_constraints_vector, group, verbose, invalid_index);
}


bool planning_scene::PlanningScene::isPathValid(const kinematic_state::KinematicTrajectory &trajectory,
                                                const std::string &group, bool verbose, std::vector<std::size_t> *invalid_index) const
{ 
  static const moveit_msgs::Constraints emp_constraints;
  static const std::vector<moveit_msgs::Constraints> emp_constraints_vector;
  return isPathValid(trajectory, emp_constraints, emp_constraints_vector, group, verbose, invalid_index);
}

void planning_scene::PlanningScene::getCostSources(const kinematic_state::KinematicTrajectory &trajectory, std::size_t max_costs,
                                                   std::set<collision_detection::CostSource> &costs, double overlap_fraction) const
{
  getCostSources(trajectory, max_costs, std::string(), costs, overlap_fraction);
}


void planning_scene::PlanningScene::getCostSources(const kinematic_state::KinematicTrajectory &trajectory, std::size_t max_costs,
                                                   const std::string &group_name, std::set<collision_detection::CostSource> &costs,
                                                   double overlap_fraction) const
{
  collision_detection::CollisionRequest creq;
  creq.max_cost_sources = max_costs;
  creq.group_name = group_name;
  creq.cost = true;
  std::set<collision_detection::CostSource> cs; 
  std::set<collision_detection::CostSource> cs_start;
  for (std::size_t i = 0 ; i < trajectory.size() ; ++i)
  {
    collision_detection::CollisionResult cres;
    checkCollision(creq, cres, *trajectory[i]);
    cs.insert(cres.cost_sources.begin(), cres.cost_sources.end());
    if (i == 0)
      cs_start.swap(cres.cost_sources);
  }
  
  if (cs.size() <= max_costs)
    costs.swap(cs);
  else
  {
    costs.clear();
    std::size_t i = 0;
    for (std::set<collision_detection::CostSource>::iterator it = cs.begin() ; i < max_costs ; ++it, ++i)
      costs.insert(*it);
  }
  
  collision_detection::removeCostSources(costs, cs_start, overlap_fraction);
  collision_detection::removeOverlapping(costs, overlap_fraction);
}

void planning_scene::PlanningScene::getCostSources(const kinematic_state::KinematicState &state, std::size_t max_costs,
                                                   std::set<collision_detection::CostSource> &costs) const
{
  getCostSources(state, max_costs, std::string(), costs);
}

void planning_scene::PlanningScene::getCostSources(const kinematic_state::KinematicState &state, std::size_t max_costs,
                                                   const std::string &group_name, std::set<collision_detection::CostSource> &costs) const
{
  collision_detection::CollisionRequest creq;
  creq.max_cost_sources = max_costs;
  creq.group_name = group_name;
  creq.cost = true;
  collision_detection::CollisionResult cres;
  checkCollision(creq, cres, state);
  cres.cost_sources.swap(costs);
}
