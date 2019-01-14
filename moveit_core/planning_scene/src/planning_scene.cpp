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

/* Author: Ioan Sucan */

#include <boost/algorithm/string.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/exceptions/exceptions.h>
#include <moveit/robot_state/attached_body.h>
#include <octomap_msgs/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <memory>
#include <set>

namespace planning_scene
{
const std::string PlanningScene::OCTOMAP_NS = "<octomap>";
const std::string PlanningScene::DEFAULT_SCENE_NAME = "(noname)";

class SceneTransforms : public robot_state::Transforms
{
public:
  SceneTransforms(const PlanningScene* scene) : Transforms(scene->getRobotModel()->getModelFrame()), scene_(scene)
  {
  }

  bool canTransform(const std::string& from_frame) const override
  {
    return scene_->knowsFrameTransform(from_frame);
  }

  bool isFixedFrame(const std::string& frame) const override
  {
    if (frame.empty())
      return false;
    if (Transforms::isFixedFrame(frame))
      return true;
    if (frame[0] == '/')
      return knowsObject(frame.substr(1));
    else
      return knowsObject(frame);
  }

  const Eigen::Affine3d& getTransform(const std::string& from_frame) const override
  {  // the call below also calls Transforms::getTransform() too
    return scene_->getFrameTransform(from_frame);
  }

private:
  bool knowsObject(const std::string& id) const
  {
    if (scene_->getWorld()->hasObject(id))
    {
      collision_detection::World::ObjectConstPtr obj = scene_->getWorld()->getObject(id);
      return obj->shape_poses_.size() == 1;
    }
    return false;
  }

  const PlanningScene* scene_;
};

bool PlanningScene::isEmpty(const moveit_msgs::PlanningScene& msg)
{
  return msg.name.empty() && msg.fixed_frame_transforms.empty() && msg.allowed_collision_matrix.entry_names.empty() &&
         msg.link_padding.empty() && msg.link_scale.empty() && isEmpty(msg.robot_state) && isEmpty(msg.world);
}

bool PlanningScene::isEmpty(const moveit_msgs::RobotState& msg)
{
  /* a state is empty if it includes no information and it is a diff; if the state is not a diff, then the implicit
     information is
     that the set of attached bodies is empty, so they must be cleared from the state to be updated */
  return static_cast<bool>(msg.is_diff) && msg.multi_dof_joint_state.joint_names.empty() &&
         msg.joint_state.name.empty() && msg.attached_collision_objects.empty() && msg.joint_state.position.empty() &&
         msg.joint_state.velocity.empty() && msg.joint_state.effort.empty() &&
         msg.multi_dof_joint_state.transforms.empty() && msg.multi_dof_joint_state.twist.empty() &&
         msg.multi_dof_joint_state.wrench.empty();
}

bool PlanningScene::isEmpty(const moveit_msgs::PlanningSceneWorld& msg)
{
  return msg.collision_objects.empty() && msg.octomap.octomap.data.empty();
}

PlanningScene::PlanningScene(const robot_model::RobotModelConstPtr& robot_model, collision_detection::WorldPtr world)
  : kmodel_(robot_model), world_(world), world_const_(world)
{
  initialize();
}

PlanningScene::PlanningScene(const urdf::ModelInterfaceSharedPtr& urdf_model,
                             const srdf::ModelConstSharedPtr& srdf_model, collision_detection::WorldPtr world)
  : world_(world), world_const_(world)
{
  if (!urdf_model)
    throw moveit::ConstructException("The URDF model cannot be NULL");

  if (!srdf_model)
    throw moveit::ConstructException("The SRDF model cannot be NULL");

  kmodel_ = createRobotModel(urdf_model, srdf_model);
  if (!kmodel_)
    throw moveit::ConstructException("Could not create RobotModel");

  initialize();
}

PlanningScene::~PlanningScene()
{
  if (current_world_object_update_callback_)
    world_->removeObserver(current_world_object_update_observer_handle_);
}

void PlanningScene::initialize()
{
  name_ = DEFAULT_SCENE_NAME;

  ftf_.reset(new SceneTransforms(this));

  kstate_.reset(new robot_state::RobotState(kmodel_));
  kstate_->setToDefaultValues();

  acm_.reset(new collision_detection::AllowedCollisionMatrix());
  // Use default collision operations in the SRDF to setup the acm
  const std::vector<std::string>& collision_links = kmodel_->getLinkModelNamesWithCollisionGeometry();
  acm_->setEntry(collision_links, collision_links, false);

  // allow collisions for pairs that have been disabled
  const std::vector<srdf::Model::DisabledCollision>& dc = getRobotModel()->getSRDF()->getDisabledCollisionPairs();
  for (std::vector<srdf::Model::DisabledCollision>::const_iterator it = dc.begin(); it != dc.end(); ++it)
    acm_->setEntry(it->link1_, it->link2_, true);

  setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
}

/* return NULL on failure */
robot_model::RobotModelPtr PlanningScene::createRobotModel(const urdf::ModelInterfaceSharedPtr& urdf_model,
                                                           const srdf::ModelConstSharedPtr& srdf_model)
{
  robot_model::RobotModelPtr robot_model(new robot_model::RobotModel(urdf_model, srdf_model));
  if (!robot_model || !robot_model->getRootJoint())
    return robot_model::RobotModelPtr();

  return robot_model;
}

PlanningScene::PlanningScene(const PlanningSceneConstPtr& parent) : parent_(parent)
{
  if (!parent_)
    throw moveit::ConstructException("NULL parent pointer for planning scene");

  if (!parent_->getName().empty())
    name_ = parent_->getName() + "+";

  kmodel_ = parent_->kmodel_;

  // maintain a separate world.  Copy on write ensures that most of the object
  // info is shared until it is modified.
  world_.reset(new collision_detection::World(*parent_->world_));
  world_const_ = world_;

  // record changes to the world
  world_diff_.reset(new collision_detection::WorldDiff(world_));

  // Set up the same collision detectors as the parent
  for (CollisionDetectorConstIterator it = parent_->collision_.begin(); it != parent_->collision_.end(); ++it)
  {
    const CollisionDetectorPtr& parent_detector = it->second;
    CollisionDetectorPtr& detector = collision_[it->first];
    detector.reset(new CollisionDetector());
    detector->alloc_ = parent_detector->alloc_;
    detector->parent_ = parent_detector;

    detector->cworld_ = detector->alloc_->allocateWorld(parent_detector->cworld_, world_);
    detector->cworld_const_ = detector->cworld_;

    // leave these empty and use parent collision_robot_ unless/until a non-const one
    // is requested (e.g. to modify link padding or scale)
    detector->crobot_.reset();
    detector->crobot_const_.reset();
    detector->crobot_unpadded_.reset();
    detector->crobot_unpadded_const_.reset();
  }
  setActiveCollisionDetector(parent_->getActiveCollisionDetectorName());
}

PlanningScenePtr PlanningScene::clone(const PlanningSceneConstPtr& scene)
{
  PlanningScenePtr result = scene->diff();
  result->decoupleParent();
  result->setName(scene->getName());
  return result;
}

PlanningScenePtr PlanningScene::diff() const
{
  return PlanningScenePtr(new PlanningScene(shared_from_this()));
}

PlanningScenePtr PlanningScene::diff(const moveit_msgs::PlanningScene& msg) const
{
  PlanningScenePtr result = diff();
  result->setPlanningSceneDiffMsg(msg);
  return result;
}

void PlanningScene::CollisionDetector::copyPadding(const PlanningScene::CollisionDetector& src)
{
  if (!crobot_)
  {
    alloc_->allocateRobot(parent_->getCollisionRobot());
    crobot_const_ = crobot_;
  }

  crobot_->setLinkPadding(src.getCollisionRobot()->getLinkPadding());
  crobot_->setLinkScale(src.getCollisionRobot()->getLinkScale());
}

void PlanningScene::propogateRobotPadding()
{
  if (!active_collision_->crobot_)
    return;

  for (CollisionDetectorIterator it = collision_.begin(); it != collision_.end(); ++it)
  {
    if (it->second != active_collision_)
      it->second->copyPadding(*active_collision_);
  }
}

void PlanningScene::CollisionDetector::findParent(const PlanningScene& scene)
{
  if (parent_ || !scene.parent_)
    return;

  CollisionDetectorConstIterator it = scene.parent_->collision_.find(alloc_->getName());
  if (it != scene.parent_->collision_.end())
    parent_ = it->second->parent_;
}

void PlanningScene::addCollisionDetector(const collision_detection::CollisionDetectorAllocatorPtr& allocator)
{
  const std::string& name = allocator->getName();
  CollisionDetectorPtr& detector = collision_[name];

  if (detector)  // already added this one
    return;

  detector.reset(new CollisionDetector());

  detector->alloc_ = allocator;

  if (!active_collision_)
    active_collision_ = detector;

  detector->findParent(*this);

  detector->cworld_ = detector->alloc_->allocateWorld(world_);
  detector->cworld_const_ = detector->cworld_;

  // Allocate CollisionRobot unless we can use the parent's crobot_.
  // If active_collision_->crobot_ is non-NULL there is local padding and we cannot use the parent's crobot_.
  if (!detector->parent_ || active_collision_->crobot_)
  {
    detector->crobot_ = detector->alloc_->allocateRobot(getRobotModel());
    detector->crobot_const_ = detector->crobot_;

    if (detector != active_collision_)
      detector->copyPadding(*active_collision_);
  }

  // Allocate CollisionRobot unless we can use the parent's crobot_unpadded_.
  if (!detector->parent_)
  {
    detector->crobot_unpadded_ = detector->alloc_->allocateRobot(getRobotModel());
    detector->crobot_unpadded_const_ = detector->crobot_unpadded_;
  }
}

void PlanningScene::setActiveCollisionDetector(const collision_detection::CollisionDetectorAllocatorPtr& allocator,
                                               bool exclusive)
{
  if (exclusive)
  {
    CollisionDetectorPtr p;
    CollisionDetectorIterator it = collision_.find(allocator->getName());
    if (it != collision_.end())
      p = it->second;

    collision_.clear();
    active_collision_.reset();

    if (p)
    {
      collision_[allocator->getName()] = p;
      active_collision_ = p;
      return;
    }
  }

  addCollisionDetector(allocator);
  setActiveCollisionDetector(allocator->getName());
}

bool PlanningScene::setActiveCollisionDetector(const std::string& collision_detector_name)
{
  CollisionDetectorIterator it = collision_.find(collision_detector_name);
  if (it != collision_.end())
  {
    active_collision_ = it->second;
    return true;
  }
  else
  {
    ROS_ERROR_NAMED("planning_scene",
                    "Cannot setActiveCollisionDetector to '%s' -- it has been added to PlanningScene. "
                    "Keeping existing active collision detector '%s'",
                    collision_detector_name.c_str(), active_collision_->alloc_->getName().c_str());
    return false;
  }
}

void PlanningScene::getCollisionDetectorNames(std::vector<std::string>& names) const
{
  names.clear();
  names.reserve(collision_.size());
  for (CollisionDetectorConstIterator it = collision_.begin(); it != collision_.end(); ++it)
    names.push_back(it->first);
}

const collision_detection::CollisionWorldConstPtr&
PlanningScene::getCollisionWorld(const std::string& collision_detector_name) const
{
  CollisionDetectorConstIterator it = collision_.find(collision_detector_name);
  if (it == collision_.end())
  {
    ROS_ERROR_NAMED("planning_scene",
                    "Could not get CollisionWorld named '%s'.  Returning active CollisionWorld '%s' instead",
                    collision_detector_name.c_str(), active_collision_->alloc_->getName().c_str());
    return active_collision_->cworld_const_;
  }

  return it->second->cworld_const_;
}

const collision_detection::CollisionRobotConstPtr&
PlanningScene::getCollisionRobot(const std::string& collision_detector_name) const
{
  CollisionDetectorConstIterator it = collision_.find(collision_detector_name);
  if (it == collision_.end())
  {
    ROS_ERROR_NAMED("planning_scene",
                    "Could not get CollisionRobot named '%s'.  Returning active CollisionRobot '%s' instead",
                    collision_detector_name.c_str(), active_collision_->alloc_->getName().c_str());
    return active_collision_->getCollisionRobot();
  }

  return it->second->getCollisionRobot();
}

const collision_detection::CollisionRobotConstPtr&
PlanningScene::getCollisionRobotUnpadded(const std::string& collision_detector_name) const
{
  CollisionDetectorConstIterator it = collision_.find(collision_detector_name);
  if (it == collision_.end())
  {
    ROS_ERROR_NAMED("planning_scene", "Could not get CollisionRobotUnpadded named '%s'. "
                                      "Returning active CollisionRobotUnpadded '%s' instead",
                    collision_detector_name.c_str(), active_collision_->alloc_->getName().c_str());
    return active_collision_->getCollisionRobotUnpadded();
  }

  return it->second->getCollisionRobotUnpadded();
}

void PlanningScene::clearDiffs()
{
  if (!parent_)
    return;

  // clear everything, reset the world, record diffs
  world_.reset(new collision_detection::World(*parent_->world_));
  world_const_ = world_;
  world_diff_.reset(new collision_detection::WorldDiff(world_));
  if (current_world_object_update_callback_)
    current_world_object_update_observer_handle_ = world_->addObserver(current_world_object_update_callback_);

  // use parent crobot_ if it exists.  Otherwise copy padding from parent.
  for (CollisionDetectorIterator it = collision_.begin(); it != collision_.end(); ++it)
  {
    if (!it->second->parent_)
      it->second->findParent(*this);

    if (it->second->parent_)
    {
      it->second->crobot_.reset();
      it->second->crobot_const_.reset();
      it->second->crobot_unpadded_.reset();
      it->second->crobot_unpadded_const_.reset();

      it->second->cworld_ = it->second->alloc_->allocateWorld(it->second->parent_->cworld_, world_);
      it->second->cworld_const_ = it->second->cworld_;
    }
    else
    {
      it->second->copyPadding(*parent_->active_collision_);

      it->second->cworld_ = it->second->alloc_->allocateWorld(world_);
      it->second->cworld_const_ = it->second->cworld_;
    }
  }

  ftf_.reset();
  kstate_.reset();
  acm_.reset();
  object_colors_.reset();
  object_types_.reset();
}

void PlanningScene::pushDiffs(const PlanningScenePtr& scene)
{
  if (!parent_)
    return;

  if (ftf_)
    scene->getTransformsNonConst().setAllTransforms(ftf_->getAllTransforms());

  if (kstate_)
  {
    scene->getCurrentStateNonConst() = *kstate_;
    // push colors and types for attached objects
    std::vector<const moveit::core::AttachedBody*> attached_objs;
    kstate_->getAttachedBodies(attached_objs);
    for (std::vector<const moveit::core::AttachedBody*>::const_iterator it = attached_objs.begin();
         it != attached_objs.end(); ++it)
    {
      if (hasObjectType((*it)->getName()))
        scene->setObjectType((*it)->getName(), getObjectType((*it)->getName()));
      if (hasObjectColor((*it)->getName()))
        scene->setObjectColor((*it)->getName(), getObjectColor((*it)->getName()));
    }
  }

  if (acm_)
    scene->getAllowedCollisionMatrixNonConst() = *acm_;

  if (active_collision_->crobot_)
  {
    collision_detection::CollisionRobotPtr active_crobot = scene->getCollisionRobotNonConst();
    active_crobot->setLinkPadding(active_collision_->crobot_->getLinkPadding());
    active_crobot->setLinkScale(active_collision_->crobot_->getLinkScale());
    scene->propogateRobotPadding();
  }

  if (world_diff_)
  {
    for (collision_detection::WorldDiff::const_iterator it = world_diff_->begin(); it != world_diff_->end(); ++it)
    {
      if (it->second == collision_detection::World::DESTROY)
      {
        scene->world_->removeObject(it->first);
        scene->removeObjectColor(it->first);
        scene->removeObjectType(it->first);
      }
      else
      {
        const collision_detection::World::Object& obj = *world_->getObject(it->first);
        scene->world_->removeObject(obj.id_);
        scene->world_->addToObject(obj.id_, obj.shapes_, obj.shape_poses_);
        if (hasObjectColor(it->first))
          scene->setObjectColor(it->first, getObjectColor(it->first));
        if (hasObjectType(it->first))
          scene->setObjectType(it->first, getObjectType(it->first));
      }
    }
  }
}

void PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                   collision_detection::CollisionResult& res)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    checkCollision(req, res, getCurrentStateNonConst());
  else
    checkCollision(req, res, getCurrentState());
}

void PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                   collision_detection::CollisionResult& res,
                                   const robot_state::RobotState& kstate) const
{
  // check collision with the world using the padded version
  getCollisionWorld()->checkRobotCollision(req, res, *getCollisionRobot(), kstate, getAllowedCollisionMatrix());

  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
  {
    // do self-collision checking with the unpadded version of the robot
    getCollisionRobotUnpadded()->checkSelfCollision(req, res, kstate, getAllowedCollisionMatrix());
  }
}

void PlanningScene::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    checkSelfCollision(req, res, getCurrentStateNonConst());
  else
    checkSelfCollision(req, res, getCurrentState());
}

void PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                   collision_detection::CollisionResult& res, const robot_state::RobotState& kstate,
                                   const collision_detection::AllowedCollisionMatrix& acm) const
{
  // check collision with the world using the padded version
  getCollisionWorld()->checkRobotCollision(req, res, *getCollisionRobot(), kstate, acm);

  // do self-collision checking with the unpadded version of the robot
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    getCollisionRobotUnpadded()->checkSelfCollision(req, res, kstate, acm);
}

void PlanningScene::checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                                           collision_detection::CollisionResult& res)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    checkCollisionUnpadded(req, res, getCurrentStateNonConst(), getAllowedCollisionMatrix());
  else
    checkCollisionUnpadded(req, res, getCurrentState(), getAllowedCollisionMatrix());
}

void PlanningScene::checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                                           collision_detection::CollisionResult& res,
                                           const robot_state::RobotState& kstate,
                                           const collision_detection::AllowedCollisionMatrix& acm) const
{
  // check collision with the world using the unpadded version
  getCollisionWorld()->checkRobotCollision(req, res, *getCollisionRobotUnpadded(), kstate, acm);

  // do self-collision checking with the unpadded version of the robot
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
  {
    getCollisionRobotUnpadded()->checkSelfCollision(req, res, kstate, acm);
  }
}

void PlanningScene::getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    getCollidingPairs(contacts, getCurrentStateNonConst(), getAllowedCollisionMatrix());
  else
    getCollidingPairs(contacts, getCurrentState(), getAllowedCollisionMatrix());
}

void PlanningScene::getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts,
                                      const robot_state::RobotState& kstate,
                                      const collision_detection::AllowedCollisionMatrix& acm) const
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = getRobotModel()->getLinkModelsWithCollisionGeometry().size() + 1;
  req.max_contacts_per_pair = 1;
  collision_detection::CollisionResult res;
  checkCollision(req, res, kstate, acm);
  res.contacts.swap(contacts);
}

void PlanningScene::getCollidingLinks(std::vector<std::string>& links)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    getCollidingLinks(links, getCurrentStateNonConst(), getAllowedCollisionMatrix());
  else
    getCollidingLinks(links, getCurrentState(), getAllowedCollisionMatrix());
}

void PlanningScene::getCollidingLinks(std::vector<std::string>& links, const robot_state::RobotState& kstate,
                                      const collision_detection::AllowedCollisionMatrix& acm) const
{
  collision_detection::CollisionResult::ContactMap contacts;
  getCollidingPairs(contacts, kstate, acm);
  links.clear();
  for (collision_detection::CollisionResult::ContactMap::const_iterator it = contacts.begin(); it != contacts.end();
       ++it)
    for (std::size_t j = 0; j < it->second.size(); ++j)
    {
      if (it->second[j].body_type_1 == collision_detection::BodyTypes::ROBOT_LINK)
        links.push_back(it->second[j].body_name_1);
      if (it->second[j].body_type_2 == collision_detection::BodyTypes::ROBOT_LINK)
        links.push_back(it->second[j].body_name_2);
    }
}

const collision_detection::CollisionRobotPtr& PlanningScene::getCollisionRobotNonConst()
{
  if (!active_collision_->crobot_)
  {
    active_collision_->crobot_ =
        active_collision_->alloc_->allocateRobot(active_collision_->parent_->getCollisionRobot());
    active_collision_->crobot_const_ = active_collision_->crobot_;
  }
  return active_collision_->crobot_;
}

robot_state::RobotState& PlanningScene::getCurrentStateNonConst()
{
  if (!kstate_)
  {
    kstate_.reset(new robot_state::RobotState(parent_->getCurrentState()));
    kstate_->setAttachedBodyUpdateCallback(current_state_attached_body_callback_);
  }
  kstate_->update();
  return *kstate_;
}

robot_state::RobotStatePtr PlanningScene::getCurrentStateUpdated(const moveit_msgs::RobotState& update) const
{
  robot_state::RobotStatePtr state(new robot_state::RobotState(getCurrentState()));
  robot_state::robotStateMsgToRobotState(getTransforms(), update, *state);
  return state;
}

void PlanningScene::setAttachedBodyUpdateCallback(const robot_state::AttachedBodyCallback& callback)
{
  current_state_attached_body_callback_ = callback;
  if (kstate_)
    kstate_->setAttachedBodyUpdateCallback(callback);
}

void PlanningScene::setCollisionObjectUpdateCallback(const collision_detection::World::ObserverCallbackFn& callback)
{
  if (current_world_object_update_callback_)
    world_->removeObserver(current_world_object_update_observer_handle_);
  if (callback)
    current_world_object_update_observer_handle_ = world_->addObserver(callback);
  current_world_object_update_callback_ = callback;
}

collision_detection::AllowedCollisionMatrix& PlanningScene::getAllowedCollisionMatrixNonConst()
{
  if (!acm_)
    acm_.reset(new collision_detection::AllowedCollisionMatrix(parent_->getAllowedCollisionMatrix()));
  return *acm_;
}

const robot_state::Transforms& PlanningScene::getTransforms()
{
  getCurrentStateNonConst().update();
  return static_cast<const PlanningScene*>(this)->getTransforms();
}

robot_state::Transforms& PlanningScene::getTransformsNonConst()
{
  getCurrentStateNonConst().update();
  if (!ftf_)
  {
    ftf_.reset(new SceneTransforms(this));
    ftf_->setAllTransforms(parent_->getTransforms().getAllTransforms());
  }
  return *ftf_;
}

void PlanningScene::getPlanningSceneDiffMsg(moveit_msgs::PlanningScene& scene_msg) const
{
  scene_msg.name = name_;
  scene_msg.robot_model_name = getRobotModel()->getName();
  scene_msg.is_diff = true;

  if (ftf_)
    ftf_->copyTransforms(scene_msg.fixed_frame_transforms);
  else
    scene_msg.fixed_frame_transforms.clear();

  if (kstate_)
    robot_state::robotStateToRobotStateMsg(*kstate_, scene_msg.robot_state);
  else
  {
    scene_msg.robot_state = moveit_msgs::RobotState();
    scene_msg.robot_state.is_diff = true;
  }

  if (acm_)
    acm_->getMessage(scene_msg.allowed_collision_matrix);
  else
    scene_msg.allowed_collision_matrix = moveit_msgs::AllowedCollisionMatrix();

  if (active_collision_->crobot_)
  {
    active_collision_->crobot_->getPadding(scene_msg.link_padding);
    active_collision_->crobot_->getScale(scene_msg.link_scale);
  }
  else
  {
    scene_msg.link_padding.clear();
    scene_msg.link_scale.clear();
  }

  scene_msg.object_colors.clear();
  if (object_colors_)
  {
    unsigned int i = 0;
    scene_msg.object_colors.resize(object_colors_->size());
    for (ObjectColorMap::const_iterator it = object_colors_->begin(); it != object_colors_->end(); ++it, ++i)
    {
      scene_msg.object_colors[i].id = it->first;
      scene_msg.object_colors[i].color = it->second;
    }
  }

  scene_msg.world.collision_objects.clear();
  scene_msg.world.octomap = octomap_msgs::OctomapWithPose();

  if (world_diff_)
  {
    bool do_omap = false;
    for (collision_detection::WorldDiff::const_iterator it = world_diff_->begin(); it != world_diff_->end(); ++it)
    {
      if (it->first == OCTOMAP_NS)
        do_omap = true;
      else if (it->second == collision_detection::World::DESTROY)
      {
        moveit_msgs::CollisionObject co;
        co.header.frame_id = getPlanningFrame();
        co.id = it->first;
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        scene_msg.world.collision_objects.push_back(co);
      }
      else
      {
        scene_msg.world.collision_objects.emplace_back();
        getCollisionObjectMsg(scene_msg.world.collision_objects.back(), it->first);
      }
    }
    if (do_omap)
      getOctomapMsg(scene_msg.world.octomap);
  }
}

namespace
{
class ShapeVisitorAddToCollisionObject : public boost::static_visitor<void>
{
public:
  ShapeVisitorAddToCollisionObject(moveit_msgs::CollisionObject* obj) : boost::static_visitor<void>(), obj_(obj)
  {
  }

  void setPoseMessage(const geometry_msgs::Pose* pose)
  {
    pose_ = pose;
  }

  void operator()(const shape_msgs::Plane& shape_msg) const
  {
    obj_->planes.push_back(shape_msg);
    obj_->plane_poses.push_back(*pose_);
  }

  void operator()(const shape_msgs::Mesh& shape_msg) const
  {
    obj_->meshes.push_back(shape_msg);
    obj_->mesh_poses.push_back(*pose_);
  }

  void operator()(const shape_msgs::SolidPrimitive& shape_msg) const
  {
    obj_->primitives.push_back(shape_msg);
    obj_->primitive_poses.push_back(*pose_);
  }

private:
  moveit_msgs::CollisionObject* obj_;
  const geometry_msgs::Pose* pose_;
};
}

bool PlanningScene::getCollisionObjectMsg(moveit_msgs::CollisionObject& collision_obj, const std::string& ns) const
{
  collision_obj.header.frame_id = getPlanningFrame();
  collision_obj.id = ns;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_detection::CollisionWorld::ObjectConstPtr obj = world_->getObject(ns);
  if (!obj)
    return false;
  ShapeVisitorAddToCollisionObject sv(&collision_obj);
  for (std::size_t j = 0; j < obj->shapes_.size(); ++j)
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

  if (!collision_obj.primitives.empty() || !collision_obj.meshes.empty() || !collision_obj.planes.empty())
  {
    if (hasObjectType(collision_obj.id))
      collision_obj.type = getObjectType(collision_obj.id);
  }
  return true;
}

void PlanningScene::getCollisionObjectMsgs(std::vector<moveit_msgs::CollisionObject>& collision_objs) const
{
  collision_objs.clear();
  const std::vector<std::string>& ns = world_->getObjectIds();
  for (std::size_t i = 0; i < ns.size(); ++i)
    if (ns[i] != OCTOMAP_NS)
    {
      collision_objs.emplace_back();
      getCollisionObjectMsg(collision_objs.back(), ns[i]);
    }
}

bool PlanningScene::getAttachedCollisionObjectMsg(moveit_msgs::AttachedCollisionObject& attached_collision_obj,
                                                  const std::string& ns) const
{
  std::vector<moveit_msgs::AttachedCollisionObject> attached_collision_objs;
  getAttachedCollisionObjectMsgs(attached_collision_objs);
  for (std::size_t i = 0; i < attached_collision_objs.size(); ++i)
  {
    if (attached_collision_objs[i].object.id == ns)
    {
      attached_collision_obj = attached_collision_objs[i];
      return true;
    }
  }
  return false;
}

void PlanningScene::getAttachedCollisionObjectMsgs(
    std::vector<moveit_msgs::AttachedCollisionObject>& attached_collision_objs) const
{
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  getCurrentState().getAttachedBodies(attached_bodies);
  attachedBodiesToAttachedCollisionObjectMsgs(attached_bodies, attached_collision_objs);
}

bool PlanningScene::getOctomapMsg(octomap_msgs::OctomapWithPose& octomap) const
{
  octomap.header.frame_id = getPlanningFrame();
  octomap.octomap = octomap_msgs::Octomap();

  collision_detection::CollisionWorld::ObjectConstPtr map = world_->getObject(OCTOMAP_NS);
  if (map)
  {
    if (map->shapes_.size() == 1)
    {
      const shapes::OcTree* o = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
      octomap_msgs::fullMapToMsg(*o->octree, octomap.octomap);
      tf::poseEigenToMsg(map->shape_poses_[0], octomap.origin);
      return true;
    }
    ROS_ERROR_NAMED("planning_scene",
                    "Unexpected number of shapes in octomap collision object. Not including '%s' object",
                    OCTOMAP_NS.c_str());
  }
  return false;
}

void PlanningScene::getObjectColorMsgs(std::vector<moveit_msgs::ObjectColor>& object_colors) const
{
  object_colors.clear();

  unsigned int i = 0;
  ObjectColorMap cmap;
  getKnownObjectColors(cmap);
  object_colors.resize(cmap.size());
  for (ObjectColorMap::const_iterator it = cmap.begin(); it != cmap.end(); ++it, ++i)
  {
    object_colors[i].id = it->first;
    object_colors[i].color = it->second;
  }
}

void PlanningScene::getPlanningSceneMsg(moveit_msgs::PlanningScene& scene_msg) const
{
  scene_msg.name = name_;
  scene_msg.is_diff = false;
  scene_msg.robot_model_name = getRobotModel()->getName();
  getTransforms().copyTransforms(scene_msg.fixed_frame_transforms);

  robot_state::robotStateToRobotStateMsg(getCurrentState(), scene_msg.robot_state);
  getAllowedCollisionMatrix().getMessage(scene_msg.allowed_collision_matrix);
  getCollisionRobot()->getPadding(scene_msg.link_padding);
  getCollisionRobot()->getScale(scene_msg.link_scale);

  getObjectColorMsgs(scene_msg.object_colors);

  // add collision objects
  getCollisionObjectMsgs(scene_msg.world.collision_objects);

  // get the octomap
  getOctomapMsg(scene_msg.world.octomap);
}

void PlanningScene::getPlanningSceneMsg(moveit_msgs::PlanningScene& scene_msg,
                                        const moveit_msgs::PlanningSceneComponents& comp) const
{
  scene_msg.is_diff = false;
  if (comp.components & moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS)
  {
    scene_msg.name = name_;
    scene_msg.robot_model_name = getRobotModel()->getName();
  }

  if (comp.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS)
    getTransforms().copyTransforms(scene_msg.fixed_frame_transforms);

  if (comp.components & moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS)
  {
    robot_state::robotStateToRobotStateMsg(getCurrentState(), scene_msg.robot_state, true);
    for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator it =
             scene_msg.robot_state.attached_collision_objects.begin();
         it != scene_msg.robot_state.attached_collision_objects.end(); ++it)
    {
      if (hasObjectType(it->object.id))
      {
        it->object.type = getObjectType(it->object.id);
      }
    }
  }
  else if (comp.components & moveit_msgs::PlanningSceneComponents::ROBOT_STATE)
  {
    robot_state::robotStateToRobotStateMsg(getCurrentState(), scene_msg.robot_state, false);
  }

  if (comp.components & moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX)
    getAllowedCollisionMatrix().getMessage(scene_msg.allowed_collision_matrix);

  if (comp.components & moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING)
  {
    getCollisionRobot()->getPadding(scene_msg.link_padding);
    getCollisionRobot()->getScale(scene_msg.link_scale);
  }

  if (comp.components & moveit_msgs::PlanningSceneComponents::OBJECT_COLORS)
    getObjectColorMsgs(scene_msg.object_colors);

  // add collision objects
  if (comp.components & moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY)
    getCollisionObjectMsgs(scene_msg.world.collision_objects);
  else if (comp.components & moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES)
  {
    const std::vector<std::string>& ns = world_->getObjectIds();
    scene_msg.world.collision_objects.clear();
    scene_msg.world.collision_objects.reserve(ns.size());
    for (std::size_t i = 0; i < ns.size(); ++i)
      if (ns[i] != OCTOMAP_NS)
      {
        moveit_msgs::CollisionObject co;
        co.id = ns[i];
        if (hasObjectType(co.id))
          co.type = getObjectType(co.id);
        scene_msg.world.collision_objects.push_back(co);
      }
  }

  // get the octomap
  if (comp.components & moveit_msgs::PlanningSceneComponents::OCTOMAP)
    getOctomapMsg(scene_msg.world.octomap);
}

void PlanningScene::saveGeometryToStream(std::ostream& out) const
{
  out << name_ << std::endl;
  const std::vector<std::string>& ns = world_->getObjectIds();
  for (std::size_t i = 0; i < ns.size(); ++i)
    if (ns[i] != OCTOMAP_NS)
    {
      collision_detection::CollisionWorld::ObjectConstPtr obj = world_->getObject(ns[i]);
      if (obj)
      {
        out << "* " << ns[i] << std::endl;
        out << obj->shapes_.size() << std::endl;
        for (std::size_t j = 0; j < obj->shapes_.size(); ++j)
        {
          shapes::saveAsText(obj->shapes_[j].get(), out);
          out << obj->shape_poses_[j].translation().x() << " " << obj->shape_poses_[j].translation().y() << " "
              << obj->shape_poses_[j].translation().z() << std::endl;
          Eigen::Quaterniond r(obj->shape_poses_[j].linear());
          out << r.x() << " " << r.y() << " " << r.z() << " " << r.w() << std::endl;
          if (hasObjectColor(ns[i]))
          {
            const std_msgs::ColorRGBA& c = getObjectColor(ns[i]);
            out << c.r << " " << c.g << " " << c.b << " " << c.a << std::endl;
          }
          else
            out << "0 0 0 0" << std::endl;
        }
      }
    }
  out << "." << std::endl;
}

void PlanningScene::loadGeometryFromStream(std::istream& in)
{
  loadGeometryFromStream(in, Eigen::Affine3d::Identity());  // Use no offset
}

void PlanningScene::loadGeometryFromStream(std::istream& in, const Eigen::Affine3d& offset)
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
      boost::algorithm::trim(ns);
      unsigned int shape_count;
      in >> shape_count;
      for (std::size_t i = 0; i < shape_count && in.good() && !in.eof(); ++i)
      {
        shapes::Shape* s = shapes::constructShapeFromText(in);
        double x, y, z, rx, ry, rz, rw;
        in >> x >> y >> z;
        in >> rx >> ry >> rz >> rw;
        float r, g, b, a;
        in >> r >> g >> b >> a;
        if (s)
        {
          Eigen::Affine3d pose = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(rw, rx, ry, rz);
          // Transform pose by input pose offset
          pose = offset * pose;
          world_->addToObject(ns, shapes::ShapePtr(s), pose);
          if (r > 0.0f || g > 0.0f || b > 0.0f || a > 0.0f)
          {
            std_msgs::ColorRGBA color;
            color.r = r;
            color.g = g;
            color.b = b;
            color.a = a;
            setObjectColor(ns, color);
          }
        }
      }
    }
    else
      break;
  } while (true);
}

void PlanningScene::setCurrentState(const moveit_msgs::RobotState& state)
{
  // The attached bodies will be processed separately by processAttachedCollisionObjectMsgs
  // after kstate_ has been updated
  moveit_msgs::RobotState state_no_attached(state);
  state_no_attached.attached_collision_objects.clear();

  if (parent_)
  {
    if (!kstate_)
    {
      kstate_.reset(new robot_state::RobotState(parent_->getCurrentState()));
      kstate_->setAttachedBodyUpdateCallback(current_state_attached_body_callback_);
    }
    robot_state::robotStateMsgToRobotState(getTransforms(), state_no_attached, *kstate_);
  }
  else
    robot_state::robotStateMsgToRobotState(*ftf_, state_no_attached, *kstate_);

  for (std::size_t i = 0; i < state.attached_collision_objects.size(); ++i)
  {
    if (!state.is_diff && state.attached_collision_objects[i].object.operation != moveit_msgs::CollisionObject::ADD)
    {
      ROS_ERROR_NAMED("planning_scene", "The specified RobotState is not marked as is_diff. "
                                        "The request to modify the object '%s' is not supported. Object is ignored.",
                      state.attached_collision_objects[i].object.id.c_str());
      continue;
    }
    processAttachedCollisionObjectMsg(state.attached_collision_objects[i]);
  }
}

void PlanningScene::setCurrentState(const robot_state::RobotState& state)
{
  getCurrentStateNonConst() = state;
}

void PlanningScene::decoupleParent()
{
  if (!parent_)
    return;

  if (!ftf_)
  {
    ftf_.reset(new SceneTransforms(this));
    ftf_->setAllTransforms(parent_->getTransforms().getAllTransforms());
  }

  if (!kstate_)
  {
    kstate_.reset(new robot_state::RobotState(parent_->getCurrentState()));
    kstate_->setAttachedBodyUpdateCallback(current_state_attached_body_callback_);
  }

  if (!acm_)
    acm_.reset(new collision_detection::AllowedCollisionMatrix(parent_->getAllowedCollisionMatrix()));

  for (CollisionDetectorIterator it = collision_.begin(); it != collision_.end(); ++it)
  {
    if (!it->second->crobot_)
    {
      it->second->crobot_ = it->second->alloc_->allocateRobot(it->second->parent_->getCollisionRobot());
      it->second->crobot_const_ = it->second->crobot_;
    }
    if (!it->second->crobot_unpadded_)
    {
      it->second->crobot_unpadded_ =
          it->second->alloc_->allocateRobot(it->second->parent_->getCollisionRobotUnpadded());
      it->second->crobot_unpadded_const_ = it->second->crobot_unpadded_;
    }
    it->second->parent_.reset();
  }
  world_diff_.reset();

  if (!object_colors_)
  {
    ObjectColorMap kc;
    parent_->getKnownObjectColors(kc);
    object_colors_.reset(new ObjectColorMap(kc));
  }
  else
  {
    ObjectColorMap kc;
    parent_->getKnownObjectColors(kc);
    for (ObjectColorMap::const_iterator it = kc.begin(); it != kc.end(); ++it)
      if (object_colors_->find(it->first) == object_colors_->end())
        (*object_colors_)[it->first] = it->second;
  }

  if (!object_types_)
  {
    ObjectTypeMap kc;
    parent_->getKnownObjectTypes(kc);
    object_types_.reset(new ObjectTypeMap(kc));
  }
  else
  {
    ObjectTypeMap kc;
    parent_->getKnownObjectTypes(kc);
    for (ObjectTypeMap::const_iterator it = kc.begin(); it != kc.end(); ++it)
      if (object_types_->find(it->first) == object_types_->end())
        (*object_types_)[it->first] = it->second;
  }

  parent_.reset();
}

bool PlanningScene::setPlanningSceneDiffMsg(const moveit_msgs::PlanningScene& scene_msg)
{
  bool result = true;

  ROS_DEBUG_NAMED("planning_scene", "Adding planning scene diff");
  if (!scene_msg.name.empty())
    name_ = scene_msg.name;

  if (!scene_msg.robot_model_name.empty() && scene_msg.robot_model_name != getRobotModel()->getName())
    ROS_WARN_NAMED("planning_scene", "Setting the scene for model '%s' but model '%s' is loaded.",
                   scene_msg.robot_model_name.c_str(), getRobotModel()->getName().c_str());

  // there is at least one transform in the list of fixed transform: from model frame to itself;
  // if the list is empty, then nothing has been set
  if (!scene_msg.fixed_frame_transforms.empty())
  {
    if (!ftf_)
      ftf_.reset(new SceneTransforms(this));
    ftf_->setTransforms(scene_msg.fixed_frame_transforms);
  }

  // if at least some joints have been specified, we set them
  if (!scene_msg.robot_state.multi_dof_joint_state.joint_names.empty() ||
      !scene_msg.robot_state.joint_state.name.empty() || !scene_msg.robot_state.attached_collision_objects.empty())
    setCurrentState(scene_msg.robot_state);

  // if at least some links are mentioned in the allowed collision matrix, then we have an update
  if (!scene_msg.allowed_collision_matrix.entry_names.empty())
    acm_.reset(new collision_detection::AllowedCollisionMatrix(scene_msg.allowed_collision_matrix));

  if (!scene_msg.link_padding.empty() || !scene_msg.link_scale.empty())
  {
    for (CollisionDetectorIterator it = collision_.begin(); it != collision_.end(); ++it)
    {
      if (!it->second->crobot_)
      {
        it->second->crobot_ = it->second->alloc_->allocateRobot(it->second->parent_->getCollisionRobot());
        it->second->crobot_const_ = it->second->crobot_;
      }
      it->second->crobot_->setPadding(scene_msg.link_padding);
      it->second->crobot_->setScale(scene_msg.link_scale);
    }
  }

  // if any colors have been specified, replace the ones we have with the specified ones
  for (std::size_t i = 0; i < scene_msg.object_colors.size(); ++i)
    setObjectColor(scene_msg.object_colors[i].id, scene_msg.object_colors[i].color);

  // process collision object updates
  for (std::size_t i = 0; i < scene_msg.world.collision_objects.size(); ++i)
    result &= processCollisionObjectMsg(scene_msg.world.collision_objects[i]);

  // if an octomap was specified, replace the one we have with that one
  if (!scene_msg.world.octomap.octomap.data.empty())
    processOctomapMsg(scene_msg.world.octomap);

  return result;
}

bool PlanningScene::setPlanningSceneMsg(const moveit_msgs::PlanningScene& scene_msg)
{
  ROS_DEBUG_NAMED("planning_scene", "Setting new planning scene: '%s'", scene_msg.name.c_str());
  name_ = scene_msg.name;

  if (!scene_msg.robot_model_name.empty() && scene_msg.robot_model_name != getRobotModel()->getName())
    ROS_WARN_NAMED("planning_scene", "Setting the scene for model '%s' but model '%s' is loaded.",
                   scene_msg.robot_model_name.c_str(), getRobotModel()->getName().c_str());

  if (parent_)
    decoupleParent();

  object_types_.reset();
  ftf_->setTransforms(scene_msg.fixed_frame_transforms);
  setCurrentState(scene_msg.robot_state);
  acm_.reset(new collision_detection::AllowedCollisionMatrix(scene_msg.allowed_collision_matrix));
  for (CollisionDetectorIterator it = collision_.begin(); it != collision_.end(); ++it)
  {
    if (!it->second->crobot_)
    {
      it->second->crobot_ = it->second->alloc_->allocateRobot(it->second->parent_->getCollisionRobot());
      it->second->crobot_const_ = it->second->crobot_;
    }
    it->second->crobot_->setPadding(scene_msg.link_padding);
    it->second->crobot_->setScale(scene_msg.link_scale);
  }
  object_colors_.reset(new ObjectColorMap());
  for (std::size_t i = 0; i < scene_msg.object_colors.size(); ++i)
    setObjectColor(scene_msg.object_colors[i].id, scene_msg.object_colors[i].color);
  world_->clearObjects();
  return processPlanningSceneWorldMsg(scene_msg.world);
}

bool PlanningScene::processPlanningSceneWorldMsg(const moveit_msgs::PlanningSceneWorld& world)
{
  bool result = true;
  for (std::size_t i = 0; i < world.collision_objects.size(); ++i)
    result &= processCollisionObjectMsg(world.collision_objects[i]);
  processOctomapMsg(world.octomap);
  return result;
}

bool PlanningScene::usePlanningSceneMsg(const moveit_msgs::PlanningScene& scene_msg)
{
  if (scene_msg.is_diff)
    return setPlanningSceneDiffMsg(scene_msg);
  else
    return setPlanningSceneMsg(scene_msg);
}

void PlanningScene::processOctomapMsg(const octomap_msgs::Octomap& map)
{
  // each octomap replaces any previous one
  world_->removeObject(OCTOMAP_NS);

  if (map.data.empty())
    return;

  if (map.id != "OcTree")
  {
    ROS_ERROR_NAMED("planning_scene", "Received octomap is of type '%s' but type 'OcTree' is expected.",
                    map.id.c_str());
    return;
  }

  std::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map)));
  if (!map.header.frame_id.empty())
  {
    const Eigen::Affine3d& t = getTransforms().getTransform(map.header.frame_id);
    world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), t);
  }
  else
  {
    world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), Eigen::Affine3d::Identity());
  }
}

void PlanningScene::removeAllCollisionObjects()
{
  const std::vector<std::string>& object_ids = world_->getObjectIds();
  for (std::size_t i = 0; i < object_ids.size(); ++i)
    if (object_ids[i] != OCTOMAP_NS)
    {
      world_->removeObject(object_ids[i]);
      removeObjectColor(object_ids[i]);
      removeObjectType(object_ids[i]);
    }
}

void PlanningScene::processOctomapMsg(const octomap_msgs::OctomapWithPose& map)
{
  // each octomap replaces any previous one
  world_->removeObject(OCTOMAP_NS);

  if (map.octomap.data.empty())
    return;

  if (map.octomap.id != "OcTree")
  {
    ROS_ERROR_NAMED("planning_scene", "Received octomap is of type '%s' but type 'OcTree' is expected.",
                    map.octomap.id.c_str());
    return;
  }

  std::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map.octomap)));
  const Eigen::Affine3d& t = getTransforms().getTransform(map.header.frame_id);
  Eigen::Affine3d p;
  tf::poseMsgToEigen(map.origin, p);
  p = t * p;
  world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), p);
}

void PlanningScene::processOctomapPtr(const std::shared_ptr<const octomap::OcTree>& octree, const Eigen::Affine3d& t)
{
  collision_detection::CollisionWorld::ObjectConstPtr map = world_->getObject(OCTOMAP_NS);
  if (map)
  {
    if (map->shapes_.size() == 1)
    {
      // check to see if we have the same octree pointer & pose.
      const shapes::OcTree* o = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
      if (o->octree == octree)
      {
        // if the pose changed, we update it
        if (map->shape_poses_[0].isApprox(t, std::numeric_limits<double>::epsilon() * 100.0))
        {
          if (world_diff_)
            world_diff_->set(OCTOMAP_NS, collision_detection::World::DESTROY | collision_detection::World::CREATE |
                                             collision_detection::World::ADD_SHAPE);
        }
        else
        {
          shapes::ShapeConstPtr shape = map->shapes_[0];
          map.reset();  // reset this pointer first so that caching optimizations can be used in CollisionWorld
          world_->moveShapeInObject(OCTOMAP_NS, shape, t);
        }
        return;
      }
    }
  }
  // if the octree pointer changed, update the structure
  world_->removeObject(OCTOMAP_NS);
  world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(octree)), t);
}

bool PlanningScene::processAttachedCollisionObjectMsg(const moveit_msgs::AttachedCollisionObject& object)
{
  if (object.object.operation == moveit_msgs::CollisionObject::ADD && !getRobotModel()->hasLinkModel(object.link_name))
  {
    ROS_ERROR_NAMED("planning_scene", "Unable to attach a body to link '%s' (link not found)",
                    object.link_name.c_str());
    return false;
  }

  if (object.object.id == OCTOMAP_NS)
  {
    ROS_ERROR_NAMED("planning_scene", "The ID '%s' cannot be used for collision objects (name reserved)",
                    OCTOMAP_NS.c_str());
    return false;
  }

  if (!kstate_)  // there must be a parent in this case
  {
    kstate_.reset(new robot_state::RobotState(parent_->getCurrentState()));
    kstate_->setAttachedBodyUpdateCallback(current_state_attached_body_callback_);
  }
  kstate_->update();

  if (object.object.operation == moveit_msgs::CollisionObject::ADD ||
      object.object.operation == moveit_msgs::CollisionObject::APPEND)
  {
    if (object.object.primitives.size() != object.object.primitive_poses.size())
    {
      ROS_ERROR_NAMED("planning_scene", "Number of primitive shapes does not match number of poses "
                                        "in attached collision object message");
      return false;
    }

    if (object.object.meshes.size() != object.object.mesh_poses.size())
    {
      ROS_ERROR_NAMED("planning_scene", "Number of meshes does not match number of poses "
                                        "in attached collision object message");
      return false;
    }

    if (object.object.planes.size() != object.object.plane_poses.size())
    {
      ROS_ERROR_NAMED("planning_scene", "Number of planes does not match number of poses "
                                        "in attached collision object message");
      return false;
    }

    const robot_model::LinkModel* lm = getRobotModel()->getLinkModel(object.link_name);
    if (lm)
    {
      std::vector<shapes::ShapeConstPtr> shapes;
      EigenSTL::vector_Affine3d poses;

      // we need to add some shapes; if the message is empty, maybe the object is already in the world
      if (object.object.operation == moveit_msgs::CollisionObject::ADD && object.object.primitives.empty() &&
          object.object.meshes.empty() && object.object.planes.empty())
      {
        collision_detection::CollisionWorld::ObjectConstPtr obj = world_->getObject(object.object.id);
        if (obj)
        {
          ROS_DEBUG_NAMED("planning_scene", "Attaching world object '%s' to link '%s'", object.object.id.c_str(),
                          object.link_name.c_str());

          // extract the shapes from the world
          shapes = obj->shapes_;
          poses = obj->shape_poses_;
          // remove the pointer to the objects from the collision world
          world_->removeObject(object.object.id);

          // need to transform poses to the link frame
          const Eigen::Affine3d& i_t = kstate_->getGlobalLinkTransform(lm).inverse(Eigen::Isometry);
          for (std::size_t i = 0; i < poses.size(); ++i)
            poses[i] = i_t * poses[i];
        }
        else
        {
          ROS_ERROR_NAMED("planning_scene", "Attempting to attach object '%s' to link '%s' but no geometry specified "
                                            "and such an object does not exist in the collision world",
                          object.object.id.c_str(), object.link_name.c_str());
          return false;
        }
      }
      else
      {
        // we clear the world objects with the same name, since we got an update on their geometry
        if (world_->removeObject(object.object.id))
        {
          if (object.object.operation == moveit_msgs::CollisionObject::ADD)
            ROS_DEBUG_NAMED("planning_scene", "Removing world object with the same name as newly attached object: '%s'",
                            object.object.id.c_str());
          else
            ROS_WARN_NAMED("planning_scene",
                           "You tried to append geometry to an attached object that is actually a world object ('%s'). "
                           "World geometry is ignored.",
                           object.object.id.c_str());
        }

        for (std::size_t i = 0; i < object.object.primitives.size(); ++i)
        {
          shapes::Shape* s = shapes::constructShapeFromMsg(object.object.primitives[i]);
          if (s)
          {
            Eigen::Affine3d p;
            tf::poseMsgToEigen(object.object.primitive_poses[i], p);
            shapes.push_back(shapes::ShapeConstPtr(s));
            poses.push_back(p);
          }
        }
        for (std::size_t i = 0; i < object.object.meshes.size(); ++i)
        {
          shapes::Shape* s = shapes::constructShapeFromMsg(object.object.meshes[i]);
          if (s)
          {
            Eigen::Affine3d p;
            tf::poseMsgToEigen(object.object.mesh_poses[i], p);
            shapes.push_back(shapes::ShapeConstPtr(s));
            poses.push_back(p);
          }
        }
        for (std::size_t i = 0; i < object.object.planes.size(); ++i)
        {
          shapes::Shape* s = shapes::constructShapeFromMsg(object.object.planes[i]);
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
          const Eigen::Affine3d& t = kstate_->getGlobalLinkTransform(lm).inverse(Eigen::Isometry) *
                                     getTransforms().getTransform(object.object.header.frame_id);
          for (std::size_t i = 0; i < poses.size(); ++i)
            poses[i] = t * poses[i];
        }
      }

      if (shapes.empty())
      {
        ROS_ERROR_NAMED("planning_scene", "There is no geometry to attach to link '%s' as part of attached body '%s'",
                        object.link_name.c_str(), object.object.id.c_str());
        return false;
      }

      if (!object.object.type.db.empty() || !object.object.type.key.empty())
        setObjectType(object.object.id, object.object.type);

      if (object.object.operation == moveit_msgs::CollisionObject::ADD || !kstate_->hasAttachedBody(object.object.id))
      {
        // there should not exist an attached object with this name
        if (kstate_->clearAttachedBody(object.object.id))
          ROS_DEBUG_NAMED("planning_scene", "The robot state already had an object named '%s' attached to link '%s'. "
                                            "The object was replaced.",
                          object.object.id.c_str(), object.link_name.c_str());
        kstate_->attachBody(object.object.id, shapes, poses, object.touch_links, object.link_name,
                            object.detach_posture);
        ROS_DEBUG_NAMED("planning_scene", "Attached object '%s' to link '%s'", object.object.id.c_str(),
                        object.link_name.c_str());
      }
      else
      {
        const robot_state::AttachedBody* ab = kstate_->getAttachedBody(object.object.id);
        shapes.insert(shapes.end(), ab->getShapes().begin(), ab->getShapes().end());
        poses.insert(poses.end(), ab->getFixedTransforms().begin(), ab->getFixedTransforms().end());
        trajectory_msgs::JointTrajectory detach_posture =
            object.detach_posture.joint_names.empty() ? ab->getDetachPosture() : object.detach_posture;
        std::set<std::string> ab_touch_links = ab->getTouchLinks();
        kstate_->clearAttachedBody(object.object.id);
        if (object.touch_links.empty())
          kstate_->attachBody(object.object.id, shapes, poses, ab_touch_links, object.link_name, detach_posture);
        else
          kstate_->attachBody(object.object.id, shapes, poses, object.touch_links, object.link_name, detach_posture);
        ROS_DEBUG_NAMED("planning_scene", "Added shapes to object '%s' attached to link '%s'", object.object.id.c_str(),
                        object.link_name.c_str());
      }

      return true;
    }
    else
      ROS_ERROR_NAMED("planning_scene", "Robot state is not compatible with robot model. This could be fatal.");
  }
  else if (object.object.operation == moveit_msgs::CollisionObject::REMOVE)
  {
    std::vector<const robot_state::AttachedBody*> attached_bodies;
    if (object.link_name.empty())
    {
      if (object.object.id.empty())
        kstate_->getAttachedBodies(attached_bodies);
      else
      {
        const robot_state::AttachedBody* ab = kstate_->getAttachedBody(object.object.id);
        if (ab)
          attached_bodies.push_back(ab);
      }
    }
    else
    {
      const robot_model::LinkModel* lm = getRobotModel()->getLinkModel(object.link_name);
      if (lm)
      {
        if (object.object.id.empty())  // if no specific object id is given, then we remove all objects attached to the
                                       // link_name
        {
          kstate_->getAttachedBodies(attached_bodies, lm);
        }
        else  // a specific object id will be removed
        {
          const robot_state::AttachedBody* ab = kstate_->getAttachedBody(object.object.id);
          if (ab)
            attached_bodies.push_back(ab);
        }
      }
    }

    for (std::size_t i = 0; i < attached_bodies.size(); ++i)
    {
      std::vector<shapes::ShapeConstPtr> shapes = attached_bodies[i]->getShapes();
      EigenSTL::vector_Affine3d poses = attached_bodies[i]->getGlobalCollisionBodyTransforms();
      std::string name = attached_bodies[i]->getName();

      kstate_->clearAttachedBody(name);

      if (world_->hasObject(name))
        ROS_WARN_NAMED("planning_scene",
                       "The collision world already has an object with the same name as the body about to be detached. "
                       "NOT adding the detached body '%s' to the collision world.",
                       object.object.id.c_str());
      else
      {
        world_->addToObject(name, shapes, poses);
        ROS_DEBUG_NAMED("planning_scene",
                        "Detached object '%s' from link '%s' and added it back in the collision world", name.c_str(),
                        object.link_name.c_str());
      }
    }
    if (!attached_bodies.empty() || object.object.id.empty())
      return true;
  }
  else if (object.object.operation == moveit_msgs::CollisionObject::MOVE)
  {
    ROS_ERROR_NAMED("planning_scene", "Move for attached objects not yet implemented");
  }
  else
  {
    ROS_ERROR_NAMED("planning_scene", "Unknown collision object operation: %d", object.object.operation);
  }

  return false;
}

bool PlanningScene::processCollisionObjectMsg(const moveit_msgs::CollisionObject& object)
{
  if (object.id == OCTOMAP_NS)
  {
    ROS_ERROR_NAMED("planning_scene", "The ID '%s' cannot be used for collision objects (name reserved)",
                    OCTOMAP_NS.c_str());
    return false;
  }

  if (object.operation == moveit_msgs::CollisionObject::ADD || object.operation == moveit_msgs::CollisionObject::APPEND)
  {
    if (object.primitives.empty() && object.meshes.empty() && object.planes.empty())
    {
      ROS_ERROR_NAMED("planning_scene", "There are no shapes specified in the collision object message");
      return false;
    }

    if (object.primitives.size() != object.primitive_poses.size())
    {
      ROS_ERROR_NAMED("planning_scene", "Number of primitive shapes does not match number of poses "
                                        "in collision object message");
      return false;
    }

    if (object.meshes.size() != object.mesh_poses.size())
    {
      ROS_ERROR_NAMED("planning_scene", "Number of meshes does not match number of poses in collision object message");
      return false;
    }

    if (object.planes.size() != object.plane_poses.size())
    {
      ROS_ERROR_NAMED("planning_scene", "Number of planes does not match number of poses in collision object message");
      return false;
    }

    // replace the object if ADD is specified instead of APPEND
    if (object.operation == moveit_msgs::CollisionObject::ADD && world_->hasObject(object.id))
      world_->removeObject(object.id);

    const Eigen::Affine3d& t = getTransforms().getTransform(object.header.frame_id);

    for (std::size_t i = 0; i < object.primitives.size(); ++i)
    {
      shapes::Shape* s = shapes::constructShapeFromMsg(object.primitives[i]);
      if (s)
      {
        Eigen::Affine3d p;
        tf::poseMsgToEigen(object.primitive_poses[i], p);
        world_->addToObject(object.id, shapes::ShapeConstPtr(s), t * p);
      }
    }
    for (std::size_t i = 0; i < object.meshes.size(); ++i)
    {
      shapes::Shape* s = shapes::constructShapeFromMsg(object.meshes[i]);
      if (s)
      {
        Eigen::Affine3d p;
        tf::poseMsgToEigen(object.mesh_poses[i], p);
        world_->addToObject(object.id, shapes::ShapeConstPtr(s), t * p);
      }
    }
    for (std::size_t i = 0; i < object.planes.size(); ++i)
    {
      shapes::Shape* s = shapes::constructShapeFromMsg(object.planes[i]);
      if (s)
      {
        Eigen::Affine3d p;
        tf::poseMsgToEigen(object.plane_poses[i], p);
        world_->addToObject(object.id, shapes::ShapeConstPtr(s), t * p);
      }
    }
    if (!object.type.key.empty() || !object.type.db.empty())
      setObjectType(object.id, object.type);
    return true;
  }
  else if (object.operation == moveit_msgs::CollisionObject::REMOVE)
  {
    if (object.id.empty())
    {
      removeAllCollisionObjects();
    }
    else
    {
      world_->removeObject(object.id);
      removeObjectColor(object.id);
      removeObjectType(object.id);
    }
    return true;
  }
  else if (object.operation == moveit_msgs::CollisionObject::MOVE)
  {
    if (world_->hasObject(object.id))
    {
      if (!object.primitives.empty() || !object.meshes.empty() || !object.planes.empty())
        ROS_WARN_NAMED("planning_scene",
                       "Move operation for object '%s' ignores the geometry specified in the message.",
                       object.id.c_str());

      const Eigen::Affine3d& t = getTransforms().getTransform(object.header.frame_id);
      EigenSTL::vector_Affine3d new_poses;
      for (std::size_t i = 0; i < object.primitive_poses.size(); ++i)
      {
        Eigen::Affine3d p;
        tf::poseMsgToEigen(object.primitive_poses[i], p);
        new_poses.push_back(t * p);
      }
      for (std::size_t i = 0; i < object.mesh_poses.size(); ++i)
      {
        Eigen::Affine3d p;
        tf::poseMsgToEigen(object.mesh_poses[i], p);
        new_poses.push_back(t * p);
      }
      for (std::size_t i = 0; i < object.plane_poses.size(); ++i)
      {
        Eigen::Affine3d p;
        tf::poseMsgToEigen(object.plane_poses[i], p);
        new_poses.push_back(t * p);
      }

      collision_detection::World::ObjectConstPtr obj = world_->getObject(object.id);
      if (obj->shapes_.size() == new_poses.size())
      {
        std::vector<shapes::ShapeConstPtr> shapes = obj->shapes_;
        obj.reset();
        world_->removeObject(object.id);
        world_->addToObject(object.id, shapes, new_poses);
      }
      else
      {
        ROS_ERROR_NAMED("planning_scene",
                        "Number of supplied poses (%zu) for object '%s' does not match number of shapes (%zu). "
                        "Not moving.",
                        new_poses.size(), object.id.c_str(), obj->shapes_.size());
        return false;
      }
      return true;
    }
    else
      ROS_ERROR_NAMED("planning_scene", "World object '%s' does not exist. Cannot move.", object.id.c_str());
  }
  else
    ROS_ERROR_NAMED("planning_scene", "Unknown collision object operation: %d", object.operation);
  return false;
}

const Eigen::Affine3d& PlanningScene::getFrameTransform(const std::string& id) const
{
  return getFrameTransform(getCurrentState(), id);
}

const Eigen::Affine3d& PlanningScene::getFrameTransform(const std::string& id)
{
  if (getCurrentState().dirtyLinkTransforms())
    return getFrameTransform(getCurrentStateNonConst(), id);
  else
    return getFrameTransform(getCurrentState(), id);
}

const Eigen::Affine3d& PlanningScene::getFrameTransform(const robot_state::RobotState& state,
                                                        const std::string& id) const
{
  if (!id.empty() && id[0] == '/')
    return getFrameTransform(id.substr(1));
  if (state.knowsFrameTransform(id))
    return state.getFrameTransform(id);
  if (getWorld()->hasObject(id))
  {
    collision_detection::World::ObjectConstPtr obj = getWorld()->getObject(id);
    if (obj->shape_poses_.size() > 1)
    {
      ROS_WARN_NAMED("planning_scene", "More than one shapes in object '%s'. Using first one to decide transform",
                     id.c_str());
      return obj->shape_poses_[0];
    }
    else if (obj->shape_poses_.size() == 1)
      return obj->shape_poses_[0];
  }
  return getTransforms().Transforms::getTransform(id);
}

bool PlanningScene::knowsFrameTransform(const std::string& id) const
{
  return knowsFrameTransform(getCurrentState(), id);
}

bool PlanningScene::knowsFrameTransform(const robot_state::RobotState& state, const std::string& id) const
{
  if (!id.empty() && id[0] == '/')
    return knowsFrameTransform(id.substr(1));
  if (state.knowsFrameTransform(id))
    return true;

  collision_detection::World::ObjectConstPtr obj = getWorld()->getObject(id);
  if (obj)
  {
    return obj->shape_poses_.size() == 1;
  }
  return getTransforms().Transforms::canTransform(id);
}

bool PlanningScene::hasObjectType(const std::string& id) const
{
  if (object_types_)
    if (object_types_->find(id) != object_types_->end())
      return true;
  if (parent_)
    return parent_->hasObjectType(id);
  return false;
}

const object_recognition_msgs::ObjectType& PlanningScene::getObjectType(const std::string& id) const
{
  if (object_types_)
  {
    ObjectTypeMap::const_iterator it = object_types_->find(id);
    if (it != object_types_->end())
      return it->second;
  }
  if (parent_)
    return parent_->getObjectType(id);
  static const object_recognition_msgs::ObjectType empty;
  return empty;
}

void PlanningScene::setObjectType(const std::string& id, const object_recognition_msgs::ObjectType& type)
{
  if (!object_types_)
    object_types_.reset(new ObjectTypeMap());
  (*object_types_)[id] = type;
}

void PlanningScene::removeObjectType(const std::string& id)
{
  if (object_types_)
    object_types_->erase(id);
}

void PlanningScene::getKnownObjectTypes(ObjectTypeMap& kc) const
{
  kc.clear();
  if (parent_)
    parent_->getKnownObjectTypes(kc);
  if (object_types_)
    for (ObjectTypeMap::const_iterator it = object_types_->begin(); it != object_types_->end(); ++it)
      kc[it->first] = it->second;
}

bool PlanningScene::hasObjectColor(const std::string& id) const
{
  if (object_colors_)
    if (object_colors_->find(id) != object_colors_->end())
      return true;
  if (parent_)
    return parent_->hasObjectColor(id);
  return false;
}

const std_msgs::ColorRGBA& PlanningScene::getObjectColor(const std::string& id) const
{
  if (object_colors_)
  {
    ObjectColorMap::const_iterator it = object_colors_->find(id);
    if (it != object_colors_->end())
      return it->second;
  }
  if (parent_)
    return parent_->getObjectColor(id);
  static const std_msgs::ColorRGBA empty;
  return empty;
}

void PlanningScene::getKnownObjectColors(ObjectColorMap& kc) const
{
  kc.clear();
  if (parent_)
    parent_->getKnownObjectColors(kc);
  if (object_colors_)
    for (ObjectColorMap::const_iterator it = object_colors_->begin(); it != object_colors_->end(); ++it)
      kc[it->first] = it->second;
}

void PlanningScene::setObjectColor(const std::string& id, const std_msgs::ColorRGBA& color)
{
  if (id.empty())
  {
    ROS_ERROR_NAMED("planning_scene", "Cannot set color of object with empty id.");
    return;
  }
  if (!object_colors_)
    object_colors_.reset(new ObjectColorMap());
  (*object_colors_)[id] = color;
}

void PlanningScene::removeObjectColor(const std::string& id)
{
  if (object_colors_)
    object_colors_->erase(id);
}

bool PlanningScene::isStateColliding(const moveit_msgs::RobotState& state, const std::string& group, bool verbose) const
{
  robot_state::RobotState s(getCurrentState());
  robot_state::robotStateMsgToRobotState(getTransforms(), state, s);
  return isStateColliding(s, group, verbose);
}

bool PlanningScene::isStateColliding(const std::string& group, bool verbose)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    return isStateColliding(getCurrentStateNonConst(), group, verbose);
  else
    return isStateColliding(getCurrentState(), group, verbose);
}

bool PlanningScene::isStateColliding(const robot_state::RobotState& state, const std::string& group, bool verbose) const
{
  collision_detection::CollisionRequest req;
  req.verbose = verbose;
  req.group_name = group;
  collision_detection::CollisionResult res;
  checkCollision(req, res, state);
  return res.collision;
}

bool PlanningScene::isStateFeasible(const moveit_msgs::RobotState& state, bool verbose) const
{
  if (state_feasibility_)
  {
    robot_state::RobotState s(getCurrentState());
    robot_state::robotStateMsgToRobotState(getTransforms(), state, s);
    return state_feasibility_(s, verbose);
  }
  return true;
}

bool PlanningScene::isStateFeasible(const robot_state::RobotState& state, bool verbose) const
{
  if (state_feasibility_)
    return state_feasibility_(state, verbose);
  return true;
}

bool PlanningScene::isStateConstrained(const moveit_msgs::RobotState& state, const moveit_msgs::Constraints& constr,
                                       bool verbose) const
{
  robot_state::RobotState s(getCurrentState());
  robot_state::robotStateMsgToRobotState(getTransforms(), state, s);
  return isStateConstrained(s, constr, verbose);
}

bool PlanningScene::isStateConstrained(const robot_state::RobotState& state, const moveit_msgs::Constraints& constr,
                                       bool verbose) const
{
  kinematic_constraints::KinematicConstraintSetPtr ks(
      new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
  ks->add(constr, getTransforms());
  if (ks->empty())
    return true;
  else
    return isStateConstrained(state, *ks, verbose);
}

bool PlanningScene::isStateConstrained(const moveit_msgs::RobotState& state,
                                       const kinematic_constraints::KinematicConstraintSet& constr, bool verbose) const
{
  robot_state::RobotState s(getCurrentState());
  robot_state::robotStateMsgToRobotState(getTransforms(), state, s);
  return isStateConstrained(s, constr, verbose);
}

bool PlanningScene::isStateConstrained(const robot_state::RobotState& state,
                                       const kinematic_constraints::KinematicConstraintSet& constr, bool verbose) const
{
  return constr.decide(state, verbose).satisfied;
}

bool PlanningScene::isStateValid(const robot_state::RobotState& state, const std::string& group, bool verbose) const
{
  static const moveit_msgs::Constraints emp_constraints;
  return isStateValid(state, emp_constraints, group, verbose);
}

bool PlanningScene::isStateValid(const moveit_msgs::RobotState& state, const std::string& group, bool verbose) const
{
  static const moveit_msgs::Constraints emp_constraints;
  return isStateValid(state, emp_constraints, group, verbose);
}

bool PlanningScene::isStateValid(const moveit_msgs::RobotState& state, const moveit_msgs::Constraints& constr,
                                 const std::string& group, bool verbose) const
{
  robot_state::RobotState s(getCurrentState());
  robot_state::robotStateMsgToRobotState(getTransforms(), state, s);
  return isStateValid(s, constr, group, verbose);
}

bool PlanningScene::isStateValid(const robot_state::RobotState& state, const moveit_msgs::Constraints& constr,
                                 const std::string& group, bool verbose) const
{
  if (isStateColliding(state, group, verbose))
    return false;
  if (!isStateFeasible(state, verbose))
    return false;
  return isStateConstrained(state, constr, verbose);
}

bool PlanningScene::isStateValid(const robot_state::RobotState& state,
                                 const kinematic_constraints::KinematicConstraintSet& constr, const std::string& group,
                                 bool verbose) const
{
  if (isStateColliding(state, group, verbose))
    return false;
  if (!isStateFeasible(state, verbose))
    return false;
  return isStateConstrained(state, constr, verbose);
}

bool PlanningScene::isPathValid(const moveit_msgs::RobotState& start_state,
                                const moveit_msgs::RobotTrajectory& trajectory, const std::string& group, bool verbose,
                                std::vector<std::size_t>* invalid_index) const
{
  static const moveit_msgs::Constraints emp_constraints;
  static const std::vector<moveit_msgs::Constraints> emp_constraints_vector;
  return isPathValid(start_state, trajectory, emp_constraints, emp_constraints_vector, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const moveit_msgs::RobotState& start_state,
                                const moveit_msgs::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  static const std::vector<moveit_msgs::Constraints> emp_constraints_vector;
  return isPathValid(start_state, trajectory, path_constraints, emp_constraints_vector, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const moveit_msgs::RobotState& start_state,
                                const moveit_msgs::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints,
                                const moveit_msgs::Constraints& goal_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  std::vector<moveit_msgs::Constraints> goal_constraints_vector(1, goal_constraints);
  return isPathValid(start_state, trajectory, path_constraints, goal_constraints_vector, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const moveit_msgs::RobotState& start_state,
                                const moveit_msgs::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints,
                                const std::vector<moveit_msgs::Constraints>& goal_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  robot_trajectory::RobotTrajectory t(getRobotModel(), group);
  robot_state::RobotState start(getCurrentState());
  robot_state::robotStateMsgToRobotState(getTransforms(), start_state, start);
  t.setRobotTrajectoryMsg(start, trajectory);
  return isPathValid(t, path_constraints, goal_constraints, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints,
                                const std::vector<moveit_msgs::Constraints>& goal_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  bool result = true;
  if (invalid_index)
    invalid_index->clear();
  kinematic_constraints::KinematicConstraintSet ks_p(getRobotModel());
  ks_p.add(path_constraints, getTransforms());
  std::size_t n_wp = trajectory.getWayPointCount();
  for (std::size_t i = 0; i < n_wp; ++i)
  {
    const robot_state::RobotState& st = trajectory.getWayPoint(i);

    bool this_state_valid = true;
    if (isStateColliding(st, group, verbose))
      this_state_valid = false;
    if (!isStateFeasible(st, verbose))
      this_state_valid = false;
    if (!ks_p.empty() && !ks_p.decide(st, verbose).satisfied)
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
    if (i + 1 == n_wp && !goal_constraints.empty())
    {
      bool found = false;
      for (std::size_t k = 0; k < goal_constraints.size(); ++k)
      {
        if (isStateConstrained(st, goal_constraints[k]))
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        if (verbose)
          ROS_INFO_NAMED("planning_scene", "Goal not satisfied");
        if (invalid_index)
          invalid_index->push_back(i);
        result = false;
      }
    }
  }
  return result;
}

bool PlanningScene::isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints,
                                const moveit_msgs::Constraints& goal_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  std::vector<moveit_msgs::Constraints> goal_constraints_vector(1, goal_constraints);
  return isPathValid(trajectory, path_constraints, goal_constraints_vector, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  static const std::vector<moveit_msgs::Constraints> emp_constraints_vector;
  return isPathValid(trajectory, path_constraints, emp_constraints_vector, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const robot_trajectory::RobotTrajectory& trajectory, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  static const moveit_msgs::Constraints emp_constraints;
  static const std::vector<moveit_msgs::Constraints> emp_constraints_vector;
  return isPathValid(trajectory, emp_constraints, emp_constraints_vector, group, verbose, invalid_index);
}

void PlanningScene::getCostSources(const robot_trajectory::RobotTrajectory& trajectory, std::size_t max_costs,
                                   std::set<collision_detection::CostSource>& costs, double overlap_fraction) const
{
  getCostSources(trajectory, max_costs, std::string(), costs, overlap_fraction);
}

void PlanningScene::getCostSources(const robot_trajectory::RobotTrajectory& trajectory, std::size_t max_costs,
                                   const std::string& group_name, std::set<collision_detection::CostSource>& costs,
                                   double overlap_fraction) const
{
  collision_detection::CollisionRequest creq;
  creq.max_cost_sources = max_costs;
  creq.group_name = group_name;
  creq.cost = true;
  std::set<collision_detection::CostSource> cs;
  std::set<collision_detection::CostSource> cs_start;
  std::size_t n_wp = trajectory.getWayPointCount();
  for (std::size_t i = 0; i < n_wp; ++i)
  {
    collision_detection::CollisionResult cres;
    checkCollision(creq, cres, trajectory.getWayPoint(i));
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
    for (std::set<collision_detection::CostSource>::iterator it = cs.begin(); i < max_costs; ++it, ++i)
      costs.insert(*it);
  }

  collision_detection::removeCostSources(costs, cs_start, overlap_fraction);
  collision_detection::removeOverlapping(costs, overlap_fraction);
}

void PlanningScene::getCostSources(const robot_state::RobotState& state, std::size_t max_costs,
                                   std::set<collision_detection::CostSource>& costs) const
{
  getCostSources(state, max_costs, std::string(), costs);
}

void PlanningScene::getCostSources(const robot_state::RobotState& state, std::size_t max_costs,
                                   const std::string& group_name,
                                   std::set<collision_detection::CostSource>& costs) const
{
  collision_detection::CollisionRequest creq;
  creq.max_cost_sources = max_costs;
  creq.group_name = group_name;
  creq.cost = true;
  collision_detection::CollisionResult cres;
  checkCollision(creq, cres, state);
  cres.cost_sources.swap(costs);
}

void PlanningScene::printKnownObjects(std::ostream& out) const
{
  const std::vector<std::string>& objects = getWorld()->getObjectIds();

  out << "Collision World Objects:\n\t ";
  std::copy(objects.begin(), objects.end(), std::ostream_iterator<std::string>(out, "\n\t "));

  std::vector<const robot_state::AttachedBody*> attached_bodies;
  getCurrentState().getAttachedBodies(attached_bodies);

  out << "\nAttached Bodies:\n";
  for (std::size_t i = 0; i < attached_bodies.size(); ++i)
  {
    out << "\t " << attached_bodies[i]->getName() << "\n";
  }
}

}  // end of namespace planning_scene
