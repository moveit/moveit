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

#include "planning_scene/planning_scene.h"
#include <collision_detection/allvalid/collision_world.h>
#include <collision_detection/allvalid/collision_robot.h>
#include <geometric_shapes/shape_operations.h>
#include <planning_models/conversions.h>

namespace planning_scene
{
    typedef collision_detection::CollisionWorldAllValid DefaultCWorldType;
    typedef collision_detection::CollisionRobotAllValid DefaultCRobotType;
    static const std::string COLLISION_MAP_NS = "__map";
}

planning_scene::PlanningScene::PlanningScene(void) : configured_(false)
{
}

planning_scene::PlanningScene::PlanningScene(const PlanningSceneConstPtr &parent) : parent_(parent), configured_(false)
{
    if (parent_->isConfigured())
        configure(parent_->getUrdfModel(), parent_->getSrdfModel());
}

bool planning_scene::PlanningScene::configure(const boost::shared_ptr<const urdf::Model> &urdf_model,
                                              const boost::shared_ptr<const srdf::Model> &srdf_model)
{
    if (!parent_)
    {
        urdf_model_ = urdf_model;
        srdf_model_ = srdf_model;

        kmodel_.reset(new planning_models::KinematicModel(urdf_model, srdf_model));
        kmodel_const_ = kmodel_;
        ftf_.reset(new planning_models::Transforms(kmodel_->getModelFrame()));
        ftf_const_ = ftf_;

        kstate_.reset(new planning_models::KinematicState(kmodel_));
        kstate_->setDefaultValues();
        acm_.reset(new collision_detection::AllowedCollisionMatrix());

        crobot_.reset(new DefaultCRobotType(kmodel_));
        crobot_unpadded_.reset(new DefaultCRobotType(kmodel_));
        crobot_const_ = crobot_;

        cworld_.reset(new DefaultCWorldType());
        cworld_const_ = cworld_;

        configured_ = true;
    }
    else
    {
        if (parent_->isConfigured())
        {
            if (srdf_model != parent_->getSrdfModel() || urdf_model != parent_->getUrdfModel())
                ROS_ERROR("Parent of planning scene is not constructed from the same robot models");

            // even if we have a parent, we do maintain a separate world representation, one that records changes
            // this is cheap however, because the worlds share the world representation
            cworld_.reset(new DefaultCWorldType(static_cast<const DefaultCWorldType&>(*parent_->getCollisionWorld())));
            cworld_->recordChanges(true);
            cworld_const_ = cworld_;
            configured_ = true;
        }
        else
            ROS_ERROR("Parent is not configured yet");
    }

    return true;
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
                                                   const planning_models::KinematicState &kstate) const
{
    // do self-collision checking with the unpadded version of the robot
    if (parent_)
        parent_->crobot_unpadded_->checkSelfCollision(req, res, kstate, getAllowedCollisionMatrix());
    else
        crobot_unpadded_->checkSelfCollision(req, res, kstate, *acm_);

    // check collision with the world using the padded version
    if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    {
        if (parent_)
            getCollisionWorld()->checkRobotCollision(req, res, *getCollisionRobot(), kstate, getAllowedCollisionMatrix());
        else
            cworld_->checkRobotCollision(req, res, *crobot_, kstate, *acm_);
    }
}

void planning_scene::PlanningScene::checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult &res,
                                                       const planning_models::KinematicState &kstate) const
{
    // do self-collision checking with the unpadded version of the robot
    if (parent_)
        parent_->crobot_unpadded_->checkSelfCollision(req, res, kstate, getAllowedCollisionMatrix());
    else
        crobot_unpadded_->checkSelfCollision(req, res, kstate, *acm_);
}

void planning_scene::PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                                   collision_detection::CollisionResult &res,
                                                   const planning_models::KinematicState &kstate,
                                                   const collision_detection::AllowedCollisionMatrix& acm) const
{
    // do self-collision checking with the unpadded version of the robot
    if (parent_)
        parent_->crobot_unpadded_->checkSelfCollision(req, res, kstate, acm);
    else
        crobot_unpadded_->checkSelfCollision(req, res, kstate, acm);

    // check collision with the world using the padded version
    if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    {
        if (parent_)
            getCollisionWorld()->checkRobotCollision(req, res, *getCollisionRobot(), kstate, acm);
        else
            cworld_->checkRobotCollision(req, res, *crobot_, kstate, acm);
    }
}

void planning_scene::PlanningScene::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                       collision_detection::CollisionResult &res,
                                                       const planning_models::KinematicState &kstate,
                                                       const collision_detection::AllowedCollisionMatrix& acm) const
{
    // do self-collision checking with the unpadded version of the robot
    if (parent_)
        parent_->crobot_unpadded_->checkSelfCollision(req, res, kstate, acm);
    else
        crobot_unpadded_->checkSelfCollision(req, res, kstate, acm);
}

const collision_detection::CollisionRobotPtr& planning_scene::PlanningScene::getCollisionRobot(void)
{
    if (!crobot_)
    {
        crobot_.reset(new DefaultCRobotType(static_cast<const DefaultCRobotType&>(*parent_->getCollisionRobot())));
        crobot_const_ = crobot_;
    }
    return crobot_;
}

planning_models::KinematicState& planning_scene::PlanningScene::getCurrentState(void)
{
    if (!kstate_)
        kstate_.reset(new planning_models::KinematicState(parent_->getCurrentState()));
    return *kstate_;
}

collision_detection::AllowedCollisionMatrix& planning_scene::PlanningScene::getAllowedCollisionMatrix(void)
{
    if (!acm_)
        acm_.reset(new collision_detection::AllowedCollisionMatrix(parent_->getAllowedCollisionMatrix()));
    return *acm_;
}

const planning_models::TransformsPtr& planning_scene::PlanningScene::getTransforms(void)
{
    if (!ftf_)
    {
        ftf_.reset(new planning_models::Transforms(*parent_->getTransforms()));
        ftf_const_ = ftf_;
    }
    return ftf_;
}

void planning_scene::PlanningScene::getPlanningSceneDiffMsg(moveit_msgs::PlanningScene &scene) const
{
    if (ftf_)
        ftf_->getTransforms(scene.fixed_frame_transforms);
    else
        scene.fixed_frame_transforms.clear();

    if (kstate_)
    {
        planning_models::kinematicStateToRobotState(*kstate_, scene.robot_state);
        getPlanningSceneMsgAttachedBodies(scene);
    }
    else
    {
        scene.robot_state = moveit_msgs::RobotState();
        scene.attached_collision_objects.clear();
    }

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

    if (cworld_->isRecordingChanges())
    {
        scene.collision_objects.clear();
        scene.collision_map = moveit_msgs::CollisionMap();

        bool skip_cmap = false;
        const std::vector<collision_detection::CollisionWorld::Change> &changes = cworld_->getChanges();
        for (std::size_t i = 0 ; i < changes.size() ; ++i)
            if (changes[i].ns_ == COLLISION_MAP_NS)
            {
                if (!skip_cmap)
                {
                    skip_cmap = true;
                    getPlanningSceneMsgCollisionMap(scene);
                }
            }
            else
            {
                if (changes[i].type_ == collision_detection::CollisionWorld::Change::ADD)
                {
                    addPlanningSceneMsgCollisionObject(scene, changes[i].ns_);
                }
                else
                    if (changes[i].type_ == collision_detection::CollisionWorld::Change::REMOVE)
                    {
                        moveit_msgs::CollisionObject co;
                        co.header.frame_id = getPlanningFrame();
                        co.id = changes[i].ns_;
                        co.operation = moveit_msgs::CollisionObject::REMOVE;
                        scene.collision_objects.push_back(co);
                    }
                    else
                        ROS_ERROR("Unknown change on collision world");
            }
    }
    else
    {
        getPlanningSceneMsgCollisionObjects(scene);
        getPlanningSceneMsgCollisionMap(scene);
    }
}

void planning_scene::PlanningScene::getPlanningSceneMsgAttachedBodies(moveit_msgs::PlanningScene &scene) const
{
    scene.attached_collision_objects.clear();
    std::vector<const planning_models::KinematicState::AttachedBody*> ab;
    getCurrentState().getAttachedBodies(ab);

    for (std::size_t i = 0 ; i < ab.size() ; ++i)
    {
        moveit_msgs::AttachedCollisionObject aco;
        aco.link_name = ab[i]->getAttachedLinkName();
        aco.touch_links = ab[i]->getTouchLinks();
        aco.object.header.frame_id = aco.link_name;
        aco.object.id = ab[i]->getName();
        aco.object.operation = moveit_msgs::CollisionObject::ADD;
        const std::vector<shapes::Shape*>& ab_shapes = ab[i]->getShapes();
        const std::vector<btTransform>& ab_tf = ab[i]->getFixedTransforms();
        for (std::size_t j = 0 ; j < ab_shapes.size() ; ++j)
        {
            moveit_msgs::Shape sm;
            if (constructMsgFromShape(ab_shapes[j], sm))
            {
                aco.object.shapes.push_back(sm);
                geometry_msgs::Pose p;
                planning_models::msgFromPose(ab_tf[j], p);
                aco.object.poses.push_back(p);
            }
        }
        if (!aco.object.shapes.empty())
            scene.attached_collision_objects.push_back(aco);
    }
}

void planning_scene::PlanningScene::addPlanningSceneMsgCollisionObject(moveit_msgs::PlanningScene &scene, const std::string &ns) const
{
    moveit_msgs::CollisionObject co;
    co.header.frame_id = getPlanningFrame();
    co.id = ns;
    co.operation = moveit_msgs::CollisionObject::ADD;
    const collision_detection::CollisionWorld::NamespaceObjects &obj = *getCollisionWorld()->getObjects(ns);
    for (std::size_t j = 0 ; j < obj.static_shapes_.size() ; ++j)
    {
        moveit_msgs::StaticShape sm;
        if (constructMsgFromShape(obj.static_shapes_[j], sm))
            co.static_shapes.push_back(sm);
    }
    for (std::size_t j = 0 ; j < obj.shapes_.size() ; ++j)
    {
        moveit_msgs::Shape sm;
        if (constructMsgFromShape(obj.shapes_[j], sm))
        {
            co.shapes.push_back(sm);
            geometry_msgs::Pose p;
            planning_models::msgFromPose(obj.shape_poses_[j], p);
            co.poses.push_back(p);
        }
    }
    if (!co.shapes.empty() || !co.static_shapes.empty())
        scene.collision_objects.push_back(co);
}

void planning_scene::PlanningScene::getPlanningSceneMsgCollisionObjects(moveit_msgs::PlanningScene &scene) const
{
    scene.collision_objects.clear();
    const std::vector<std::string> &ns = getCollisionWorld()->getNamespaces();
    for (std::size_t i = 0 ; i < ns.size() ; ++i)
        if (ns[i] != COLLISION_MAP_NS)
            addPlanningSceneMsgCollisionObject(scene, ns[i]);
}

void planning_scene::PlanningScene::getPlanningSceneMsgCollisionMap(moveit_msgs::PlanningScene &scene) const
{
    scene.collision_map.header.frame_id = getPlanningFrame();
    scene.collision_map.boxes.clear();
    if (getCollisionWorld()->haveNamespace(COLLISION_MAP_NS))
    {
        const collision_detection::CollisionWorld::NamespaceObjects& map = *getCollisionWorld()->getObjects(COLLISION_MAP_NS);
        if (!map.static_shapes_.empty())
            ROS_ERROR("Static shapes are not supported in the collision map.");
        for (std::size_t i = 0 ; i < map.shapes_.size() ; ++i)
        {
            shapes::Box *b = static_cast<shapes::Box*>(map.shapes_[i]);
            moveit_msgs::OrientedBoundingBox obb;
            obb.extents.x = b->size[0]; obb.extents.y = b->size[1]; obb.extents.z = b->size[2];
            planning_models::msgFromPose(map.shape_poses_[i], obb.pose);
            scene.collision_map.boxes.push_back(obb);
        }
    }
}

void planning_scene::PlanningScene::getPlanningSceneMsg(moveit_msgs::PlanningScene &scene) const
{
    getTransforms()->getTransforms(scene.fixed_frame_transforms);
    planning_models::kinematicStateToRobotState(getCurrentState(), scene.robot_state);
    getAllowedCollisionMatrix().getMessage(scene.allowed_collision_matrix);
    getCollisionRobot()->getPadding(scene.link_padding);
    getCollisionRobot()->getScale(scene.link_scale);

    // add collision objects
    getPlanningSceneMsgCollisionObjects(scene);

    // add the attached bodies
    getPlanningSceneMsgAttachedBodies(scene);

    // get the collision map
    getPlanningSceneMsgCollisionMap(scene);
}

void planning_scene::PlanningScene::setCurrentState(const moveit_msgs::RobotState &state)
{
    if (parent_)
    {
        if (!kstate_)
            kstate_.reset(new planning_models::KinematicState(parent_->getCurrentState()));
        planning_models::robotStateToKinematicState(*getTransforms(), state, *kstate_);
    }
    else
        planning_models::robotStateToKinematicState(*ftf_, state, *kstate_);
}

void planning_scene::PlanningScene::setCurrentState(const planning_models::KinematicState &state)
{
    if (!kstate_)
        kstate_.reset(new planning_models::KinematicState(getKinematicModel()));
    *kstate_ = state;
}

void planning_scene::PlanningScene::decoupleParent(void)
{
    if (!parent_)
        return;
    if (parent_->isConfigured())
    {
        urdf_model_ = parent_->urdf_model_;
        srdf_model_ = parent_->srdf_model_;
        kmodel_ = parent_->kmodel_;
        kmodel_const_ = kmodel_;

        if (!ftf_)
        {
            ftf_.reset(new planning_models::Transforms(*parent_->getTransforms()));
            ftf_const_ = ftf_;
        }

        if (!kstate_)
            kstate_.reset(new planning_models::KinematicState(parent_->getCurrentState()));

        if (!acm_)
            acm_.reset(new collision_detection::AllowedCollisionMatrix(parent_->getAllowedCollisionMatrix()));

        crobot_unpadded_.reset(new DefaultCRobotType(kmodel_));

        if (!crobot_)
        {
            crobot_.reset(new DefaultCRobotType(static_cast<const DefaultCRobotType&>(*parent_->getCollisionRobot())));
            crobot_const_ = crobot_;
        }

        if (!cworld_)
        {
            cworld_.reset(new DefaultCWorldType(static_cast<const DefaultCWorldType&>(*parent_->getCollisionWorld())));
            cworld_const_ = cworld_;
        }
        else
        {
            cworld_->recordChanges(false);
            cworld_->clearChanges();
        }

        configured_ = true;
    }

    parent_.reset();
}

void planning_scene::PlanningScene::setPlanningSceneDiffMsg(const moveit_msgs::PlanningScene &scene)
{
    // there is at least one transform in the list of fixed transform: from model frame to itself;
    // if the list is empty, then nothing has been set
    if (!scene.fixed_frame_transforms.empty())
    {
        if (!ftf_)
        {
            ftf_.reset(new planning_models::Transforms(getKinematicModel()->getModelFrame()));
            ftf_const_ = ftf_;
        }
        ftf_->recordTransforms(scene.fixed_frame_transforms);
    }

    // if at least some joints have been specified, we set them
    if (!scene.robot_state.multi_dof_joint_state.joint_names.empty() ||
        !scene.robot_state.joint_state.name.empty())
        setCurrentState(scene.robot_state);

    if (!scene.attached_collision_objects.empty())
    {
        if (!kstate_) // there must be a parent in this case
            kstate_.reset(new planning_models::KinematicState(parent_->getCurrentState()));
        for (std::size_t i = 0 ; i < scene.attached_collision_objects.size() ; ++i)
            processAttachedCollisionObjectMsg(scene.attached_collision_objects[i]);
    }

    // if at least some links are mentioned in the allowed collision matrix, then we have an update
    if (!scene.allowed_collision_matrix.link_names.empty())
        acm_.reset(new collision_detection::AllowedCollisionMatrix(scene.allowed_collision_matrix));

    if (!scene.link_padding.empty() || !scene.link_scale.empty())
    {
        if (!crobot_)
        { // this means we have a parent too
            crobot_.reset(new DefaultCRobotType(static_cast<const DefaultCRobotType&>(*parent_->getCollisionRobot())));
            crobot_const_ = crobot_;
        }
        crobot_->setPadding(scene.link_padding);
        crobot_->setScale(scene.link_scale);
    }

    if ((!scene.collision_map.header.frame_id.empty() && !scene.collision_map.boxes.empty()) || !scene.collision_objects.empty())
    {
        for (std::size_t i = 0 ; i < scene.collision_objects.size() ; ++i)
            processCollisionObjectMsg(scene.collision_objects[i]);

        if (!scene.collision_map.header.frame_id.empty() && !scene.collision_map.boxes.empty())
            processCollisionMapMsg(scene.collision_map);
    }
}

void planning_scene::PlanningScene::setPlanningSceneMsg(const moveit_msgs::PlanningScene &scene)
{
    if (parent_)
    {
        // if we have a parent, but we set a new planning scene, then we do not care about the parent any more
        // and we no longer represent the scene as a diff
        urdf_model_ = parent_->urdf_model_;
        srdf_model_ = parent_->srdf_model_;
        kmodel_ = parent_->kmodel_;
        kmodel_const_ = kmodel_;

        if (!ftf_)
        {
            ftf_.reset(new planning_models::Transforms(kmodel_->getModelFrame()));
            ftf_const_ = ftf_;
        }

        if (!kstate_)
            kstate_.reset(new planning_models::KinematicState(kmodel_));

        if (!crobot_)
        {
            crobot_.reset(new DefaultCRobotType(kmodel_));
            crobot_const_ = crobot_;
        }
        crobot_unpadded_.reset(new DefaultCRobotType(kmodel_));

        cworld_->recordChanges(false);
        cworld_->clearChanges();

        configured_ = true;
        parent_.reset();
    }
    ftf_->recordTransforms(scene.fixed_frame_transforms);
    setCurrentState(scene.robot_state);
    acm_.reset(new collision_detection::AllowedCollisionMatrix(scene.allowed_collision_matrix));
    crobot_->setPadding(scene.link_padding);
    crobot_->setScale(scene.link_scale);
    cworld_->clearObjects();
    for (std::size_t i = 0 ; i < scene.collision_objects.size() ; ++i)
        processCollisionObjectMsg(scene.collision_objects[i]);
    kstate_->clearAttachedBodies();
    for (std::size_t i = 0 ; i < scene.attached_collision_objects.size() ; ++i)
        processAttachedCollisionObjectMsg(scene.attached_collision_objects[i]);
    processCollisionMapMsg(scene.collision_map);
}

void planning_scene::PlanningScene::processCollisionMapMsg(const moveit_msgs::CollisionMap &map)
{
    const btTransform &t = ftf_->getTransformToTargetFrame(*kstate_, map.header.frame_id);
    for (std::size_t i = 0 ; i < map.boxes.size() ; ++i)
    {
        btTransform p; planning_models::poseFromMsg(map.boxes[i].pose, p);
        shapes::Shape *s = new shapes::Box(map.boxes[i].extents.x, map.boxes[i].extents.y, map.boxes[i].extents.z);
        cworld_->addObject(COLLISION_MAP_NS, s, t * p);
    }
}

bool planning_scene::PlanningScene::processAttachedCollisionObjectMsg(const moveit_msgs::AttachedCollisionObject &object)
{
    if (!kmodel_->hasLinkModel(object.link_name))
    {
        ROS_ERROR("Unable to attach a body to link '%s' (link not found)", object.link_name.c_str());
        return false;
    }

    if (object.object.id == COLLISION_MAP_NS)
    {
        ROS_ERROR("The ID '%s' cannot be used for collision objects (name reserved)", COLLISION_MAP_NS.c_str());
        return false;
    }

    if (object.object.operation == moveit_msgs::CollisionObject::ADD)
    {
        if (object.object.shapes.size() != object.object.poses.size())
        {
            ROS_ERROR("Number of shapes does not match number of poses in attached collision object message");
            return false;
        }

        planning_models::KinematicState::LinkState *ls = kstate_->getLinkState(object.link_name);
        if (ls)
        {
            std::vector<shapes::Shape*> shapes;
            std::vector<btTransform>    poses;

            // we need to add some shapes; if the message is empty, maybe the object is already in the world
            if (object.object.shapes.empty())
            {
                if (cworld_->haveNamespace(object.object.id))
                {
                    ROS_DEBUG("Attaching world object '%s' to link '%s'", object.object.id.c_str(), object.link_name.c_str());

                    // extract the shapes from the world
                    collision_detection::CollisionWorld::NamespaceObjectsPtr obj = cworld_->getObjects(object.object.id);
                    shapes = obj->shapes_;
                    poses = obj->shape_poses_;
                    // remove the pointer to the objects from the collision world
                    cworld_->clearObjects(object.object.id);

                    if (obj.unique())
                    {
                        // make sure the memory for the shapes is not deleted
                        obj->shapes_.clear();
                        ROS_DEBUG("The memory representing shapes was moved from the collision world to the planning model");
                    }
                    else
                    {
                        // clone the shapes because we cannot assume their ownership; this will probably rarely happen (if ever)
                        for (std::size_t i = 0 ; i < shapes.size() ; ++i)
                            shapes[i] = shapes[i]->clone();
                        ROS_DEBUG("The memory representing shapes was copied from the collision world to the planning model");
                    }

                    if (!obj->static_shapes_.empty())
                        ROS_WARN("Static shapes from object '%s' are lost when the object is attached to the robot", object.object.id.c_str());

                    // need to transform poses to the link frame
                    const btTransform &i_t = ls->getGlobalLinkTransform().inverse();
                    for (std::size_t i = 0 ; i < poses.size() ; ++i)
                        poses[i] = i_t * poses[i];
                }
                else
                {
                    ROS_ERROR("Attempting to attach object '%s' to link '%s' but no geometry specified and such an object does not exist in the collision world",
                              object.object.id.c_str(), object.link_name.c_str());
                    return false;
                }
            }
            else
            {
                // we clear the world objects with the same name, since we got an update on their geometry
                if (cworld_->haveNamespace(object.object.id))
                    cworld_->clearObjects(object.object.id);
                if (!object.object.static_shapes.empty())
                    ROS_ERROR("Static shapes are ignored for attached object '%s'", object.object.id.c_str());

                for (std::size_t i = 0 ; i < object.object.shapes.size() ; ++i)
                {
                    shapes::Shape *s = shapes::constructShapeFromMsg(object.object.shapes[i]);
                    if (s)
                    {
                        btTransform p; planning_models::poseFromMsg(object.object.poses[i], p);
                        shapes.push_back(s);
                        poses.push_back(p);
                    }
                }
                // transform poses to link frame
                if (object.object.header.frame_id != object.link_name)
                {
                    const btTransform &t = ls->getGlobalLinkTransform().inverse() * ftf_->getTransformToTargetFrame(*kstate_, object.object.header.frame_id);
                    for (std::size_t i = 0 ; i < poses.size() ; ++i)
                        poses[i] = t * poses[i];
                }
            }

            if (shapes.empty())
            {
                ROS_ERROR("There is no geometry to attach to link '%s' as part of attached body '%s'", object.link_name.c_str(), object.object.id.c_str());
                return false;
            }

            // there should not exist an attached object with this name
            if (ls->clearAttachedBody(object.object.id))
                ROS_WARN("The kinematic state already had an object named '%s' attached to link '%s'. The object was replaced.",
                         object.object.id.c_str(), object.link_name.c_str());
            ls->attachBody(object.object.id, shapes, poses, object.touch_links);
            ROS_DEBUG("Attached object '%s' to link '%s'", object.object.id.c_str(), object.link_name.c_str());
            return true;
        }
        else
            ROS_FATAL("Kinematic state is not compatible with kinematic model");
    }
    else
        if (object.object.operation == moveit_msgs::CollisionObject::REMOVE)
        {
            planning_models::KinematicState::LinkState *ls = kstate_->getLinkState(object.link_name);
            if (ls)
            {
                const planning_models::KinematicState::AttachedBody *ab = ls->getAttachedBody(object.object.id);
                if (ab)
                {
                    boost::shared_ptr<planning_models::KinematicState::AttachedBodyProperties> prop = ab->getProperties();
                    std::vector<btTransform> poses = ab->getGlobalCollisionBodyTransforms();
                    ls->clearAttachedBody(object.object.id);

                    if (prop.unique())
                    {
                        ROS_DEBUG("The memory representing shapes was moved from the planning model to the collision world");
                        cworld_->addObjects(object.object.id, prop->shapes_, poses);
                        prop->shapes_.clear(); // memory is now owned by the collision world
                    }
                    else
                    {
                        // the attached body is used elsewhere, so we do not modify it
                        std::vector<shapes::Shape*> shapes(prop->shapes_.size());
                        for (std::size_t i = 0 ; i < shapes.size() ; ++i)
                            shapes[i] = prop->shapes_[i]->clone();
                        ROS_DEBUG("The memory representing shapes was copied from the planning model to the collision world");
                        cworld_->addObjects(object.object.id, shapes, poses);
                    }
                    ROS_DEBUG("Detached object '%s' from link '%s' and added it back in the collision world", object.object.id.c_str(), object.link_name.c_str());
                    return true;
                }
                else
                    ROS_ERROR("No object named '%s' is attached to link '%s'", object.object.id.c_str(), object.link_name.c_str());
            }
            else
                ROS_FATAL("Kinematic state is not compatible with kinematic model");
        }
        else
            ROS_ERROR("Unknown collision object operation: %d", object.object.operation);
    return false;
}

bool planning_scene::PlanningScene::processCollisionObjectMsg(const moveit_msgs::CollisionObject &object)
{
    if (object.id == COLLISION_MAP_NS)
    {
        ROS_ERROR("The ID '%s' cannot be used for collision objects (name reserved)", COLLISION_MAP_NS.c_str());
        return false;
    }

    if (object.operation == moveit_msgs::CollisionObject::ADD)
    {
        if (object.shapes.empty() && object.static_shapes.empty())
        {
            ROS_ERROR("There are no shapes specified in the collision object message");
            return false;
        }
        if (object.shapes.size() != object.poses.size())
        {
            ROS_ERROR("Number of shapes does not match number of poses in collision object message");
            return false;
        }

        for (std::size_t i = 0 ; i < object.static_shapes.size() ; ++i)
        {
            shapes::StaticShape *s = shapes::constructShapeFromMsg(object.static_shapes[i]);
            if (s)
                cworld_->addObject(object.id, s);
        }

        const btTransform &t = ftf_->getTransformToTargetFrame(*kstate_, object.header.frame_id);
        for (std::size_t i = 0 ; i < object.shapes.size() ; ++i)
        {
            shapes::Shape *s = shapes::constructShapeFromMsg(object.shapes[i]);
            if (s)
            {
                btTransform p; planning_models::poseFromMsg(object.poses[i], p);
                cworld_->addObject(object.id, s, t * p);
            }
        }
        return true;
    }
    else
        if (object.operation == moveit_msgs::CollisionObject::REMOVE)
        {
            cworld_->clearObjects(object.id);
            return true;
        }
        else
            ROS_ERROR("Unknown collision object operation: %d", object.operation);
    return false;
}
