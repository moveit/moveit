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

static const std::string COLLISION_MAP_NS = "__map";

bool planning_scene::PlanningScene::configure(const boost::shared_ptr<const urdf::Model> &urdf_model,
                                              const boost::shared_ptr<const srdf::Model> &srdf_model)
{
    kmodel_.reset(new planning_models::KinematicModel(urdf_model, srdf_model));
    kmodel_const_ = kmodel_;
    tf_.reset(new planning_models::Transforms(kmodel_->getModelFrame()));
    kstate_.reset(new planning_models::KinematicState(kmodel_));
    kstate_->setDefaultValues();
    crobot_.reset(new collision_detection::CollisionRobotAllValid(kmodel_));
    crobot_unpadded_.reset(new collision_detection::CollisionRobotAllValid(kmodel_));
    cworld_.reset(new collision_detection::CollisionWorldAllValid());
    configured_ = true;
    return true;
}

void planning_scene::PlanningScene::checkCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult &res,
                                                   const planning_models::KinematicState &kstate) const
{
  // do self-collision checking with the unpadded version of the robot
  crobot_unpadded_->checkSelfCollision(req, res, kstate, acm_);

  // check collision with the world using the padded version
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    cworld_->checkRobotCollision(req, res, *crobot_, kstate, acm_);
}

void planning_scene::PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                                   collision_detection::CollisionResult &res,
                                                   const planning_models::KinematicState &kstate,
                                                   const collision_detection::AllowedCollisionMatrix& acm) const
{
  // do self-collision checking with the unpadded version of the robot
  crobot_unpadded_->checkSelfCollision(req, res, kstate, acm);

  // check collision with the world using the padded version
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    cworld_->checkRobotCollision(req, res, *crobot_, kstate, acm);
}


void planning_scene::PlanningScene::getPlanningSceneMsg(moveit_msgs::PlanningScene &scene) const
{
    tf_->getTransforms(scene.fixed_frame_transforms);
    planning_models::kinematicStateToRobotState(*kstate_, scene.robot_state);
    acm_.getMessage(scene.allowed_collision_matrix);
    crobot_->getPadding(scene.link_padding);

    // add collision objects
    scene.collision_objects.clear();
    const std::vector<std::string> &ns = cworld_->getNamespaces();
    for (std::size_t i = 0 ; i < ns.size() ; ++i)
        if (ns[i] != COLLISION_MAP_NS)
        {
            moveit_msgs::CollisionObject co;
            co.header.frame_id = tf_->getPlanningFrame();
            co.id = ns[i];
            co.operation = moveit_msgs::CollisionObject::ADD;
            const collision_detection::CollisionWorld::NamespaceObjects &obj = cworld_->getObjects(ns[i]);
            if (!obj.static_shape.empty())
                ROS_WARN("Static shapes not encoded in PlanningScene message");
            for (std::size_t j = 0 ; j < obj.shape.size() ; ++j)
            {
                moveit_msgs::Shape sm;
                if (constructMsgFromShape(obj.shape[j], sm))
                {
                    co.shapes.push_back(sm);
                    geometry_msgs::Pose p;
                    planning_models::msgFromPose(obj.shape_pose[j], p);
                    co.poses.push_back(p);
                }
            }
            if (!co.shapes.empty())
                scene.collision_objects.push_back(co);
        }

    // add the attached bodies
    scene.attached_collision_objects.clear();
    std::vector<const planning_models::KinematicState::AttachedBody*> ab;
    kstate_->getAttachedBodies(ab);
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

    // get the collision map
    scene.collision_map.header.frame_id = tf_->getPlanningFrame();
    scene.collision_map.boxes.clear();
    const collision_detection::CollisionWorld::NamespaceObjects& map = cworld_->getObjects(COLLISION_MAP_NS);
    if (!map.static_shape.empty())
        ROS_WARN("Static shapes not encoded in PlanningScene message");
    for (std::size_t i = 0 ; i < map.shape.size() ; ++i)
    {
        shapes::Box *b = static_cast<shapes::Box*>(map.shape[i]);
        moveit_msgs::OrientedBoundingBox obb;
        obb.extents.x = b->size[0]; obb.extents.y = b->size[1]; obb.extents.z = b->size[2];
        planning_models::msgFromPose(map.shape_pose[i], obb.pose);
        scene.collision_map.boxes.push_back(obb);
    }
}

void planning_scene::PlanningScene::setCurrentState(const moveit_msgs::RobotState &state)
{
    planning_models::robotStateToKinematicState(*tf_, state, *kstate_);
}

void planning_scene::PlanningScene::setCurrentState(const planning_models::KinematicState &state)
{
    *kstate_ = state;
}

void planning_scene::PlanningScene::setPlanningSceneMsg(const moveit_msgs::PlanningScene &scene)
{
    tf_->recordTransforms(scene.fixed_frame_transforms);
    setCurrentState(scene.robot_state);
    acm_ = collision_detection::AllowedCollisionMatrix(scene.allowed_collision_matrix);
    crobot_->setPadding(scene.link_padding);
    cworld_->clearObjects();
    for (std::size_t i = 0 ; i < scene.collision_objects.size() ; ++i)
        processCollisionObjectMsg(scene.collision_objects[i]);
    for (std::size_t i = 0 ; i < scene.attached_collision_objects.size() ; ++i)
        processAttachedCollisionObjectMsg(scene.attached_collision_objects[i]);
    processCollisionMapMsg(scene.collision_map);
}

void planning_scene::PlanningScene::processCollisionMapMsg(const moveit_msgs::CollisionMap &map)
{
    const btTransform &t = tf_->getTransformToTargetFrame(*kstate_, map.header.frame_id);
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
                    // extract the shapes from the world
                    const collision_detection::CollisionWorld::NamespaceObjects &obj = cworld_->getObjects(object.object.id);
                    shapes = obj.shape;
                    poses = obj.shape_pose;
                    cworld_->removeObjects(object.object.id);
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
                    const btTransform &t = ls->getGlobalLinkTransform().inverse() * tf_->getTransformToTargetFrame(*kstate_, object.object.header.frame_id);
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
                        cworld_->addObjects(object.object.id, prop->shapes_, poses);
                        prop->shapes_.clear(); // memory is now owned by the collision world
                    }
                    else
                    {
                        // the attached body is used elsewhere, so we do not modify it
                        std::vector<shapes::Shape*> shapes(prop->shapes_.size());
                        for (std::size_t i = 0 ; i < shapes.size() ; ++i)
                            shapes[i] = prop->shapes_[i]->clone();
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
        if (object.shapes.empty())
        {
            ROS_ERROR("There are no shapes specified in the collision object message");
            return false;
        }
        if (object.shapes.size() != object.poses.size())
        {
            ROS_ERROR("Number of shapes does not match number of poses in collision object message");
            return false;
        }

        const btTransform &t = tf_->getTransformToTargetFrame(*kstate_, object.header.frame_id);
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
