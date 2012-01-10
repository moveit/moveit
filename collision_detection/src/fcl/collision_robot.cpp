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

#include "collision_detection/fcl/collision_robot.h"
#include <ros/console.h>

collision_detection::CollisionRobotFCL::CollisionRobotFCL(const planning_models::KinematicModelConstPtr &kmodel, double padding, double scale) : CollisionRobot(kmodel, padding, scale)
{
    links_ = kmodel_->getLinkModels();

    // we keep the same order of objects as what KinematicState::getLinkState() returns
    for (std::size_t i = 0 ; i < links_.size() ; ++i)
        if (links_[i] && links_[i]->getShape())
        {
            boost::shared_ptr<fcl::CollisionGeometry> cg =
                createCollisionGeometry(links_[i]->getShape().get(), getLinkScale(links_[i]->getName()), getLinkPadding(links_[i]->getName()));
            if (cg)
            {
                CollisionGeometryData *cgd = new CollisionGeometryData(links_[i]);
                collision_geometry_data_[links_[i]->getName()].reset(cgd);
                index_map_[links_[i]->getName()] = geoms_.size();
                cg->setUserData(cgd);
            }
            else
                links_[i] = NULL;
            geoms_.push_back(cg);
        }
        else
        {
            links_[i] = NULL;
            geoms_.push_back(boost::shared_ptr<fcl::CollisionGeometry>());
        }
}

collision_detection::CollisionRobotFCL::CollisionRobotFCL(const CollisionRobotFCL &other) : CollisionRobot(other)
{
    links_ = other.links_;
    geoms_ = other.geoms_;
    collision_geometry_data_ = other.collision_geometry_data_;
    index_map_ = other.index_map_;
}

const std::vector<boost::shared_ptr<fcl::CollisionGeometry> >&
collision_detection::CollisionRobotFCL::getAttachedBodyObjects(const planning_models::KinematicState::AttachedBody *ab) const
{
    // need an actualy instance of the shared ptr for thread safety
    boost::shared_ptr<planning_models::KinematicState::AttachedBodyProperties> props = ab->getProperties();
    boost::mutex::scoped_lock slock(attached_bodies_lock_);
    AttachedBodyObject::const_iterator it = attached_bodies_.find(props);
    if (it != attached_bodies_.end())
        return it->second;

    CollisionGeometryData *cgd = new CollisionGeometryData(props.get());

    // this is safe because collision_geometry_data_ is not modified elsewhere and we are already locked in this function
    const_cast<CollisionRobotFCL*>(this)->collision_geometry_data_[props->id_].reset(cgd);

    const std::vector<shapes::Shape*> &shapes = ab->getShapes();
    std::vector<boost::shared_ptr<fcl::CollisionGeometry> > obj;
    for (std::size_t i = 0 ; i < shapes.size() ; ++i)
    {
        boost::shared_ptr<fcl::CollisionGeometry> co = createCollisionGeometry(shapes[i]);
        if (co)
            co->setUserData(cgd);
        obj.push_back(co);
    }
    return attached_bodies_.insert(std::make_pair(props, obj)).first->second;
}

void collision_detection::CollisionRobotFCL::constructFCLObject(const planning_models::KinematicState &state, FCLObject &fcl_obj) const
{
    const std::vector<planning_models::KinematicState::LinkState*> &link_states = state.getLinkStateVector();
    for (std::size_t i = 0 ; i < geoms_.size() ; ++i)
        if (geoms_[i])
        {
            fcl::CollisionObject *collObj = new fcl::CollisionObject(geoms_[i], transform2fcl(link_states[i]->getGlobalCollisionBodyTransform()));
            fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
            const std::vector<planning_models::KinematicState::AttachedBody*> ab = link_states[i]->getAttachedBodies();
            for (std::size_t j = 0 ; j < ab.size() ; ++j)
            {
                const std::vector<boost::shared_ptr<fcl::CollisionGeometry> > &objs = getAttachedBodyObjects(ab[j]);
                const std::vector<btTransform> &ab_t = ab[j]->getGlobalCollisionBodyTransforms();
                for (std::size_t k = 0 ; k < objs.size() ; ++k)
                    if (objs[k])
                    {
                        fcl::CollisionObject *collObj = new fcl::CollisionObject(objs[k], transform2fcl(ab_t[k]));
                        fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
                    }
            }
        }
}

void collision_detection::CollisionRobotFCL::allocSelfCollisionBroadPhase(const planning_models::KinematicState &state, FCLManager &manager) const
{
    manager.manager_.reset(new fcl::SSaPCollisionManager());
    constructFCLObject(state, manager.object_);
    manager.object_.registerTo(manager.manager_.get());
}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state) const
{
    checkSelfCollisionHelper(req, res, state, NULL);
}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                const AllowedCollisionMatrix &acm) const
{
    checkSelfCollisionHelper(req, res, state, &acm);
}

void collision_detection::CollisionRobotFCL::checkSelfCollisionHelper(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                      const AllowedCollisionMatrix *acm) const
{
    FCLManager manager;
    allocSelfCollisionBroadPhase(state, manager);
    CollisionData cd(&req, &res, acm);
    manager.manager_->collide(&cd, &collisionCallback);
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                 const CollisionRobot &other_robot, const planning_models::KinematicState &other_state) const
{
    checkOtherCollisionHelper(req, res, state, other_robot, other_state, NULL);
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                 const CollisionRobot &other_robot, const planning_models::KinematicState &other_state,
                                                                 const AllowedCollisionMatrix &acm) const
{
    checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void collision_detection::CollisionRobotFCL::checkOtherCollisionHelper(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                       const CollisionRobot &other_robot, const planning_models::KinematicState &other_state,
                                                                       const AllowedCollisionMatrix *acm) const
{
    FCLManager manager;
    allocSelfCollisionBroadPhase(state, manager);

    const CollisionRobotFCL &fcl_rob = dynamic_cast<const CollisionRobotFCL&>(other_robot);
    FCLObject other_fcl_obj;
    fcl_rob.constructFCLObject(other_state, other_fcl_obj);

    CollisionData cd(&req, &res, acm);
    for (std::size_t i = 0 ; !cd.done_ && i < other_fcl_obj.collision_objects_.size() ; ++i)
        manager.manager_->collide(other_fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);
}

void collision_detection::CollisionRobotFCL::updatedPaddingOrScaling(const std::vector<std::string> &links)
{
    for (std::size_t i = 0 ; i < links.size() ; ++i)
    {
        std::map<std::string, std::size_t>::const_iterator it = index_map_.find(links[i]);
        const planning_models::KinematicModel::LinkModel *lmodel = kmodel_->getLinkModel(links[i]);
        if (it != index_map_.end() && lmodel)
        {
            boost::shared_ptr<fcl::CollisionGeometry> cg = createCollisionGeometry(lmodel->getShape().get(), getLinkScale(links[i]), getLinkPadding(links[i]));
            cg->setUserData(geoms_[it->second]->getUserData());
            geoms_[it->second] = cg;
        }
        else
            ROS_ERROR("Updating padding or scaling for unknown link: '%s'", links[i].c_str());
    }
}
