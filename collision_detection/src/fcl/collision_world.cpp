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

/* Author Ioan Sucan */

#include "collision_detection/fcl/collision_world.h"

collision_detection::CollisionWorldFCL::CollisionWorldFCL(void) : CollisionWorld()
{
    manager_.reset(new fcl::SSaPCollisionManager());
}

collision_detection::CollisionWorldFCL::~CollisionWorldFCL(void)
{
}


void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const planning_models::KinematicState &state) const
{
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const planning_models::KinematicState &state, const AllowedCollisionMatrix &acm) const
{
}

void collision_detection::CollisionWorldFCL::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world) const
{
    checkWorldCollisionHelper(req, res, other_world, NULL);
}

void collision_detection::CollisionWorldFCL::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix &acm) const
{
    checkWorldCollisionHelper(req, res, other_world, &acm);
}

void collision_detection::CollisionWorldFCL::checkWorldCollisionHelper(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix *acm) const
{
    const CollisionWorldFCL *other_fcl_world = dynamic_cast<const CollisionWorldFCL*>(&other_world);
    if (other_fcl_world)
    {
	CollisionData cd(&req, &res, acm);
	manager_->collide(other_fcl_world->manager_.get(), &cd, &collisionCallback);
    }
    else
    {
	const std::vector<std::string> &other_ids = other_world.getObjectIds();
        for (std::size_t i = 0 ; i < other_ids.size() ; ++i)
        {
            ObjectConstPtr obj = other_world.getObject(other_ids[i]);
            // create collision objects for everything & call collide()
	    FCLObject fcl_obj;
	    constructFCLObject(obj.get(), fcl_obj);
	    CollisionData cd(&req, &res, acm);
	    for (std::size_t j = 0 ; j < fcl_obj.collision_objects_.size() && !cd.done_ ; ++j)
		manager_->collide(fcl_obj.collision_objects_[j].get(),  &cd, &collisionCallback);
	}
    }
}

void collision_detection::CollisionWorldFCL::constructFCLObject(const Object *obj, FCLObject &fcl_obj) const
{
    for (std::size_t i = 0 ; i < obj->static_shapes_.size() ; ++i)
    {
	boost::shared_ptr<fcl::CollisionGeometry> cg = createCollisionGeometry(obj->static_shapes_[i]);
	if (cg)
	{    
	    CollisionGeometryData *cgd = new CollisionGeometryData(obj);
	    cg->setUserData(cgd);
	    fcl::CollisionObject *co = new fcl::CollisionObject(cg);
	    fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(co));
	    fcl_obj.collision_geometry_data_.push_back(boost::shared_ptr<CollisionGeometryData>(cgd));
	}
    }
    for (std::size_t i = 0 ; i < obj->shapes_.size() ; ++i)
    {
	boost::shared_ptr<fcl::CollisionGeometry> cg = createCollisionGeometry(obj->shapes_[i]);
	if (cg)
	{    
	    CollisionGeometryData *cgd = new CollisionGeometryData(obj);
	    cg->setUserData(cgd);
	    fcl::CollisionObject *co = new fcl::CollisionObject(cg,  transform2fcl(obj->shape_poses_[i]));
	    fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(co));
	    fcl_obj.collision_geometry_data_.push_back(boost::shared_ptr<CollisionGeometryData>(cgd));
	}
    }
}

void collision_detection::CollisionWorldFCL::updateFCLObject(const std::string &id)
{
    // check to see if we have this object
    std::map<std::string, ObjectPtr>::iterator it = objects_.find(id);
    if (it == objects_.end())
	return;
    
    // remove FCL objects that correspond to this object
    std::map<std::string, FCLObject>::iterator jt = fcl_objs_.find(id);
    jt->second.unregisterFrom(manager_.get());
    jt->second.clear();
    
    // construct FCL objects that correspond to this object
    constructFCLObject(it->second.get(), jt->second);
    jt->second.registerTo(manager_.get());
}

void collision_detection::CollisionWorldFCL::addToObject(const std::string &id, shapes::StaticShape *shape)
{
    CollisionWorld::addToObject(id, shape);
    updateFCLObject(id);
}

void collision_detection::CollisionWorldFCL::addToObject(const std::string &id, shapes::Shape *shape, const btTransform &pose)
{
    CollisionWorld::addToObject(id, shape, pose);
    updateFCLObject(id);
}

bool collision_detection::CollisionWorldFCL::moveShapeInObject(const std::string &id, const shapes::Shape *shape, const btTransform &pose)
{
    if (CollisionWorld::moveShapeInObject(id, shape, pose))
    {
	updateFCLObject(id);
	return true;
    }
    else
	return false;
}

bool collision_detection::CollisionWorldFCL::removeShapeFromObject(const std::string &id, const shapes::Shape *shape)
{
    if (CollisionWorld::removeShapeFromObject(id, shape))
    {
	updateFCLObject(id);
        return true;
    }
    else
        return false;
}

bool collision_detection::CollisionWorldFCL::removeStaticShapeFromObject(const std::string &id, const shapes::StaticShape *shape)
{
    if (CollisionWorld::removeStaticShapeFromObject(id, shape))
    {
	updateFCLObject(id);
	return true;
    }
    else	    
        return false;
}

void collision_detection::CollisionWorldFCL::removeObject(const std::string &id)
{
    CollisionWorld::removeObject(id);
    std::map<std::string, FCLObject>::iterator it = fcl_objs_.find(id);
    if (it != fcl_objs_.end())
    {
	it->second.unregisterFrom(manager_.get());
	it->second.clear();
	fcl_objs_.erase(it);
    }
}

void collision_detection::CollisionWorldFCL::clearObjects(void)
{
    CollisionWorld::clearObjects();
    manager_->clear();
    fcl_objs_.clear();
}

void collision_detection::CollisionWorldFCL::FCLObject::registerTo(fcl::BroadPhaseCollisionManager *manager)
{
    for (std::size_t i = 0 ; i < collision_objects_.size() ; ++i)
	manager->registerObject(collision_objects_[i].get());
}

void collision_detection::CollisionWorldFCL::FCLObject::unregisterFrom(fcl::BroadPhaseCollisionManager *manager)
{
    for (std::size_t i = 0 ; i < collision_objects_.size() ; ++i)
	manager->unregisterObject(collision_objects_[i].get());
}

void collision_detection::CollisionWorldFCL::FCLObject::clear(void)
{
    collision_objects_.clear();
    collision_geometry_data_.clear();
}
