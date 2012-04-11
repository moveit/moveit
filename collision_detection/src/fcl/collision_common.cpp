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

/* Author: Ioan Sucan, Jia Pan */

#include "collision_detection/fcl/collision_common.h"
#include <fcl/geometric_shape_to_BVH_model.h>
#include <ros/console.h>

namespace collision_detection
{
bool collisionCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void *data)
{
  CollisionData *cdata = (CollisionData*)data;
  if (cdata->done_)
    return true;
  const CollisionGeometryData *cd1 = static_cast<const CollisionGeometryData*>(o1->getCollisionGeometry()->getUserData());
  const CollisionGeometryData *cd2 = static_cast<const CollisionGeometryData*>(o2->getCollisionGeometry()->getUserData());
  
  // use the collision matrix (if any) to avoid certain collision checks
  DecideContactFn dcf;
  bool always_allow_collision = false;
  if (cdata->acm_)
  {
    AllowedCollision::Type type;
    bool found = cdata->acm_->getAllowedCollision(cd1->getID(), cd2->getID(), type);
    if (found)
    {
      // if we have an entry in the collision matrix, we read it
      if (type == AllowedCollision::ALWAYS)
      {
        always_allow_collision = true;
        if (cdata->req_->verbose)
          ROS_DEBUG_NAMED("allowed_collisions", "Collision between '%s' and '%s' is always allowed. No contacts are computed.",
                          cd1->getID().c_str(), cd2->getID().c_str());
      }
      else
        if (type == AllowedCollision::CONDITIONAL)
        {
          cdata->acm_->getAllowedCollision(cd1->getID(), cd2->getID(), dcf);
          if (cdata->req_->verbose)
            ROS_DEBUG_NAMED("allowed_collisions", "Collision between '%s' and '%s' is conditionally allowed", cd1->getID().c_str(), cd2->getID().c_str());
        }
    }
  }
  
  // check if a link is touching an attached object
  if (cd1->type == BodyTypes::ROBOT_LINK && cd2->type == BodyTypes::ROBOT_ATTACHED)
  {
    const std::set<std::string> &tl = cd2->ptr.ab->getTouchLinks();
    if (tl.find(cd1->getID()) != tl.end())
    {
      always_allow_collision = true;
      if (cdata->req_->verbose)
        ROS_DEBUG_NAMED("allowed_collisions", "Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                        cd1->getID().c_str(), cd2->getID().c_str());
    }
  }
  else
    if (cd2->type == BodyTypes::ROBOT_LINK && cd1->type == BodyTypes::ROBOT_ATTACHED)
    {
      const std::set<std::string> &tl = cd1->ptr.ab->getTouchLinks();
      if (tl.find(cd2->getID()) != tl.end())
      {
        always_allow_collision = true;
        if (cdata->req_->verbose)
          ROS_DEBUG_NAMED("allowed_collisions", "Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                          cd2->getID().c_str(), cd1->getID().c_str());
      }
    }
  
  // if collisions are always allowed, we are done
  if (always_allow_collision)
    return false;

  if (cdata->req_->verbose)
    ROS_DEBUG_STREAM_NAMED("allowed_collisions", "Actually checking collisions between " << cd1->getID() << " and " << cd2->getID());
  
  // see if we need to compute a contact
  std::size_t want_contact_count = 0;
  if (cdata->req_->contacts)
    if (cdata->res_->contact_count < cdata->req_->max_contacts)
    {
      std::size_t have;
      if (cd1->getID() < cd2->getID())
      {
        std::pair<std::string, std::string> cp(cd1->getID(), cd2->getID());
        have = cdata->res_->contacts.find(cp) != cdata->res_->contacts.end() ? cdata->res_->contacts[cp].size() : 0;
      }
      else
      {
        std::pair<std::string, std::string> cp(cd2->getID(), cd1->getID());
        have = cdata->res_->contacts.find(cp) != cdata->res_->contacts.end() ? cdata->res_->contacts[cp].size() : 0;
      }
      if (have < cdata->req_->max_contacts_per_pair)
        want_contact_count = cdata->req_->max_contacts_per_pair - have;
    }
  
  if (dcf)
  {
    // if we have a decider for allowed contacts, we need to look at all the contacts
    bool exhaustive = true;
    bool enable_contact = true;
    std::vector<fcl::Contact> contacts;
    int num_contacts = fcl::collide(o1, o2, std::numeric_limits<int>::max(), exhaustive, enable_contact, contacts);
    if (num_contacts > 0)
    {
      if (cdata->req_->verbose)
        ROS_INFO_NAMED("contact_information", "Found %d contacts between '%s' and '%s'. These contacts will be evaluated to check if they are accepted or not",
                       num_contacts, cd1->getID().c_str(), cd2->getID().c_str());       
      Contact c;
      const std::pair<std::string, std::string> &pc = cd1->getID() < cd2->getID() ?
        std::make_pair(cd1->getID(), cd2->getID()) : std::make_pair(cd2->getID(), cd1->getID());
      for (int i = 0 ; i < num_contacts ; ++i)
      {
        fcl2contact(contacts[i], c);
        // if the contact is  not allowed, we have a collision
        if (dcf(c) == false)
        {
          // store the contact, if it is needed
          if (want_contact_count > 0)
          {
            --want_contact_count;
            cdata->res_->contacts[pc].push_back(c);
            cdata->res_->contact_count++;
            if (cdata->req_->verbose)
              ROS_INFO_NAMED("contact_information", "Found unacceptable contact between '%s' and '%s'. Contact was stored.",
                             cd1->getID().c_str(), cd2->getID().c_str());
          }
          else
            if (cdata->req_->verbose)
              ROS_INFO_NAMED("contact_information", "Found unacceptable contact between '%s' and '%s'. Contact was not stored.",
                             cd1->getID().c_str(), cd2->getID().c_str());
          cdata->res_->collision = true;
          if (want_contact_count == 0)
            break;
        }
      }
    }
  }
  else
  {
    if (want_contact_count > 0)
    {
      // otherwise, we need to compute more things
      bool exhaustive = false;
      bool enable_contact = true;
      std::vector<fcl::Contact> contacts;
      int num_contacts = fcl::collide(o1, o2, want_contact_count, exhaustive, enable_contact, contacts);
      if (num_contacts > 0)
      {
        // make sure we don't get more contacts than we want
        if (want_contact_count >= (std::size_t)num_contacts)
          want_contact_count -= num_contacts;
        else
        {
          num_contacts = want_contact_count;
          want_contact_count = 0;
        }
	
        if (cdata->req_->verbose)
          ROS_INFO_NAMED("contact_information", "Found %d contacts between '%s' and '%s', which constitute a collision. %d contacts will be stored",
                         num_contacts, cd1->getID().c_str(), cd2->getID().c_str(), (int)num_contacts);
        const std::pair<std::string, std::string> &pc = cd1->getID() < cd2->getID() ?
          std::make_pair(cd1->getID(), cd2->getID()) : std::make_pair(cd2->getID(), cd1->getID());
        cdata->res_->collision = true;
        for (int i = 0 ; i < num_contacts ; ++i)
        {
          Contact c;
          fcl2contact(contacts[i], c);
          cdata->res_->contacts[pc].push_back(c);
          cdata->res_->contact_count++;
        }
      }
    }
    else
    {
      bool exhaustive = false;
      bool enable_contact = false;
      std::vector<fcl::Contact> contacts;
      int num_contacts = fcl::collide(o1, o2, 1, exhaustive, enable_contact, contacts);
      if (num_contacts > 0)
      {
        cdata->res_->collision = true;
        if (cdata->req_->verbose)
          ROS_INFO_NAMED("contact_information", "Found a contact between '%s' and '%s', which constitutes a collision. Contact information is not stored.",
                         cd1->getID().c_str(), cd2->getID().c_str());
      }
    }
  }
  
  if (cdata->res_->collision)
    if (!cdata->req_->contacts || cdata->res_->contact_count >= cdata->req_->max_contacts)
    {
      cdata->done_ = true;
      if (cdata->req_->verbose) 
        ROS_INFO_NAMED("contact_information", "Collision checking is considered complete (collision was found and %d contacts are stored)",
                       (unsigned int)cdata->res_->contact_count);
    }
  
  return cdata->done_;
}

/* Cache for shapes; The structure is the same for StaticShape or Shape */
template<typename ShapeType>
struct FCLShapeCache
{
  FCLShapeCache(void) : clean_count_(0) {}
  static const unsigned int MAX_CLEAN_COUNT = 100; // every this many uses of the cache, a cleaning operation is executed (this is only removal of expired entries)
  std::map<boost::weak_ptr<const ShapeType>, FCLGeometryConstPtr> map_;
  unsigned int clean_count_;
  boost::mutex lock_;
};

/* Cache for static shapes */
typedef FCLShapeCache<shapes::StaticShape> FCLShapeCache_StaticShape;

/* Cache for shapes (not static) */
typedef FCLShapeCache<shapes::Shape> FCLShapeCache_Shape;

/* We template the function so we get a different cache for each of the template arguments combinations */
template<typename BV, typename T>
FCLShapeCache_StaticShape& GetStaticShapeCache(void)
{
  static FCLShapeCache_StaticShape cache;
  return cache;
}

/* We template the function so we get a different cache for each of the template arguments combinations */
template<typename BV, typename T>
FCLShapeCache_Shape& GetShapeCache(void)
{
  static FCLShapeCache_Shape cache;
  return cache;
}

template<typename T1, typename T2>
struct IfSameType
{
  enum { value = 0 };
};

template<typename T>
struct IfSameType<T, T>
{
  enum { value = 1 };
};

template<typename BV, typename T>
FCLGeometryConstPtr createCollisionGeometry(const shapes::StaticShapeConstPtr &shape, const T *data)
{ 
  FCLShapeCache_StaticShape &cache = GetStaticShapeCache<BV, T>();
  
  boost::weak_ptr<const shapes::StaticShape> wptr(shape);
  {
    boost::mutex::scoped_lock slock(cache.lock_);
    std::map<boost::weak_ptr<const shapes::StaticShape>, FCLGeometryConstPtr>::const_iterator cache_it = cache.map_.find(wptr);
    if (cache_it != cache.map_.end())
      return cache_it->second;
  }
  
  fcl::CollisionGeometry* g = NULL;
  switch (shape->type)
  {
  case shapes::PLANE:
    {
      const shapes::Plane *p = static_cast<const shapes::Plane*>(shape.get());
      g = new fcl::Plane(p->a, p->b, p->c, p->d);
    }
    break;
  default:
    ROS_FATAL("This shape type (%d) is not supported using FCL yet", (int)shape->type);
  }
  if (g)
  {
    g->computeLocalAABB();
    FCLGeometryConstPtr res(new FCLGeometry(g, data));
    boost::mutex::scoped_lock slock(cache.lock_);
    cache.map_[wptr] = res;
    cache.clean_count_++;
    
    // clean-up for cache (we don't want to keep infinitely large number of weak ptrs stored)
    if (cache.clean_count_ > cache.MAX_CLEAN_COUNT)
    {
      cache.clean_count_ = 0;
      unsigned int from = cache.map_.size();
      for (std::map<boost::weak_ptr<const shapes::StaticShape>, FCLGeometryConstPtr>::iterator it = cache.map_.begin() ; it != cache.map_.end() ; )
      {
        std::map<boost::weak_ptr<const shapes::StaticShape>, FCLGeometryConstPtr>::iterator nit = it; ++nit;
        if (it->first.expired())
          cache.map_.erase(it);
        it = nit;
      }
      ROS_DEBUG("Cleaning up cache for FCL objects that correspond to static shapes. Cache size reduced from %u to %u", from, (unsigned int)cache.map_.size());
    }
    return res;
  }
  return FCLGeometryConstPtr();
}

template<typename BV, typename T>
FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, const T *data)
{
  FCLShapeCache_Shape &cache = GetShapeCache<BV, T>();
  
  boost::weak_ptr<const shapes::Shape> wptr(shape);
  {
    boost::mutex::scoped_lock slock(cache.lock_);
    std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::const_iterator cache_it = cache.map_.find(wptr);
    if (cache_it != cache.map_.end())
      return cache_it->second;
  }
  
  // attached objects could have previously been CollisionWorld::Object; we try to move them
  // from their old cache to the new one, if possible. the code is not pretty, but should help
  // when we attach/detach objects that are in the world
  if (IfSameType<T, planning_models::KinematicState::AttachedBody>::value == 1)
  {
    // get the cache that corresponds to objects; maybe this attached object used to be a world object
    FCLShapeCache_Shape &othercache = GetShapeCache<BV, CollisionWorld::Object>();

    // attached bodies could be just moved from the environment.
    othercache.lock_.lock(); // lock manually to avoid having 2 simultaneous locks active (avoids possible deadlock)
    std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::iterator cache_it = othercache.map_.find(wptr);
    if (cache_it != othercache.map_.end())
    {
      // this reinterpret_cast is safe because we checked the type above
      if (cache_it->second->collision_geometry_data_->getID() == reinterpret_cast<const planning_models::KinematicState::AttachedBody*>(data)->getName()
          && cache_it->second.unique())
      {
        // remove from old cache
        FCLGeometryConstPtr obj_cache = cache_it->second;
        othercache.map_.erase(cache_it);
        othercache.lock_.unlock();

        // update the CollisionGeometryData
        const_cast<FCLGeometry*>(obj_cache.get())->updateCollisionGeometryData(data);
        
        // add to the new cache
        boost::mutex::scoped_lock slock(cache.lock_);
        cache.map_[wptr] = obj_cache;
        cache.clean_count_++;
        return obj_cache;        
      }
    }
    othercache.lock_.unlock();
  }
  else
  // world objects could have previously been attached objects; we try to move them
  // from their old cache to the new one, if possible. the code is not pretty, but should help
  // when we attach/detach objects that are in the world
  if (IfSameType<T, CollisionWorld::Object>::value == 1)
  {
    // get the cache that corresponds to objects; maybe this attached object used to be a world object
    FCLShapeCache_Shape &othercache = GetShapeCache<BV, planning_models::KinematicState::AttachedBody>();

    // attached bodies could be just moved from the environment.
    othercache.lock_.lock(); // lock manually to avoid having 2 simultaneous locks active (avoids possible deadlock)
    std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::iterator cache_it = othercache.map_.find(wptr);
    if (cache_it != othercache.map_.end())
    {
      // this reinterpret_cast is safe because we checked the type above
      if (cache_it->second->collision_geometry_data_->getID() == reinterpret_cast<const CollisionWorld::Object*>(data)->id_ && 
          cache_it->second.unique())
      {
        // remove from old cache
        FCLGeometryConstPtr obj_cache = cache_it->second;
        othercache.map_.erase(cache_it);
        othercache.lock_.unlock();

        // update the CollisionGeometryData
        const_cast<FCLGeometry*>(obj_cache.get())->updateCollisionGeometryData(data);

        // add to the new cache
        boost::mutex::scoped_lock slock(cache.lock_);
        cache.map_[wptr] = obj_cache;
        cache.clean_count_++;
        return obj_cache;        
      }
    }
    othercache.lock_.unlock();
  }

  fcl::BVHModel<BV>* g = new fcl::BVHModel<BV>();
  
  switch (shape->type)
  {
  case shapes::SPHERE:
    {
      fcl::generateBVHModel(*g, fcl::Sphere(static_cast<const shapes::Sphere*>(shape.get())->radius));
    }
    break;
  case shapes::BOX:
    {
      const double *size = static_cast<const shapes::Box*>(shape.get())->size;
      fcl::generateBVHModel(*g, fcl::Box(size[0], size[1], size[2]));
    }
    break;
  case shapes::CYLINDER:
    {
      fcl::generateBVHModel(*g, fcl::Cylinder(static_cast<const shapes::Cylinder*>(shape.get())->radius,
                                              static_cast<const shapes::Cylinder*>(shape.get())->length));
    }
    break;
  case shapes::MESH:
    {
      const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape.get());
      if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
      {
        std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
        for(unsigned int i = 0; i < mesh->triangle_count; ++i)
          tri_indices[i] = fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);
        
        std::vector<fcl::Vec3f> points(mesh->vertex_count);
        for (unsigned int i = 0; i < mesh->vertex_count; ++i)
          points[i] = fcl::Vec3f(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
        
        g->beginModel();
        g->addSubModel(points, tri_indices);
        g->endModel();
      }
    }
    break;
  default:
    ROS_ERROR("This shape type (%d) is not supported using FCL yet", (int)shape->type);
  }
  if (g)
  {
    g->computeLocalAABB();
    FCLGeometryConstPtr res(new FCLGeometry(g, data));
    boost::mutex::scoped_lock slock(cache.lock_);
    cache.map_[wptr] = res;
    cache.clean_count_++;
    
    // clean-up for cache (we don't want to keep infinitely large number of weak ptrs stored)
    if (cache.clean_count_ > cache.MAX_CLEAN_COUNT)
    {
      cache.clean_count_ = 0;
      unsigned int from = cache.map_.size();
      for (std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::iterator it = cache.map_.begin() ; it != cache.map_.end() ; )
      {
        std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::iterator nit = it; ++nit;
        if (it->first.expired())
        {
          it->second.reset();
          cache.map_.erase(it);
        }
        it = nit;
      }
      ROS_DEBUG("Cleaning up cache for FCL objects that correspond to shapes. Cache size reduced from %u to %u", from, (unsigned int)cache.map_.size());
    }
    return res;
  }
  return FCLGeometryConstPtr();
} 



/////////////////////////////////////////////////////
FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::StaticShapeConstPtr &shape,
                                            const planning_models::KinematicModel::LinkModel *link)
{
  return obb ? createCollisionGeometry<fcl::OBB, planning_models::KinematicModel::LinkModel>(shape, link) :
    createCollisionGeometry<fcl::RSS, planning_models::KinematicModel::LinkModel>(shape, link);
}

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::StaticShapeConstPtr &shape,
                                            const planning_models::KinematicState::AttachedBody *ab)
{
  return obb ? createCollisionGeometry<fcl::OBB, planning_models::KinematicState::AttachedBody>(shape, ab) :
    createCollisionGeometry<fcl::RSS, planning_models::KinematicState::AttachedBody>(shape, ab);
}

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::StaticShapeConstPtr &shape,
                                            const CollisionWorld::Object *obj)
{
  return obb ? createCollisionGeometry<fcl::OBB, CollisionWorld::Object>(shape, obj) :
    createCollisionGeometry<fcl::RSS, CollisionWorld::Object>(shape, obj);
}

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape,
                                            const planning_models::KinematicModel::LinkModel *link)
{
  return obb ? createCollisionGeometry<fcl::OBB, planning_models::KinematicModel::LinkModel>(shape, link) :
    createCollisionGeometry<fcl::RSS, planning_models::KinematicModel::LinkModel>(shape, link);
}

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape,
                                            const planning_models::KinematicState::AttachedBody *ab)
{
  return obb ? createCollisionGeometry<fcl::OBB, planning_models::KinematicState::AttachedBody>(shape, ab) :
    createCollisionGeometry<fcl::RSS, planning_models::KinematicState::AttachedBody>(shape, ab);
}

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape,
                                            const CollisionWorld::Object *obj)
{
  return obb ? createCollisionGeometry<fcl::OBB, CollisionWorld::Object>(shape, obj) :
    createCollisionGeometry<fcl::RSS, CollisionWorld::Object>(shape, obj);
}

template<typename BV, typename T>
FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding, const T *data)
{
  if (fabs(scale - 1.0) <= std::numeric_limits<double>::epsilon() && fabs(padding) <= std::numeric_limits<double>::epsilon())
    return createCollisionGeometry<BV, T>(shape, data);
  else
  {
    boost::shared_ptr<shapes::Shape> scaled_shape(shape->clone());
    scaled_shape->scaleAndPadd(scale, padding);
    return createCollisionGeometry<BV, T>(scaled_shape, data);
  }
}

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const planning_models::KinematicModel::LinkModel *link)
{
  return obb ? createCollisionGeometry<fcl::OBB, planning_models::KinematicModel::LinkModel>(shape, scale, padding, link) :
    createCollisionGeometry<fcl::RSS, planning_models::KinematicModel::LinkModel>(shape, scale, padding, link);
}

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const planning_models::KinematicState::AttachedBody *ab)
{
  return obb ? createCollisionGeometry<fcl::OBB, planning_models::KinematicState::AttachedBody>(shape, scale, padding, ab) :
    createCollisionGeometry<fcl::RSS, planning_models::KinematicState::AttachedBody>(shape, scale, padding, ab);
}

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const CollisionWorld::Object *obj)
{
  return obb ? createCollisionGeometry<fcl::OBB, CollisionWorld::Object>(shape, scale, padding, obj) :
    createCollisionGeometry<fcl::RSS, CollisionWorld::Object>(shape, scale, padding, obj);
}

}

void collision_detection::FCLObject::registerTo(fcl::BroadPhaseCollisionManager *manager)
{
  for (std::size_t i = 0 ; i < collision_objects_.size() ; ++i)
    manager->registerObject(collision_objects_[i].get());
}

void collision_detection::FCLObject::unregisterFrom(fcl::BroadPhaseCollisionManager *manager)
{
  for (std::size_t i = 0 ; i < collision_objects_.size() ; ++i)
    manager->unregisterObject(collision_objects_[i].get());
}

void collision_detection::FCLObject::clear(void)
{
  collision_objects_.clear();
  collision_geometry_.clear();
}
