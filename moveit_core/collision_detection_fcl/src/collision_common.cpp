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

#include <moveit/collision_detection_fcl/collision_common.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/octree.h>
#include <boost/thread/mutex.hpp>

namespace collision_detection
{

bool collisionCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void *data)
{
  CollisionData *cdata = reinterpret_cast<CollisionData*>(data);
  if (cdata->done_)
    return true;
  const CollisionGeometryData *cd1 = static_cast<const CollisionGeometryData*>(o1->getCollisionGeometry()->getUserData());
  const CollisionGeometryData *cd2 = static_cast<const CollisionGeometryData*>(o2->getCollisionGeometry()->getUserData());

  // If active components are specified
  if (cdata->active_components_only_)
  {
    const robot_model::LinkModel *l1 = cd1->type == BodyTypes::ROBOT_LINK ? cd1->ptr.link : (cd1->type == BodyTypes::ROBOT_ATTACHED ? cd1->ptr.ab->getAttachedLink() : NULL);
    const robot_model::LinkModel *l2 = cd2->type == BodyTypes::ROBOT_LINK ? cd2->ptr.link : (cd2->type == BodyTypes::ROBOT_ATTACHED ? cd2->ptr.ab->getAttachedLink() : NULL);

    // If neither of the involved components is active
    if ((!l1 || cdata->active_components_only_->find(l1) == cdata->active_components_only_->end()) &&
        (!l2 || cdata->active_components_only_->find(l2) == cdata->active_components_only_->end()))
      return false;
  }

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
          logDebug("Collision between '%s' (type '%s') and '%s' (type '%s') is always allowed. No contacts are computed.",
                   cd1->getID().c_str(),
                   cd1->getTypeString().c_str(),
                   cd2->getID().c_str(),
                   cd2->getTypeString().c_str());
      }
      else
        if (type == AllowedCollision::CONDITIONAL)
        {
          cdata->acm_->getAllowedCollision(cd1->getID(), cd2->getID(), dcf);
          if (cdata->req_->verbose)
            logDebug("Collision between '%s' and '%s' is conditionally allowed", cd1->getID().c_str(), cd2->getID().c_str());
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
        logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
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
          logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                   cd2->getID().c_str(), cd1->getID().c_str());
      }
    }
  // bodies attached to the same link should not collide
  if (cd1->type == BodyTypes::ROBOT_ATTACHED && cd2->type == BodyTypes::ROBOT_ATTACHED)
  {
    if (cd1->ptr.ab->getAttachedLink() == cd2->ptr.ab->getAttachedLink())
      always_allow_collision = true;
  }

  // if collisions are always allowed, we are done
  if (always_allow_collision)
    return false;

  if (cdata->req_->verbose)
    logDebug("Actually checking collisions between %s and %s", cd1->getID().c_str(), cd2->getID().c_str());

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
        want_contact_count = std::min(cdata->req_->max_contacts_per_pair - have, cdata->req_->max_contacts - cdata->res_->contact_count);
    }

  if (dcf)
  {
    // if we have a decider for allowed contacts, we need to look at all the contacts
    bool enable_cost = cdata->req_->cost;
    std::size_t num_max_cost_sources = cdata->req_->max_cost_sources;
    bool enable_contact = true;
    fcl::CollisionResult col_result;
    int num_contacts = fcl::collide(o1, o2, fcl::CollisionRequest(std::numeric_limits<size_t>::max(), enable_contact, num_max_cost_sources, enable_cost), col_result);
    if (num_contacts > 0)
    {
      if (cdata->req_->verbose)
        logInform("Found %d contacts between '%s' and '%s'. These contacts will be evaluated to check if they are accepted or not",
                  num_contacts, cd1->getID().c_str(), cd2->getID().c_str());
      Contact c;
      const std::pair<std::string, std::string> &pc = cd1->getID() < cd2->getID() ?
        std::make_pair(cd1->getID(), cd2->getID()) : std::make_pair(cd2->getID(), cd1->getID());
      for (int i = 0 ; i < num_contacts ; ++i)
      {
        fcl2contact(col_result.getContact(i), c);
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
              logInform("Found unacceptable contact between '%s' and '%s'. Contact was stored.",
                        cd1->getID().c_str(), cd2->getID().c_str());
          }
          else
            if (cdata->req_->verbose)
              logInform("Found unacceptable contact between '%s' (type '%s') and '%s' (type '%s'). Contact was stored.",
                        cd1->getID().c_str(), cd1->getTypeString().c_str(),
                        cd2->getID().c_str(), cd2->getTypeString().c_str());
          cdata->res_->collision = true;
          if (want_contact_count == 0)
            break;
        }
      }
    }

    if (enable_cost)
    {
      std::vector<fcl::CostSource> cost_sources;
      col_result.getCostSources(cost_sources);

      CostSource cs;
      for (std::size_t i = 0; i < cost_sources.size(); ++i)
      {
        fcl2costsource(cost_sources[i], cs);
        cdata->res_->cost_sources.insert(cs);
        while (cdata->res_->cost_sources.size() > cdata->req_->max_cost_sources)
          cdata->res_->cost_sources.erase(--cdata->res_->cost_sources.end());
      }
    }
  }
  else
  {
    if (want_contact_count > 0)
    {
      // otherwise, we need to compute more things
      bool enable_cost = cdata->req_->cost;
      std::size_t num_max_cost_sources = cdata->req_->max_cost_sources;
      bool enable_contact = true;

      fcl::CollisionResult col_result;
      int num_contacts = fcl::collide(o1, o2, fcl::CollisionRequest(want_contact_count, enable_contact, num_max_cost_sources, enable_cost), col_result);
      if (num_contacts > 0)
      {
        int num_contacts_initial = num_contacts;

        // make sure we don't get more contacts than we want
        if (want_contact_count >= (std::size_t)num_contacts)
          want_contact_count -= num_contacts;
        else
        {
          num_contacts = want_contact_count;
          want_contact_count = 0;
        }

        if (cdata->req_->verbose)
          logInform("Found %d contacts between '%s' (type '%s') and '%s' (type '%s'), which constitute a collision. %d contacts will be stored",
                    num_contacts_initial,
                    cd1->getID().c_str(), cd1->getTypeString().c_str(),
                    cd2->getID().c_str(), cd2->getTypeString().c_str(),
                    num_contacts);

        const std::pair<std::string, std::string> &pc = cd1->getID() < cd2->getID() ?
          std::make_pair(cd1->getID(), cd2->getID()) : std::make_pair(cd2->getID(), cd1->getID());
        cdata->res_->collision = true;
        for (int i = 0 ; i < num_contacts ; ++i)
        {
          Contact c;
          fcl2contact(col_result.getContact(i), c);
          cdata->res_->contacts[pc].push_back(c);
          cdata->res_->contact_count++;
        }
      }

      if (enable_cost)
      {
        std::vector<fcl::CostSource> cost_sources;
        col_result.getCostSources(cost_sources);

        CostSource cs;
        for (std::size_t i = 0; i < cost_sources.size(); ++i)
        {
          fcl2costsource(cost_sources[i], cs);
          cdata->res_->cost_sources.insert(cs);
          while (cdata->res_->cost_sources.size() > cdata->req_->max_cost_sources)
            cdata->res_->cost_sources.erase(--cdata->res_->cost_sources.end());
        }
      }
    }
    else
    {
      bool enable_cost = cdata->req_->cost;
      std::size_t num_max_cost_sources = cdata->req_->max_cost_sources;
      bool enable_contact = false;
      fcl::CollisionResult col_result;
      int num_contacts = fcl::collide(o1, o2, fcl::CollisionRequest(1, enable_contact, num_max_cost_sources, enable_cost), col_result);
      if (num_contacts > 0)
      {
        cdata->res_->collision = true;
        if (cdata->req_->verbose)
          logInform("Found a contact between '%s' (type '%s') and '%s' (type '%s'), which constitutes a collision. Contact information is not stored.",
                    cd1->getID().c_str(), cd1->getTypeString().c_str(), cd2->getID().c_str(), cd2->getTypeString().c_str());
      }

      if (enable_cost)
      {
        std::vector<fcl::CostSource> cost_sources;
        col_result.getCostSources(cost_sources);

        CostSource cs;
        for (std::size_t i = 0; i < cost_sources.size(); ++i)
        {
          fcl2costsource(cost_sources[i], cs);
          cdata->res_->cost_sources.insert(cs);
          while (cdata->res_->cost_sources.size() > cdata->req_->max_cost_sources)
            cdata->res_->cost_sources.erase(--cdata->res_->cost_sources.end());
        }
      }
    }
  }


  if (cdata->res_->collision)
    if (!cdata->req_->contacts || cdata->res_->contact_count >= cdata->req_->max_contacts)
    {
      if (!cdata->req_->cost)
        cdata->done_ = true;
      if (cdata->req_->verbose)
        logInform("Collision checking is considered complete (collision was found and %u contacts are stored)",
                  (unsigned int)cdata->res_->contact_count);
    }

  if (!cdata->done_ && cdata->req_->is_done)
  {
    cdata->done_ = cdata->req_->is_done(*cdata->res_);
    if (cdata->done_ && cdata->req_->verbose)
      logInform("Collision checking is considered complete due to external callback. %s was found. %u contacts are stored.",
                cdata->res_->collision ? "Collision" : "No collision", (unsigned int)cdata->res_->contact_count);
  }

  return cdata->done_;
}

struct FCLShapeCache
{
  FCLShapeCache() : clean_count_(0) {}

  void bumpUseCount(bool force = false)
  {
    clean_count_++;

    // clean-up for cache (we don't want to keep infinitely large number of weak ptrs stored)
    if (clean_count_ > MAX_CLEAN_COUNT || force)
    {
      clean_count_ = 0;
      unsigned int from = map_.size();
      for (std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::iterator it = map_.begin() ; it != map_.end() ; )
      {
        std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::iterator nit = it; ++nit;
        if (it->first.expired())
          map_.erase(it);
        it = nit;
      }
      //      logDebug("Cleaning up cache for FCL objects that correspond to static shapes. Cache size reduced from %u to %u", from, (unsigned int)map_.size());
    }
  }

  static const unsigned int MAX_CLEAN_COUNT = 100; // every this many uses of the cache, a cleaning operation is executed (this is only removal of expired entries)
  std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr> map_;
  unsigned int clean_count_;
  boost::mutex lock_;
};


bool distanceCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data, double& min_dist)
{
  CollisionData* cdata = reinterpret_cast<CollisionData*>(data);

  const CollisionGeometryData* cd1 = static_cast<const CollisionGeometryData*>(o1->getCollisionGeometry()->getUserData());
  const CollisionGeometryData* cd2 = static_cast<const CollisionGeometryData*>(o2->getCollisionGeometry()->getUserData());

  // If active components are specified
  if (cdata->active_components_only_)
  {
    const robot_model::LinkModel *l1 = cd1->type == BodyTypes::ROBOT_LINK ? cd1->ptr.link : (cd1->type == BodyTypes::ROBOT_ATTACHED ? cd1->ptr.ab->getAttachedLink() : NULL);
    const robot_model::LinkModel *l2 = cd2->type == BodyTypes::ROBOT_LINK ? cd2->ptr.link : (cd2->type == BodyTypes::ROBOT_ATTACHED ? cd2->ptr.ab->getAttachedLink() : NULL);

    // If neither of the involved components is active
    if ((!l1 || cdata->active_components_only_->find(l1) == cdata->active_components_only_->end()) &&
        (!l2 || cdata->active_components_only_->find(l2) == cdata->active_components_only_->end()))
    {
      min_dist = cdata->res_->distance;
      return cdata->done_;
    }
  }

  // use the collision matrix (if any) to avoid certain distance checks
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
          logDebug("Collision between '%s' and '%s' is always allowed. No contacts are computed.",
                   cd1->getID().c_str(), cd2->getID().c_str());
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
        logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                 cd1->getID().c_str(), cd2->getID().c_str());
    }
  }
  else
  {
    if (cd2->type == BodyTypes::ROBOT_LINK && cd1->type == BodyTypes::ROBOT_ATTACHED)
    {
      const std::set<std::string> &tl = cd1->ptr.ab->getTouchLinks();
      if (tl.find(cd2->getID()) != tl.end())
      {
        always_allow_collision = true;
        if (cdata->req_->verbose)
          logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                   cd2->getID().c_str(), cd1->getID().c_str());
      }
    }
  }

  if(always_allow_collision)
  {
    min_dist = cdata->res_->distance;
    return cdata->done_;
  }

  if (cdata->req_->verbose)
    logDebug("Actually checking collisions between %s and %s", cd1->getID().c_str(), cd2->getID().c_str());

  fcl::DistanceResult dist_result;
  dist_result.update(cdata->res_->distance, NULL, NULL, fcl::DistanceResult::NONE, fcl::DistanceResult::NONE); // can be faster
  double d = fcl::distance(o1, o2, fcl::DistanceRequest(), dist_result);

  if(d < 0)
  {
    cdata->done_ = true;
    cdata->res_->distance = -1;
  }
  else
  {
    if(cdata->res_->distance > d)
      cdata->res_->distance = d;
  }

  min_dist = cdata->res_->distance;

  return cdata->done_;
}

/* We template the function so we get a different cache for each of the template arguments combinations */
template<typename BV, typename T>
FCLShapeCache& GetShapeCache()
{
  static FCLShapeCache cache;
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
FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, const T *data)
{
  FCLShapeCache &cache = GetShapeCache<BV, T>();

  boost::weak_ptr<const shapes::Shape> wptr(shape);
  {
    boost::mutex::scoped_lock slock(cache.lock_);
    std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::const_iterator cache_it = cache.map_.find(wptr);
    if (cache_it != cache.map_.end())
    {
      if (cache_it->second->collision_geometry_data_->ptr.raw == (void*)data)
      {
        //        logDebug("Collision data structures for object %s retrieved from cache.", cache_it->second->collision_geometry_data_->getID().c_str());
        return cache_it->second;
      }
      else
        if (cache_it->second.unique())
        {
          const_cast<FCLGeometry*>(cache_it->second.get())->updateCollisionGeometryData(data, false);
          //          logDebug("Collision data structures for object %s retrieved from cache after updating the source object.", cache_it->second->collision_geometry_data_->getID().c_str());
          return cache_it->second;
        }
    }
  }

  // attached objects could have previously been World::Object; we try to move them
  // from their old cache to the new one, if possible. the code is not pretty, but should help
  // when we attach/detach objects that are in the world
  if (IfSameType<T, robot_state::AttachedBody>::value == 1)
  {
    // get the cache that corresponds to objects; maybe this attached object used to be a world object
    FCLShapeCache &othercache = GetShapeCache<BV, World::Object>();

    // attached bodies could be just moved from the environment.
    othercache.lock_.lock(); // lock manually to avoid having 2 simultaneous locks active (avoids possible deadlock)
    std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::iterator cache_it = othercache.map_.find(wptr);
    if (cache_it != othercache.map_.end())
    {
      if (cache_it->second.unique())
      {
        // remove from old cache
        FCLGeometryConstPtr obj_cache = cache_it->second;
        othercache.map_.erase(cache_it);
        othercache.lock_.unlock();

        // update the CollisionGeometryData; nobody has a pointer to this, so we can safely modify it
        const_cast<FCLGeometry*>(obj_cache.get())->updateCollisionGeometryData(data, true);

        //        logDebug("Collision data structures for attached body %s retrieved from the cache for world objects.", obj_cache->collision_geometry_data_->getID().c_str());

        // add to the new cache
        boost::mutex::scoped_lock slock(cache.lock_);
        cache.map_[wptr] = obj_cache;
        cache.bumpUseCount();
        return obj_cache;
      }
    }
    othercache.lock_.unlock();
  }
  else
    // world objects could have previously been attached objects; we try to move them
    // from their old cache to the new one, if possible. the code is not pretty, but should help
    // when we attach/detach objects that are in the world
    if (IfSameType<T, World::Object>::value == 1)
    {
      // get the cache that corresponds to objects; maybe this attached object used to be a world object
      FCLShapeCache &othercache = GetShapeCache<BV, robot_state::AttachedBody>();

      // attached bodies could be just moved from the environment.
      othercache.lock_.lock(); // lock manually to avoid having 2 simultaneous locks active (avoids possible deadlock)
      std::map<boost::weak_ptr<const shapes::Shape>, FCLGeometryConstPtr>::iterator cache_it = othercache.map_.find(wptr);
      if (cache_it != othercache.map_.end())
      {
        if (cache_it->second.unique())
        {
          // remove from old cache
          FCLGeometryConstPtr obj_cache = cache_it->second;
          othercache.map_.erase(cache_it);
          othercache.lock_.unlock();

          // update the CollisionGeometryData; nobody has a pointer to this, so we can safely modify it
          const_cast<FCLGeometry*>(obj_cache.get())->updateCollisionGeometryData(data, true);

          //          logDebug("Collision data structures for world object %s retrieved from the cache for attached bodies.",
          //                   obj_cache->collision_geometry_data_->getID().c_str());

          // add to the new cache
          boost::mutex::scoped_lock slock(cache.lock_);
          cache.map_[wptr] = obj_cache;
          cache.bumpUseCount();
          return obj_cache;
        }
      }
      othercache.lock_.unlock();
    }

  fcl::CollisionGeometry* cg_g = NULL;
  if (shape->type == shapes::PLANE) // shapes that directly produce CollisionGeometry
  {
    // handle cases individually
    switch (shape->type)
    {
    case shapes::PLANE:
      {
        const shapes::Plane* p = static_cast<const shapes::Plane*>(shape.get());
        cg_g = new fcl::Plane(p->a, p->b, p->c, p->d);
      }
      break;
    default:
      break;
    }
  }
  else
  {
    switch (shape->type)
    {
    case shapes::SPHERE:
      {
        const shapes::Sphere* s = static_cast<const shapes::Sphere*>(shape.get());
        cg_g = new fcl::Sphere(s->radius);
      }
      break;
    case shapes::BOX:
      {
        const shapes::Box* s = static_cast<const shapes::Box*>(shape.get());
        const double* size = s->size;
        cg_g = new fcl::Box(size[0], size[1], size[2]);
      }
      break;
    case shapes::CYLINDER:
      {
        const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(shape.get());
        cg_g = new fcl::Cylinder(s->radius, s->length);
      }
      break;
    case shapes::CONE:
      {
        const shapes::Cone* s = static_cast<const shapes::Cone*>(shape.get());
        cg_g = new fcl::Cone(s->radius, s->length);
      }
      break;
    case shapes::MESH:
      {
        fcl::BVHModel<BV>* g = new fcl::BVHModel<BV>();
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
        cg_g = g;
      }
      break;
    case shapes::OCTREE:
      {
        const shapes::OcTree* g = static_cast<const shapes::OcTree*>(shape.get());
        cg_g = new fcl::OcTree(g->octree);
      }
      break;
    default:
      logError("This shape type (%d) is not supported using FCL yet", (int)shape->type);
      cg_g = NULL;
    }
  }
  if (cg_g)
  {
    cg_g->computeLocalAABB();
    FCLGeometryConstPtr res(new FCLGeometry(cg_g, data));
    boost::mutex::scoped_lock slock(cache.lock_);
    cache.map_[wptr] = res;
    cache.bumpUseCount();
    return res;
  }
  return FCLGeometryConstPtr();
}


/////////////////////////////////////////////////////
FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                            const robot_model::LinkModel *link)
{
  return createCollisionGeometry<fcl::OBBRSS, robot_model::LinkModel>(shape, link);
}

FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                            const robot_state::AttachedBody *ab)
{
  return createCollisionGeometry<fcl::OBBRSS, robot_state::AttachedBody>(shape, ab);
}

FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                            const World::Object *obj)
{
  return createCollisionGeometry<fcl::OBBRSS, World::Object>(shape, obj);
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

FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const robot_model::LinkModel *link)
{
  return createCollisionGeometry<fcl::OBBRSS, robot_model::LinkModel>(shape, scale, padding, link);
}

FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const robot_state::AttachedBody *ab)
{
  return createCollisionGeometry<fcl::OBBRSS, robot_state::AttachedBody>(shape, scale, padding, ab);
}

FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const World::Object *obj)
{
  return createCollisionGeometry<fcl::OBBRSS, World::Object>(shape, scale, padding, obj);
}

void cleanCollisionGeometryCache()
{
  FCLShapeCache &cache1 = GetShapeCache<fcl::OBBRSS, World::Object>();
  {
    boost::mutex::scoped_lock slock(cache1.lock_);
    cache1.bumpUseCount(true);
  }
  FCLShapeCache &cache2 = GetShapeCache<fcl::OBBRSS, robot_state::AttachedBody>();
  {
    boost::mutex::scoped_lock slock(cache2.lock_);
    cache2.bumpUseCount(true);
  }
}

}

void collision_detection::CollisionData::enableGroup(const robot_model::RobotModelConstPtr &kmodel)
{
  if (kmodel->hasJointModelGroup(req_->group_name))
    active_components_only_ = &kmodel->getJointModelGroup(req_->group_name)->getUpdatedLinkModelsWithGeometrySet();
  else
    active_components_only_ = NULL;
}

void collision_detection::FCLObject::registerTo(fcl::BroadPhaseCollisionManager *manager)
{
  std::vector<fcl::CollisionObject*> collision_objects(collision_objects_.size());
  for(std::size_t i = 0; i < collision_objects_.size(); ++i)
    collision_objects[i] = collision_objects_[i].get();
  if (collision_objects.size() > 0)
    manager->registerObjects(collision_objects);
}

void collision_detection::FCLObject::unregisterFrom(fcl::BroadPhaseCollisionManager *manager)
{
  for (std::size_t i = 0 ; i < collision_objects_.size() ; ++i)
    manager->unregisterObject(collision_objects_[i].get());
}

void collision_detection::FCLObject::clear()
{
  collision_objects_.clear();
  collision_geometry_.clear();
}
