/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/** \author Ioan Sucan */

#include "collision_detection/collision_objects.h"
#include <geometric_shapes/shape_operations.h>

std::vector<std::string> collision_detection::CollisionObjects::getNamespaces(void) const
{
    std::vector<std::string> ns;
    for (std::map<std::string, NamespaceObjects>::const_iterator it = objects_.begin() ; it != objects_.end() ; ++it)
        ns.push_back(it->first);
    return ns;
}

const collision_detection::CollisionObjects::NamespaceObjects& collision_detection::CollisionObjects::getObjects(const std::string &ns) const
{
    static NamespaceObjects empty;
    std::map<std::string, NamespaceObjects>::const_iterator it = objects_.find(ns);
    if (it == objects_.end())
        return empty;
    else
        return it->second;
}

collision_detection::CollisionObjects::NamespaceObjects& collision_detection::CollisionObjects::getObjects(const std::string &ns)
{
    return objects_[ns];
}

bool collision_detection::CollisionObjects::haveNamespace(const std::string &ns) const
{
    return objects_.find(ns) != objects_.end();
}

void collision_detection::CollisionObjects::addObject(const std::string &ns, shapes::StaticShape *shape)
{
    objects_[ns].static_shape.push_back(shape);
}

void collision_detection::CollisionObjects::addObject(const std::string &ns, shapes::Shape *shape, const btTransform &pose)
{
    objects_[ns].shape.push_back(shape);
    objects_[ns].shape_pose.push_back(pose);
}

bool collision_detection::CollisionObjects::moveObject(const std::string &ns, const shapes::Shape *shape, const btTransform &pose)
{
    std::map<std::string, NamespaceObjects>::iterator it = objects_.find(ns);
    if (it != objects_.end())
    {
        unsigned int n = it->second.shape.size();
        for (unsigned int i = 0 ; i < n ; ++i)
            if (it->second.shape[i] == shape)
            {
                it->second.shape_pose[i] = pose;
                return true;
            }
    }
    return false;
}

bool collision_detection::CollisionObjects::removeObject(const std::string &ns, const shapes::Shape *shape)
{
    std::map<std::string, NamespaceObjects>::iterator it = objects_.find(ns);
    if (it != objects_.end())
    {
        unsigned int n = it->second.shape.size();
        for (unsigned int i = 0 ; i < n ; ++i)
            if (it->second.shape[i] == shape)
            {
                it->second.shape.erase(it->second.shape.begin() + i);
                it->second.shape_pose.erase(it->second.shape_pose.begin() + i);
                return true;
            }
    }
    return false;
}

bool collision_detection::CollisionObjects::removeObject(const std::string &ns, const shapes::StaticShape *shape)
{
    std::map<std::string, NamespaceObjects>::iterator it = objects_.find(ns);
    if (it != objects_.end())
    {
        unsigned int n = it->second.static_shape.size();
        for (unsigned int i = 0 ; i < n ; ++i)
            if (it->second.static_shape[i] == shape)
            {
                it->second.static_shape.erase(it->second.static_shape.begin() + i);
                return true;
            }
    }
    return false;
}

void collision_detection::CollisionObjects::clearObjects(const std::string &ns)
{
    std::map<std::string, NamespaceObjects>::iterator it = objects_.find(ns);
    if (it != objects_.end())
    {
        unsigned int n = it->second.static_shape.size();
        for (unsigned int i = 0 ; i < n ; ++i)
            delete it->second.static_shape[i];
        n = it->second.shape.size();
        for (unsigned int i = 0 ; i < n ; ++i)
            delete it->second.shape[i];
        objects_.erase(it);
    }
}

void collision_detection::CollisionObjects::clearObjects(void)
{
    std::vector<std::string> ns = getNamespaces();
    for (unsigned int i = 0 ; i < ns.size() ; ++i)
        clearObjects(ns[i]);
}
