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

/** \author Ioan Sucan */

#include "collision_detection/collision_world.h"
#include <ros/console.h>

bool collision_detection::CollisionWorld::isCollision(const CollisionRobot &robot, const planning_models::KinematicState &state, std::vector<Contact> &contacts,
                                                      unsigned int max_total, unsigned int max_per_pair) const
{
    bool result = robot.isSelfCollision(state, contacts, max_total, max_per_pair);
    if (contacts.size() < max_total)
    {
        bool result2 = isWorldCollision(robot, state, contacts, max_total, max_per_pair);
        result = result && result2;
    }
    return result;
}

bool collision_detection::CollisionWorld::isCollision(const CollisionRobot &robot, const planning_models::KinematicState &state, const AllowedCollisionMatrix &acm,
                                                      std::vector<Contact> &contacts, unsigned int max_total, unsigned int max_per_pair) const
{
    bool result = robot.isSelfCollision(state, acm, contacts, max_total, max_per_pair);
    if (contacts.size() < max_total)
    {
        bool result2 = isWorldCollision(robot, state, acm, contacts, max_total, max_per_pair);
        result = result && result2;
    }
    return result;
}

void collision_detection::CollisionWorld::addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<btTransform> &poses)
{
    if (shapes.size() != poses.size())
        ROS_ERROR("Number of shapes and number of poses do not match. Not adding any objects to collision world.");
    else
        for (std::size_t i = 0 ; i < shapes.size() ; ++i)
            addObject(ns, shapes[i], poses[i]);
}
