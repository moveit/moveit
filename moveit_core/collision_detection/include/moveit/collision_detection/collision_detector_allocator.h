/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Acorn Pooley, Ioan Sucan */

#ifndef MOVEIT_COLLISION_DETECTION_COLLISION_DETECTOR_
#define MOVEIT_COLLISION_DETECTION_COLLISION_DETECTOR_

#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>

namespace collision_detection
{

  /** \brief An allocator for a compatible CollisionWorld/CollisionRobot pair. */
  class CollisionDetectorAllocator
  {
  public:
    /** A unique name identifying the CollisionWorld/CollisionRobot pairing. */
    virtual const std::string& getName() const = 0;

    /** create a new CollisionWorld for checking collisions with the supplied world. */
    virtual CollisionWorldPtr allocateWorld(const WorldPtr& world) const = 0;

    /** create a new CollisionWorld by copying an existing CollisionWorld of the same type.s
     * The world must be either the same world as used by \orig or a copy of that world which has not yet been modified. */
    virtual CollisionWorldPtr allocateWorld(const CollisionWorldConstPtr& orig, const WorldPtr& world) const = 0;

    /** create a new CollisionRobot given a robot_model */
    virtual CollisionRobotPtr allocateRobot(const robot_model::RobotModelConstPtr& robot_model) const = 0;

    /** create a new CollisionRobot by copying an existing CollisionRobot of the same type. */
    virtual CollisionRobotPtr allocateRobot(const CollisionRobotConstPtr& orig) const = 0;
  };

  typedef boost::shared_ptr<CollisionDetectorAllocator> CollisionDetectorAllocatorPtr;



  /** \brief Template class to make it easy to create an allocator for a specific CollisionWorld/CollisionRobot pair. */
  template<class CollisionWorldType, class CollisionRobotType, class CollisionDetectorAllocatorType>
  class CollisionDetectorAllocatorTemplate : public CollisionDetectorAllocator
  {
  public:
    virtual const std::string& getName() const
    {
      return CollisionDetectorAllocatorType::NAME_;
    }

    virtual CollisionWorldPtr allocateWorld(const WorldPtr& world) const
    {
      return CollisionWorldPtr(new CollisionWorldType(world));
    }

    virtual CollisionWorldPtr allocateWorld(const CollisionWorldConstPtr& orig, const WorldPtr& world) const
    {
      return CollisionWorldPtr(new CollisionWorldType(dynamic_cast<const CollisionWorldType&>(*orig), world));
    }

    virtual CollisionRobotPtr allocateRobot(const robot_model::RobotModelConstPtr& robot_model) const
    {
      return CollisionRobotPtr(new CollisionRobotType(robot_model));
    }

    virtual CollisionRobotPtr allocateRobot(const CollisionRobotConstPtr& orig) const
    {
      return CollisionRobotPtr(new CollisionRobotType(dynamic_cast<const CollisionRobotType&>(*orig)));
    }

    /** Create an allocator for FCL collision detectors */
    static CollisionDetectorAllocatorPtr create()
    {
      return CollisionDetectorAllocatorPtr(new CollisionDetectorAllocatorType());
    }
  };

}

#endif
