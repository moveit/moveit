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

/* Author: Ioan Sucan, Acorn Pooley */

#ifndef MOVEIT_COLLISION_DETECTION_COLLISION_DETECTION_ALLOC_
#define MOVEIT_COLLISION_DETECTION_COLLISION_DETECTION_ALLOC_

#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection/collision_robot.h>

namespace collision_detection
{
  /** \brief Represents a particular type of collision detection.
   * A type of collision detection is identified by a combination of a subclass
   * of CollisionWorld and a subclass of CollisionRobot.  Valid combinations
   * must have a static function getCollisionDetectorName() in the
   * CollisionWorld sublass which takes as argument a pointer instance of the
   * compatible CollisionRobot class. */
  struct CollisionDetectionAllocBase
  {
    virtual const std::string& getCollisionDetectorName() = 0;
    virtual CollisionRobotPtr allocateRobot(const robot_model::RobotModelConstPtr &kmodel) = 0;
    virtual CollisionRobotPtr allocateRobot(const CollisionRobotConstPtr &copy) = 0;
    virtual CollisionWorldPtr allocateWorld(const WorldPtr& world) = 0;
    virtual CollisionWorldPtr allocateWorld(const CollisionWorldConstPtr &copy, const WorldPtr& world) = 0;
  };
  typedef boost::shared_ptr<CollisionDetectionAllocBase> CollisionDetectionAllocBasePtr;

  /** \brief template for specific collision detection combinations. */
  template<class CollisionWorldType, class CollisionRobotType>
  struct CollisionDetectionAlloc : public CollisionDetectionAllocBase
  {
    BOOST_CONCEPT_ASSERT((boost::Convertible<CollisionWorldType*, CollisionWorld*>));
    BOOST_CONCEPT_ASSERT((boost::Convertible<CollisionRobotType*, CollisionRobot*>));

    CollisionDetectionAlloc()
    {
      // This verifies (at compile time) that the CollisionWorldType/CollisionRobotType combination is valid.
      // If you get a compile error then the CollisionWorldType and CollisionRobotType you passed in are not compatible.
      (void)CollisionWorldType::getCollisionDetectorName((CollisionRobotType*)0);
    }

    virtual const std::string& getCollisionDetectorName()
    {
      return CollisionWorldType::getCollisionDetectorName((CollisionRobotType*)0);
    }
    virtual CollisionRobotPtr allocateRobot(const robot_model::RobotModelConstPtr &kmodel)
    {
      return CollisionRobotPtr(new CollisionRobotType(kmodel));
    }
    virtual CollisionWorldPtr allocateWorld(const WorldPtr& world)
    {
      return CollisionWorldPtr(new CollisionWorldType(world));
    }
    virtual CollisionRobotPtr allocateRobot(const CollisionRobotConstPtr &copy)
    {
      return CollisionRobotPtr(new CollisionRobotType(dynamic_cast<const CollisionRobotType&>(*copy)));
    }
    virtual CollisionWorldPtr allocateWorld(const  CollisionWorldConstPtr &copy, const WorldPtr& world)
    {
      return CollisionWorldPtr(new CollisionWorldType(dynamic_cast<const CollisionWorldType&>(*copy), world));
    }
  };

}


#endif
