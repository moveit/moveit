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

bool planning_scene::PlanningScene::configure(const urdf::Model &urdf_model, const srdf::Model &srdf_model) 
{
    kmodel_.reset(new planning_models::KinematicModel(urdf_model, srdf_model));
    tf_.reset(new planning_models::Transforms(kmodel_->getModelFrame()));
    kstate_.reset(new planning_models::KinematicState(kmodel_));
    crobot_.reset(new collision_detection::CollisionRobotAllValid(kmodel_));
    cworld_.reset(new collision_detection::CollisionWorldAllValid());
    configured_ = true;
    return true;
}

void planning_scene::PlanningScene::addCollisionObject(const moveit_msgs::CollisionObject &object)
{
    if (object.shapes.empty())
    {
	ROS_ERROR("There are no shapes specified in the collision object message");
	return;
    }
    if (object.shapes.size() != object.poses.size())
    {
	ROS_ERROR("Number of shapes does not match number of poses in collision object message");
	return;	
    }
    if (object.operation != moveit_msgs::CollisionObject::ADD)
    {
	ROS_ERROR("Asked to add a collision object to the collision world, but the operation on that object is not ADD.");
	return;
    }
    
    
    for (std::size_t i = 0 ; i < object.shapes.size() ; ++i)
    {
	shapes::Shape *s = shapes::constructShapeFromMsg(object.shapes[i]);
	if (s)
	{
	    object.poses[i].position.x;
	    
	}
    }
    
}
