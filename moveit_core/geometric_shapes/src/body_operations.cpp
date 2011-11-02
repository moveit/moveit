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

/** \author Ioan Sucan, E. Gil Jones */

#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>

bodies::Body* bodies::createBodyFromShape(const shapes::Shape *shape)
{
  Body *body = NULL;

  if (shape)
    switch (shape->type)
    {
    case shapes::BOX:
      body = new bodies::Box(shape);
      break;
    case shapes::SPHERE:
      body = new bodies::Sphere(shape);
      break;
    case shapes::CYLINDER:
      body = new bodies::Cylinder(shape);
      break;
    case shapes::MESH:
      body = new bodies::ConvexMesh(shape);
      break;
    default:
      ROS_ERROR("Creating body from shape: Unknown shape type %d", (int)shape->type);
      break;
    }

  return body;
}

void bodies::mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres, BoundingSphere &mergedSphere)
{
    if (spheres.empty())
    {
        mergedSphere.center.setValue(btScalar(0), btScalar(0), btScalar(0));
        mergedSphere.radius = 0.0;
    }
    else
    {
        mergedSphere = spheres[0];
        for (unsigned int i = 1 ; i < spheres.size() ; ++i)
        {
            if (spheres[i].radius <= 0.0)
                continue;
            double d = spheres[i].center.distance(mergedSphere.center);
            if (d + mergedSphere.radius <= spheres[i].radius)
            {
                mergedSphere.center = spheres[i].center;
                mergedSphere.radius = spheres[i].radius;
            }
            else
                if (d + spheres[i].radius > mergedSphere.radius)
                {
                    btVector3 delta = mergedSphere.center - spheres[i].center;
                    mergedSphere.radius = (delta.length() + spheres[i].radius + mergedSphere.radius)/2.0;
                    mergedSphere.center = delta.normalized() * (mergedSphere.radius - spheres[i].radius) + spheres[i].center;
                }
        }
    }
}

bodies::Body* bodies::constructBodyFromMsg(const moveit_msgs::Shape &shape_msg, const geometry_msgs::Pose &pose)
{
    shapes::Shape *shape = shapes::constructShapeFromMsg(shape_msg);
    if (shape)
    {
        Body *body = createBodyFromShape(shape);
        if (body)
        {
            btQuaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            if (fabs(q.length2() - 1.0) > 1e-3)
            {
                ROS_ERROR("Quaternion is not normalized. Assuming identity.");
                q = btQuaternion(0.0, 0.0, 0.0, 1.0);
            }
            body->setPose(btTransform(q,btVector3(pose.position.x, pose.position.y, pose.position.z)));
            return body;
        }
    }

    return NULL;
}
