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

/* Author Ioan Sucan, Jia Pan */

#include "collision_detection/fcl/collision_robot.h"
#include <ros/console.h>
#include <fcl/geometric_shape_to_BVH_model.h>
#include <fcl/collision.h>

collision_detection::CollisionRobotFCL::CollisionRobotFCL(const planning_models::KinematicModelPtr &kmodel, double padding, double scale) : CollisionRobot(kmodel, padding, scale)
{
    geom_manager_.reset(new fcl::SSaPCollisionManager());
    const std::vector<planning_models::KinematicModel::LinkModel*>& links = kmodel_->getLinkModels();

    for (std::size_t i = 0 ; i < links.size() ; ++i)
        if (links[i] && links[i]->getShape())
        {
            fcl::CollisionObject* co = createCollisionObject(links[i]->getShape().get(), getLinkScale(links[i]->getName()), getLinkPadding(links[i]->getName()));
            if (co)
            {
                geom_manager_->registerObject(co);
                geom_map_[links[i]->getName()] = co;
            }
        }
}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state) const
{
    // do I need to call update() or setup() first?
    // how do I specify the pose of the system?
    //    geom_manager_->collide();

    // Is it cheap to construct a collision manager for different poses at each collision check?

}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state, const AllowedCollisionMatrix &acm) const
{
}

void collision_detection::CollisionRobotFCL::updatedPaddingOrScaling(const std::vector<std::string> &links)
{
    for (std::size_t i = 0 ; i < links.size() ; ++i)
    {
        std::map<std::string, fcl::CollisionObject*>::const_iterator it = geom_map_.find(links[i]);
        const planning_models::KinematicModel::LinkModel *lmodel = kmodel_->getLinkModel(links[i]);
        if (it != geom_map_.end() && lmodel)
        {
            geom_manager_->unregisterObject(it->second);
            fcl::CollisionObject* co = createCollisionObject(lmodel->getShape().get(), getLinkScale(links[i]), getLinkPadding(links[i]));
            geom_manager_->registerObject(co);
            geom_map_[links[i]] = co;
        }
        else
            ROS_ERROR("Updating padding or scaling for unknown link: '%s'", links[i].c_str());
    }
}

fcl::CollisionObject* collision_detection::CollisionRobotFCL::createCollisionObject(const shapes::StaticShape *shape, double scale, double padding) const
{
    fcl::CollisionObject* g = NULL;
    switch (shape->type)
    {
    case shapes::PLANE:
        {
            // TODO: plane implementation
            ROS_FATAL_STREAM("Plane is not supported using FCL yet");
        }
        break;
    default:
        break;
    }
    return g;
}

fcl::CollisionObject* collision_detection::CollisionRobotFCL::createCollisionObject(const shapes::Shape *shape, double scale, double padding) const
{
    fcl::BVHModel<fcl::OBB>* g = new fcl::BVHModel<fcl::OBB>();

    switch (shape->type)
    {
    case shapes::SPHERE:
        {
            fcl::generateBVHModel(*g, fcl::Sphere(static_cast<const shapes::Sphere*>(shape)->radius * scale + padding));
        }
        break;
    case shapes::BOX:
        {
            const double *size = static_cast<const shapes::Box*>(shape)->size;
            fcl::generateBVHModel(*g, fcl::Box(size[0] * scale + padding * 2.0, size[1] * scale + padding * 2.0, size[2] * scale + padding * 2.0));
        }
        break;
    case shapes::CYLINDER:
        {
            fcl::generateBVHModel(*g, fcl::Cylinder(static_cast<const shapes::Cylinder*>(shape)->radius * scale + padding,
                                                    static_cast<const shapes::Cylinder*>(shape)->length * scale + padding * 2.0));
        }
        break;
    case shapes::MESH:
        {
            const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);
            if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
            {
                std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
                for(unsigned int i = 0; i < mesh->triangle_count; ++i)
                    tri_indices[i] = fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

                std::vector<fcl::Vec3f> points(mesh->vertex_count);
                double sx = 0.0, sy = 0.0, sz = 0.0;
                for (unsigned int i = 0; i < mesh->vertex_count; ++i)
                {
                    points[i] = fcl::Vec3f(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
                    sx += points[i][0];
                    sy += points[i][1];
                    sz += points[i][2];
                }
                // the center of the mesh
                sx /= (double)mesh->vertex_count;
                sy /= (double)mesh->vertex_count;
                sz /= (double)mesh->vertex_count;

                // scale the mesh
                for(unsigned int i = 0; i < mesh->vertex_count; ++i)
                {
                    // vector from center to the vertex
                    double dx = points[i][0] - sx;
                    double dy = points[i][1] - sy;
                    double dz = points[i][2] - sz;

		    // length of vector
		    double norm = sqrt(dx * dx + dy * dy + dz * dz);
		    if (norm > 1e-6)
		    {
			double fact = scale + padding/norm;
			points[i] = fcl::Vec3f(sx + dx * fact, sy + dy * fact, sz + dz * fact);
		    }
		    else
		    {
			double ndx = ((dx > 0) ? dx+padding : dx-padding);
			double ndy = ((dy > 0) ? dy+padding : dy-padding);
			double ndz = ((dz > 0) ? dz+padding : dz-padding);        
			points[i] = fcl::Vec3f(sx + ndx, sy + ndy, sz + ndz);
		    }
		}

                g->beginModel();
                g->addSubModel(points, tri_indices);
                g->endModel();
                g->computeLocalAABB();
            }
        }
        break;
    default:
        ROS_FATAL_STREAM("This shape type is not supported using FCL yet");
    }

    return g;
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                 const CollisionRobot &other_robot, const planning_models::KinematicState &other_state) const
{
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                 const CollisionRobot &other_robot, const planning_models::KinematicState &other_state,
                                                                 const AllowedCollisionMatrix &acm) const
{
}
