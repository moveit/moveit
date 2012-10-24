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

#include <moveit/collision_detection_fcl/collision_world.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>

collision_detection::CollisionWorldFCL::CollisionWorldFCL(void) : CollisionWorld()
{
  fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);
}

collision_detection::CollisionWorldFCL::CollisionWorldFCL(const CollisionWorldFCL &other) : CollisionWorld(other)
{
  fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);

  fcl_objs_ = other.fcl_objs_;
  for (std::map<std::string, FCLObject>::iterator it = fcl_objs_.begin() ; it != fcl_objs_.end() ; ++it)
    it->second.registerTo(manager_.get());
  // manager_->update();
}

collision_detection::CollisionWorldFCL::~CollisionWorldFCL(void)
{
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state) const
{
  checkRobotCollisionHelper(req, res, robot, state, NULL);
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state, const AllowedCollisionMatrix &acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state1, const kinematic_state::KinematicState &state2) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state1, const kinematic_state::KinematicState &state2, const AllowedCollisionMatrix &acm) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionWorldFCL::checkRobotCollisionHelper(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state, const AllowedCollisionMatrix *acm) const
{
  const CollisionRobotFCL &robot_fcl = dynamic_cast<const CollisionRobotFCL&>(robot);
  FCLObject fcl_obj;
  robot_fcl.constructFCLObject(state, fcl_obj);
  
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getKinematicModel());
  for (std::size_t i = 0 ; !cd.done_ && i < fcl_obj.collision_objects_.size() ; ++i)
    manager_->collide(fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);
  
  if (req.distance)
    res.distance = distanceRobotHelper(robot, state, acm);

  // Refine/smooth the contact normals for contacts with octomap.
  if(req.contacts && res.contact_count)
  {
    refineContactNormals(res);
  }
}

void collision_detection::CollisionWorldFCL::refineContactNormals(CollisionResult &res) const
{
  // iterate through contacts
  for( collision_detection::CollisionResult::ContactMap::iterator it = res.contacts.begin(); it != res.contacts.end(); ++it)
  {
    std::string contact1 = it->first.first;
    std::string contact2 = it->first.second;
    std::string octomap_name = "";
    std::vector<collision_detection::Contact>& contact_vector = it->second;

    if(      contact1.find("octomap") != std::string::npos ) octomap_name = contact1;
    else if( contact2.find("octomap") != std::string::npos ) octomap_name = contact2;
    else
    {
      continue;
    }

    ObjectConstPtr object = getObject(octomap_name);
    if(!object) return;

    float cell_size = 0;
    if(!object->shapes_.empty())
    {
      shapes::ShapeConstPtr shape = object->shapes_[0];
      boost::shared_ptr<const shapes::OcTree> octree = boost::dynamic_pointer_cast<const shapes::OcTree>(shape);
      if(octree)
      {
        cell_size = octree->octree->getResolution();
        for(size_t contact_index = 0; contact_index < contact_vector.size(); contact_index++)
        {
          Eigen::Vector3d point =   contact_vector[contact_index].pos;
          //Eigen::Vector3d normal =  contact_vector[contact_index].normal;

          octomath::Vector3 contact_point(point[0], point[1], point[2]);
          octomath::Vector3 diagonal = octomath::Vector3(1,1,1);
          octomath::Vector3 bbx_min = contact_point - diagonal*cell_size; // TODO should this be a bit larger? (or smaller?)
          octomath::Vector3 bbx_max = contact_point + diagonal*cell_size;
          octomap::point3d_list node_centers;
          octree->octree->getOccupiedLeafsBBX(node_centers, bbx_min, bbx_max);

          octomath::Vector3 n;
          if(getCloudNormal(node_centers, cell_size, contact_point, n))
          {
            contact_vector[contact_index].normal = Eigen::Vector3d(n.x(), n.y(), n.z());
          }
        }
      }
    }
  }
}

bool collision_detection::CollisionWorldFCL::getCloudNormal(const octomap::point3d_list &cloud, const float &spacing, const octomath::Vector3 &position, octomath::Vector3 &normal) const
{
  float intensity;
  octomath::Vector3 gradient;
  if(sampleCloud(cloud, spacing, position, intensity, gradient))
  {
    gradient.normalize();
    normal = -gradient; // by convention the normal should point "out"
    return true;
  }
  else
  {
    return false;
  }
}


bool collision_detection::CollisionWorldFCL::sampleCloud(const octomap::point3d_list &cloud, const float &spacing, const octomath::Vector3 &position, float &intensity, octomath::Vector3 &gradient) const
{
  intensity = 0.f;
  gradient = octomath::Vector3(0,0,0);

  float R = 1.5*spacing; // TODO magic number!
  //float T = 0.5; // TODO magic number!

  int NN = cloud.size();
  if(NN == 0)
  {
    return false;
  }

  // variables for Wyvill
  double a=0, b=0, c=0, R2=0, R4=0, R6=0, a1=0, b1=0, c1=0, a2=0, b2=0, c2=0;
  bool WYVILL = true;

  octomap::point3d_list::const_iterator it;
  for ( it = cloud.begin(); it != cloud.end(); it++ )
  {
    octomath::Vector3 v = (*it);

    if(WYVILL)
    {
      R2 = R*R;
      R4 = R2*R2;
      R6 = R4*R2;
      a = -4.0/9.0; b  = 17.0/9.0; c = -22.0/9.0;
      a1 = a/R6; b1 = b/R4; c1 = c/R2;
      a2 = 6*a1; b2 = 4*b1; c2 = 2*c1;
    }
    else
    {
      ROS_ERROR("This should not be called!");
    }

    double f_val = 0;
    octomath::Vector3 f_grad(0,0,0);

    octomath::Vector3 pos = position-v;
    double r = pos.norm();
    pos = pos*(1.0/r);
    if(r > R)  // must skip points outside valid bounds.
    {
      continue;
    }
    double r2 = r*r;
    double r3 = r*r2;
    double r4 = r2*r2;
    double r5 = r3*r2;
    double r6 = r3*r3;

    if(WYVILL)
    {
      f_val = (a1*r6 + b1*r4 + c1*r2 + 1);
      f_grad = pos*(a2*r5 + b2*r3 + c2*r);
    }
    else
    {
      ROS_ERROR("This should not be called!");
      double r_scaled = r/R;
      // TODO still need to address the scaling...
      f_val = pow((1-r_scaled),4)*(4*r_scaled + 1);
      f_grad = pos*(-4.0/R*pow(1.0-r_scaled,3)*(4.0*r_scaled+1.0)+4.0/R*pow(1-r_scaled,4));
    }

    // TODO:  The whole library should be overhauled to follow the "gradient points out"
    //        convention of implicit functions.
    intensity += f_val;
    gradient += f_grad;
  }
  return true; // it worked
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
  const CollisionWorldFCL &other_fcl_world = dynamic_cast<const CollisionWorldFCL&>(other_world);
  CollisionData cd(&req, &res, acm);
  manager_->collide(other_fcl_world.manager_.get(), &cd, &collisionCallback);
  
  if (req.distance)
    res.distance = distanceWorldHelper(other_world, acm);
}

void collision_detection::CollisionWorldFCL::constructFCLObject(const Object *obj, FCLObject &fcl_obj) const
{
  for (std::size_t i = 0 ; i < obj->shapes_.size() ; ++i)
  {
    FCLGeometryConstPtr g = createCollisionGeometry(obj->shapes_[i], obj);
    if (g)
    {
      fcl::CollisionObject *co = new fcl::CollisionObject(g->collision_geometry_,  transform2fcl(obj->shape_poses_[i]));
      fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(co));
      fcl_obj.collision_geometry_.push_back(g);
    }
  }
}

void collision_detection::CollisionWorldFCL::updateFCLObject(const std::string &id)
{
  // remove FCL objects that correspond to this object
  std::map<std::string, FCLObject>::iterator jt = fcl_objs_.find(id);
  if (jt != fcl_objs_.end())
  {
    jt->second.unregisterFrom(manager_.get());
    jt->second.clear();
  }

  // check to see if we have this object
  std::map<std::string, ObjectPtr>::iterator it = objects_.find(id);
  if (it != objects_.end())
  {
    // construct FCL objects that correspond to this object
    if (jt != fcl_objs_.end())
    {
      constructFCLObject(it->second.get(), jt->second);
      jt->second.registerTo(manager_.get());
    }
    else
    {
      constructFCLObject(it->second.get(), fcl_objs_[id]);
      fcl_objs_[id].registerTo(manager_.get());
    }
  }
  else
  {
    if (jt != fcl_objs_.end())
      fcl_objs_.erase(jt);
  }
  
  // manager_->update();
}

void collision_detection::CollisionWorldFCL::addToObject(const std::string &id, const std::vector<shapes::ShapeConstPtr> &shapes, const EigenSTL::vector_Affine3d &poses)
{
  CollisionWorld::addToObject(id, shapes, poses);
  updateFCLObject(id);
}

void collision_detection::CollisionWorldFCL::addToObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  CollisionWorld::addToObject(id, shape, pose);
  updateFCLObject(id);
}

bool collision_detection::CollisionWorldFCL::moveShapeInObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  if (CollisionWorld::moveShapeInObject(id, shape, pose))
  {
    updateFCLObject(id);
    return true;
  }
  else
    return false;
}

bool collision_detection::CollisionWorldFCL::removeShapeFromObject(const std::string &id, const shapes::ShapeConstPtr &shape)
{    
  if (CollisionWorld::removeShapeFromObject(id, shape))
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
    // manager_->update();
  }
}

void collision_detection::CollisionWorldFCL::clearObjects(void)
{
  CollisionWorld::clearObjects();
  manager_->clear();
  fcl_objs_.clear();
}

double collision_detection::CollisionWorldFCL::distanceRobotHelper(const CollisionRobot &robot, const kinematic_state::KinematicState &state, const AllowedCollisionMatrix *acm) const
{       
  const CollisionRobotFCL& robot_fcl = dynamic_cast<const CollisionRobotFCL&>(robot);
  FCLObject fcl_obj;
  robot_fcl.constructFCLObject(state, fcl_obj);

  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getKinematicModel());
  
  for(std::size_t i = 0; !cd.done_ && i < fcl_obj.collision_objects_.size(); ++i)
    manager_->distance(fcl_obj.collision_objects_[i].get(), &cd, &distanceCallback);


  return res.distance;
}

double collision_detection::CollisionWorldFCL::distanceRobot(const CollisionRobot &robot, const kinematic_state::KinematicState &state) const
{
  return distanceRobotHelper(robot, state, NULL);
}

double collision_detection::CollisionWorldFCL::distanceRobot(const CollisionRobot &robot, const kinematic_state::KinematicState &state, const AllowedCollisionMatrix &acm) const
{
  return distanceRobotHelper(robot, state, &acm);
}

double collision_detection::CollisionWorldFCL::distanceWorld(const CollisionWorld &world) const
{
  return distanceWorldHelper(world, NULL);
}

double collision_detection::CollisionWorldFCL::distanceWorld(const CollisionWorld &world, const AllowedCollisionMatrix &acm) const
{
  return distanceWorldHelper(world, &acm);
}

double collision_detection::CollisionWorldFCL::distanceWorldHelper(const CollisionWorld &other_world, const AllowedCollisionMatrix *acm) const
{
  const CollisionWorldFCL& other_fcl_world = dynamic_cast<const CollisionWorldFCL&>(other_world);
  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  manager_->distance(other_fcl_world.manager_.get(), &cd, &distanceCallback);

  return res.distance;
}
