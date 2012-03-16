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

#include "collision_detection/fcl/collision_robot.h"
#include <ros/console.h>

collision_detection::CollisionRobotFCL::CollisionRobotFCL(const planning_models::KinematicModelConstPtr &kmodel, double padding, double scale) : CollisionRobot(kmodel, padding, scale)
{
  links_ = kmodel_->getLinkModels();
  
  // we keep the same order of objects as what KinematicState::getLinkState() returns
  for (std::size_t i = 0 ; i < links_.size() ; ++i)
    if (links_[i] && links_[i]->getShape())
    {
      boost::shared_ptr<fcl::CollisionGeometry> cg =
	createCollisionGeometryOBB(links_[i]->getShape().get(), getLinkScale(links_[i]->getName()), getLinkPadding(links_[i]->getName()));
      CollisionGeometryData *cgd = NULL;
      if (cg)
      {
	cgd = new CollisionGeometryData(links_[i]);
	collision_geometry_data_[links_[i]->getName()].reset(cgd);
	index_map_[links_[i]->getName()] = geoms_obb_.size();
	cg->setUserData(cgd);
      }
      else
	links_[i] = NULL;
      geoms_obb_.push_back(cg);

      if (cg)
      {
        cg = createCollisionGeometryRSS(links_[i]->getShape().get(), getLinkScale(links_[i]->getName()), getLinkPadding(links_[i]->getName()));
	cg->setUserData(cgd);
      }
      geoms_rss_.push_back(cg);
    }
    else
    {
      links_[i] = NULL;
      geoms_obb_.push_back(boost::shared_ptr<fcl::CollisionGeometry>());
      geoms_rss_.push_back(boost::shared_ptr<fcl::CollisionGeometry>());
    }
}

collision_detection::CollisionRobotFCL::CollisionRobotFCL(const CollisionRobotFCL &other) : CollisionRobot(other)
{
  links_ = other.links_;
  geoms_obb_ = other.geoms_obb_;
  geoms_rss_ = other.geoms_rss_;
  collision_geometry_data_ = other.collision_geometry_data_;
  index_map_ = other.index_map_;
}

namespace collision_detection
{
namespace
{
typedef std::map<boost::shared_ptr<planning_models::KinematicState::AttachedBodyProperties>,
                 std::vector<boost::shared_ptr<fcl::CollisionGeometry> > > AttachedBodyObject;

static inline const std::vector<boost::shared_ptr<fcl::CollisionGeometry> >&
getAttachedBodyObjectsHelper(boost::mutex &lock, const planning_models::KinematicState::AttachedBody *ab,
                             std::map<std::string, boost::shared_ptr<CollisionGeometryData> > &cgd_map,
                             AttachedBodyObject &o1, AttachedBodyObject &o2, bool obb)
{ 
  // need an actual instance of the shared ptr for thread safety
  boost::shared_ptr<planning_models::KinematicState::AttachedBodyProperties> props = ab->getProperties();
  boost::mutex::scoped_lock slock(lock);

  AttachedBodyObject::const_iterator it = o1.find(props);
  if (it != o1.end())
    return it->second;
  
  CollisionGeometryData *cgd = NULL;
  
  it = o2.find(props);
  if (it != o2.end())
  {
    const std::vector<boost::shared_ptr<fcl::CollisionGeometry> > &arr = it->second;
    if (arr.empty())
    {   
      cgd = new CollisionGeometryData(props.get());
      // this is safe because collision_geometry_data_ is not modified elsewhere and we are already locked in this function
      cgd_map[props->id_].reset(cgd);
    }
    else
      cgd = (CollisionGeometryData*)arr.front()->getUserData();
  }
  else
  {
    cgd = new CollisionGeometryData(props.get());
    // this is safe because collision_geometry_data_ is not modified elsewhere and we are already locked in this function
    cgd_map[props->id_].reset(cgd);
  }
  
  const std::vector<shapes::Shape*> &shapes = ab->getShapes();
  std::vector<boost::shared_ptr<fcl::CollisionGeometry> > obj;
  for (std::size_t i = 0 ; i < shapes.size() ; ++i)
  {
    boost::shared_ptr<fcl::CollisionGeometry> co = obb ? createCollisionGeometryOBB(shapes[i]) : createCollisionGeometryRSS(shapes[i]);
    if (co)
      co->setUserData(cgd);
    obj.push_back(co);
  }
  return o1.insert(std::make_pair(props, obj)).first->second;
}

}
}

const std::vector<boost::shared_ptr<fcl::CollisionGeometry> >&
collision_detection::CollisionRobotFCL::getAttachedBodyObjectsOBB(const planning_models::KinematicState::AttachedBody *ab) const
{
  return getAttachedBodyObjectsHelper(attached_bodies_lock_, ab, const_cast<CollisionRobotFCL*>(this)->collision_geometry_data_, attached_bodies_obb_, attached_bodies_rss_, true);
}

const std::vector<boost::shared_ptr<fcl::CollisionGeometry> >&
collision_detection::CollisionRobotFCL::getAttachedBodyObjectsRSS(const planning_models::KinematicState::AttachedBody *ab) const
{
  return getAttachedBodyObjectsHelper(attached_bodies_lock_, ab, const_cast<CollisionRobotFCL*>(this)->collision_geometry_data_, attached_bodies_rss_, attached_bodies_obb_, false);
}

void collision_detection::CollisionRobotFCL::constructFCLObject(const planning_models::KinematicState &state, FCLObject &fcl_obj, bool obb) const
{
  const std::vector<boost::shared_ptr<fcl::CollisionGeometry> > &geoms = obb ? geoms_obb_ : geoms_rss_;
  const std::vector<planning_models::KinematicState::LinkState*> &link_states = state.getLinkStateVector();
  for (std::size_t i = 0 ; i < geoms.size() ; ++i)
  {
    if (geoms[i])
    {
      fcl::CollisionObject *collObj = new fcl::CollisionObject(geoms[i], transform2fcl(link_states[i]->getGlobalCollisionBodyTransform()));
      fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
    }
    std::vector<const planning_models::KinematicState::AttachedBody*> ab;
    link_states[i]->getAttachedBodies(ab);
    for (std::size_t j = 0 ; j < ab.size() ; ++j)
    {
      const std::vector<boost::shared_ptr<fcl::CollisionGeometry> > &objs = obb ? getAttachedBodyObjectsOBB(ab[j]) : getAttachedBodyObjectsRSS(ab[j]);
      const std::vector<Eigen::Affine3d> &ab_t = ab[j]->getGlobalCollisionBodyTransforms();
      for (std::size_t k = 0 ; k < objs.size() ; ++k)
        if (objs[k])
        {
          fcl::CollisionObject *collObj = new fcl::CollisionObject(objs[k], transform2fcl(ab_t[k]));
          fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
        }
    }
  }
}

void collision_detection::CollisionRobotFCL::allocSelfCollisionBroadPhase(const planning_models::KinematicState &state, FCLManager &manager) const
{
  manager.manager_.reset(new fcl::SSaPCollisionManager());
  constructFCLObject(state, manager.object_, true);
  manager.object_.registerTo(manager.manager_.get());
  manager.manager_->update();
}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state) const
{
  checkSelfCollisionHelper(req, res, state, NULL);
}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                const AllowedCollisionMatrix &acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void collision_detection::CollisionRobotFCL::checkSelfCollisionHelper(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                      const AllowedCollisionMatrix *acm) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);
  CollisionData cd(&req, &res, acm);
  manager.manager_->collide(&cd, &collisionCallback);
  if (req.distance)
    res.distance = distanceSelfHelper(state, acm);
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                 const CollisionRobot &other_robot, const planning_models::KinematicState &other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, NULL);
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                 const CollisionRobot &other_robot, const planning_models::KinematicState &other_state,
                                                                 const AllowedCollisionMatrix &acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void collision_detection::CollisionRobotFCL::checkOtherCollisionHelper(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                                                       const CollisionRobot &other_robot, const planning_models::KinematicState &other_state,
                                                                       const AllowedCollisionMatrix *acm) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);
  
  const CollisionRobotFCL &fcl_rob = dynamic_cast<const CollisionRobotFCL&>(other_robot);
  FCLObject other_fcl_obj;
  fcl_rob.constructFCLObject(other_state, other_fcl_obj, true);
  
  CollisionData cd(&req, &res, acm);
  for (std::size_t i = 0 ; !cd.done_ && i < other_fcl_obj.collision_objects_.size() ; ++i)
    manager.manager_->collide(other_fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);
  if (req.distance)
    res.distance = distanceOtherHelper(state, other_robot, other_state, acm);
}

void collision_detection::CollisionRobotFCL::updatedPaddingOrScaling(const std::vector<std::string> &links)
{
  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    std::map<std::string, std::size_t>::const_iterator it = index_map_.find(links[i]);
    const planning_models::KinematicModel::LinkModel *lmodel = kmodel_->getLinkModel(links[i]);
    if (it != index_map_.end() && lmodel)
    {
      boost::shared_ptr<fcl::CollisionGeometry> cg = createCollisionGeometryOBB(lmodel->getShape().get(), getLinkScale(links[i]), getLinkPadding(links[i]));
      cg->setUserData(geoms_obb_[it->second]->getUserData());
      geoms_obb_[it->second] = cg;

      cg = createCollisionGeometryRSS(lmodel->getShape().get(), getLinkScale(links[i]), getLinkPadding(links[i]));
      cg->setUserData(geoms_rss_[it->second]->getUserData());
      geoms_rss_[it->second] = cg;
    }
    else
      ROS_ERROR("Updating padding or scaling for unknown link: '%s'", links[i].c_str());
  }
}

double collision_detection::CollisionRobotFCL::distanceSelf(const planning_models::KinematicState &state) const
{
  return distanceSelfHelper(state, NULL);
}

double collision_detection::CollisionRobotFCL::distanceSelf(const planning_models::KinematicState &state,
                                                            const AllowedCollisionMatrix &acm) const
{
  return distanceSelfHelper(state, &acm);
}

double collision_detection::CollisionRobotFCL::distanceSelfHelper(const planning_models::KinematicState &state,
                                                                  const AllowedCollisionMatrix *acm) const
{
  return 0.0;
}

double collision_detection::CollisionRobotFCL::distanceOther(const planning_models::KinematicState &state,
                                                             const CollisionRobot &other_robot,
                                                             const planning_models::KinematicState &other_state) const
{
  return distanceOtherHelper(state, other_robot, other_state, NULL);
}

double collision_detection::CollisionRobotFCL::distanceOther(const planning_models::KinematicState &state,
                                                             const CollisionRobot &other_robot,
                                                             const planning_models::KinematicState &other_state,
                                                             const AllowedCollisionMatrix &acm) const
{
  return distanceOtherHelper(state, other_robot, other_state, &acm);
}

double collision_detection::CollisionRobotFCL::distanceOtherHelper(const planning_models::KinematicState &state,
                                                                   const CollisionRobot &other_robot,
                                                                   const planning_models::KinematicState &other_state,
                                                                   const AllowedCollisionMatrix *acm) const
{
  return 0.0;
}
