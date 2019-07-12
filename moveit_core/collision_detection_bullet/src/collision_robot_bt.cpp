/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Jens Petit */

#include <moveit/collision_detection_bullet/collision_robot_bt.h>
#include <moveit/collision_detection_bullet/tesseract/ros_tesseract_utils.h>
#include <urdf/model.h>

namespace collision_detection
{
CollisionRobotBt::CollisionRobotBt(const robot_model::RobotModelConstPtr& model, double padding, double scale)
  : CollisionRobot(model, padding, scale), bt_manager_(new collision_detection_bullet::BulletDiscreteBVHManager)
{
  auto fun = std::bind(&collision_detection_bullet::allowedCollisionCheck, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3);

  bt_manager_->setIsContactAllowedFn(fun);

  for (const std::pair<std::string, urdf::LinkSharedPtr>& link : robot_model_->getURDF()->links_)
  {
    addLinkAsCOW(link.second);
  }
}

CollisionRobotBt::CollisionRobotBt(const CollisionRobotBt& other)
  : CollisionRobot(other), bt_manager_(other.bt_manager_->clone())
{
}

void CollisionRobotBt::addAttachedOjects(const robot_state::RobotState& state,
                                         std::vector<collision_detection_bullet::COWPtr>& cows) const
{
  std::vector<const robot_state::AttachedBody*> attached_bodies;
  state.getAttachedBodies(attached_bodies);
  for (const robot_state::AttachedBody*& body : attached_bodies)
  {
    const EigenSTL::vector_Isometry3d& attached_body_transform = body->getGlobalCollisionBodyTransforms();
    std::vector<collision_detection_bullet::CollisionObjectType> collision_object_types(
        attached_body_transform.size(), collision_detection_bullet::CollisionObjectType::UseShapeType);

    cows.emplace_back(collision_detection_bullet::createCollisionObject(
        body->getName(), collision_detection::BodyType::ROBOT_ATTACHED, body->getShapes(), attached_body_transform,
        collision_object_types, body->getTouchLinks(), true));
  }
}

void CollisionRobotBt::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                          const robot_state::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionRobotBt::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                          const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionRobotBt::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                          const robot_state::RobotState& state1,
                                          const robot_state::RobotState& state2) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet self continuous collision checking not yet implemented");
}

void CollisionRobotBt::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                          const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                          const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet self continuous collision checking not yet implemented");
}

void CollisionRobotBt::checkSelfCollisionCCDHelper(const CollisionRequest& req, CollisionResult& res,
                                                   const robot_state::RobotState& state1,
                                                   const robot_state::RobotState& state2,
                                                   const AllowedCollisionMatrix* acm) const
{
  // TODO: Not in tesseract yet
}

void CollisionRobotBt::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                const robot_state::RobotState& state,
                                                const AllowedCollisionMatrix* acm) const
{
  std::vector<collision_detection_bullet::COWPtr> cows;
  addAttachedOjects(state, cows);
  collision_detection_bullet::BulletDiscreteBVHManagerPtr discrete_clone_manager = bt_manager_->clone();
  updateTransformsFromState(state, discrete_clone_manager);
  discrete_clone_manager->contactTest(res, req, acm, cows);
}

void CollisionRobotBt::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                           const robot_state::RobotState& other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, nullptr);
}

void CollisionRobotBt::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                           const robot_state::RobotState& other_state,
                                           const AllowedCollisionMatrix& acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void CollisionRobotBt::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                           const CollisionRobot& other_robot,
                                           const robot_state::RobotState& other_state1,
                                           const robot_state::RobotState& other_state2) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet other robot continuous collision checking not yet implemented");
}

void CollisionRobotBt::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                           const CollisionRobot& other_robot,
                                           const robot_state::RobotState& other_state1,
                                           const robot_state::RobotState& other_state2,
                                           const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet other robot continuous collision checking not yet implemented");
}

void CollisionRobotBt::checkOtherCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                 const robot_state::RobotState& state,
                                                 const CollisionRobot& other_robot,
                                                 const robot_state::RobotState& other_state,
                                                 const AllowedCollisionMatrix* acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Other collision not implemented yet.");
}

void CollisionRobotBt::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  for (const std::string& link : links)
  {
    if (robot_model_->getURDF()->links_.find(link) != robot_model_->getURDF()->links_.end())
    {
      addLinkAsCOW(robot_model_->getURDF()->links_[link]);
    }
    else
    {
      ROS_ERROR_NAMED("collision_detection.bullet", "Updating padding or scaling for unknown link: '%s'", link.c_str());
    }
  }
}

void CollisionRobotBt::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                    const robot_state::RobotState& state) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Collision distance to self not implemented yet.");
}

void CollisionRobotBt::distanceOther(const DistanceRequest& req, DistanceResult& res,
                                     const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                     const robot_state::RobotState& other_state) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Collision distance to other not implemented yet.");
}

void CollisionRobotBt::updateTransformsFromState(const robot_state::RobotState& state,
                                                 collision_detection_bullet::BulletDiscreteBVHManagerPtr manager) const
{
  // updating link positions with the current robot state
  for (const std::pair<std::string, collision_detection_bullet::COWPtr>& link : manager->getCollisionObjects())
  {
    // select the first of the transformations for each link (composed of multiple shapes...)
    manager->setCollisionObjectsTransform(link.first, state.getCollisionBodyTransform(link.first, 0));
  }
}

void CollisionRobotBt::addLinkAsCOW(const urdf::LinkSharedPtr link)
{
  if (link->collision_array.size() > 0)
  {
    const std::vector<urdf::CollisionSharedPtr>& col_array =
        link->collision_array.empty() ? std::vector<urdf::CollisionSharedPtr>(1, link->collision) :
                                        link->collision_array;

    std::vector<shapes::ShapeConstPtr> shapes;
    collision_detection_bullet::AlignedVector<Eigen::Isometry3d> shape_poses;
    std::vector<collision_detection_bullet::CollisionObjectType> collision_object_types;

    for (std::size_t i = 0; i < col_array.size(); ++i)
    {
      if (col_array[i] && col_array[i]->geometry)
      {
        shapes::ShapePtr s = collision_detection_bullet::constructShape(col_array[i]->geometry.get());

        if (s)
        {
          if (fabs(getLinkScale(link->name) - 1.0) >= std::numeric_limits<double>::epsilon() ||
              fabs(getLinkPadding(link->name)) >= std::numeric_limits<double>::epsilon())
          {
            s->scaleAndPadd(getLinkScale(link->name), getLinkPadding(link->name));
          }

          shapes.push_back(s);
          shape_poses.push_back(collision_detection_bullet::urdfPose2Eigen(col_array[i]->origin));

          if (s->type == shapes::MESH)
          {
            collision_object_types.push_back(collision_detection_bullet::CollisionObjectType::ConvexHull);
          }
          else
          {
            collision_object_types.push_back(collision_detection_bullet::CollisionObjectType::UseShapeType);
          }
        }
      }
    }

    collision_detection_bullet::COWPtr cow = collision_detection_bullet::createCollisionObject(
        link->name, collision_detection::BodyType::ROBOT_LINK, shapes, shape_poses, collision_object_types, true);

    if (bt_manager_->hasCollisionObject(link->name))
    {
      bt_manager_->removeCollisionObject(link->name);
    }
    bt_manager_->addCollisionObject(cow);
  }
}

}  // end of namespace collision_detection
