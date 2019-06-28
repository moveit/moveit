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

namespace collision_detection
{
CollisionRobotBt::CollisionRobotBt(const robot_model::RobotModelConstPtr& model, double padding, double scale)
  : CollisionRobot(model, padding, scale)
{
  auto fun =
      std::bind(&tesseract::allowedCollisionCheck, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  bt_manager_.setIsContactAllowedFn(fun);

  // TODO: For the first link some memory error occurs
  for (const auto& link : robot_model_->getURDF()->links_)
  {
    addLinkAsCOW(link.second);
  }
}

CollisionRobotBt::CollisionRobotBt(const CollisionRobotBt& other) : CollisionRobot(other)
{
  const CollisionRobotBt& other_bt = dynamic_cast<const CollisionRobotBt&>(other);

  // TODO: use clone method of manager
  for (const auto& cow : other_bt.bt_manager_.getCollisionObjects())
  {
    tesseract::tesseract_bullet::COWPtr new_cow = cow.second->clone();
    new_cow->setWorldTransform(cow.second->getWorldTransform());
    new_cow->setContactProcessingThreshold(static_cast<btScalar>(other_bt.bt_manager_.getContactDistanceThreshold()));
    bt_manager_.addCollisionObject(new_cow);
  }

  bt_manager_.setActiveCollisionObjects(other_bt.bt_manager_.getActiveCollisionObjects());
  bt_manager_.setContactDistanceThreshold(other_bt.bt_manager_.getContactDistanceThreshold());
  bt_manager_.setIsContactAllowedFn(other_bt.bt_manager_.getIsContactAllowedFn());
}

void CollisionRobotBt::addAttachedOjects(const robot_state::RobotState& state,
                                         std::vector<tesseract::tesseract_bullet::COWPtr>& cows) const
{
  std::vector<const robot_state::AttachedBody*> ab;
  state.getAttachedBodies(ab);
  for (auto& body : ab)
  {
    const EigenSTL::vector_Isometry3d& attached_body_transform = body->getGlobalCollisionBodyTransforms();
    std::vector<tesseract::CollisionObjectType> collision_object_types(attached_body_transform.size(),
                                                                       tesseract::CollisionObjectType::UseShapeType);

    cows.emplace_back(tesseract::tesseract_bullet::createCollisionObject(
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
  // updateTransformsFromStateCCD(state1, state2);
  // bt_manager_CCD_.contactTest(res, req, acm);
}

void CollisionRobotBt::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                const robot_state::RobotState& state,
                                                const AllowedCollisionMatrix* acm) const
{
  updateTransformsFromState(state);
  std::vector<tesseract::tesseract_bullet::COWPtr> cows;
  addAttachedOjects(state, cows);
  bt_manager_.contactTest(res, req, acm, cows);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(getRobotModel());
    distanceSelf(dreq, dres, state);
    res.distance = dres.minimum_distance.distance;
  }
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
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet continuous collision checking not yet implemented");
}

void CollisionRobotBt::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                           const CollisionRobot& other_robot,
                                           const robot_state::RobotState& other_state1,
                                           const robot_state::RobotState& other_state2,
                                           const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Bullet continuous collision checking not yet implemented");
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
  // TODO: add for bullet manager -> iterate through each input link and then construct new object
  // out of the geometry from the robot
  for (const auto& link : links)
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

void CollisionRobotBt::updateTransformsFromState(const robot_state::RobotState& state) const
{
  // updating link positions with the current robot state
  for (auto& link : bt_manager_.getCollisionObjects())
  {
    // select the first of the transformations for each link (composed of multiple shapes...)
    // TODO: further investigate if this brings problems
    bt_manager_.setCollisionObjectsTransform(link.first, state.getCollisionBodyTransform(link.first, 0));
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
    tesseract::AlignedVector<Eigen::Isometry3d> shape_poses;
    std::vector<tesseract::CollisionObjectType> collision_object_types;

    for (std::size_t i = 0; i < col_array.size(); ++i)
    {
      if (col_array[i] && col_array[i]->geometry)
      {
        shapes::ShapePtr s = tesseract::tesseract_ros::constructShape(col_array[i]->geometry.get());

        if (s)
        {
          if (fabs(getLinkScale(link->name) - 1.0) >= std::numeric_limits<double>::epsilon() &&
              fabs(getLinkPadding(link->name)) >= std::numeric_limits<double>::epsilon())
          {
            s->scaleAndPadd(getLinkScale(link->name), getLinkPadding(link->name));
          }

          shapes.push_back(s);
          shape_poses.push_back(tesseract::tesseract_ros::urdfPose2Eigen(col_array[i]->origin));

          if (s->type == shapes::MESH)
          {
            collision_object_types.push_back(tesseract::CollisionObjectType::ConvexHull);
          }
          else
          {
            collision_object_types.push_back(tesseract::CollisionObjectType::UseShapeType);
          }
        }
      }
    }

    tesseract::tesseract_bullet::COWPtr cow = tesseract::tesseract_bullet::createCollisionObject(
        link->name, collision_detection::BodyType::ROBOT_LINK, shapes, shape_poses, collision_object_types, true);

    link2cow_[link->name] = cow;
    link2cow_CCD_[link->name] = cow->clone();

    if (bt_manager_.hasCollisionObject(link->name))
    {
      bt_manager_.removeCollisionObject(link->name);
    }
    bt_manager_.addCollisionObject(cow->clone());
  }
}

}  // end of namespace collision_detection
