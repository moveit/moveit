#include <moveit/collision_fcl_detailed/collision_robot_fcl_detailed.h>
#include <ros/ros.h>
#include <ctime>

namespace collision_detection
{

  void CollisionRobotFCLDetailed::distanceSelf(const DistanceRequest &req, DistanceResult &res, const robot_state::RobotState &state) const
  {
    CollisionRobotFCLDetailed::distanceSelfDetailedHelper(req, res, state);
  }

  void CollisionRobotFCLDetailed::distanceSelfDetailedHelper(const DistanceRequest &req, DistanceResult &res, const robot_state::RobotState &state) const
  {
    FCLManager manager;
    allocSelfCollisionBroadPhase(state, manager);
    DistanceData drd(&req, &res);

    manager.manager_->distance(&drd, &distanceDetailedCallback);
  }

  void CollisionRobotFCLDetailed::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
  {
    CollisionRobotFCLDetailed::checkSelfDetailedCollisionHelper(req, res, state, &acm);
  }

  void CollisionRobotFCLDetailed::checkSelfDetailedCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm) const
  {
    FCLManager manager;
    allocSelfCollisionBroadPhase(state, manager);
    CollisionData cd(&req, &res, acm);
    cd.enableGroup(getRobotModel());
    manager.manager_->collide(&cd, &collisionCallback);
    if (req.distance)
    {
      DistanceRequest dreq(false, true, req.group_name, acm, distance_threshold_);
      DistanceResult dres;

      dreq.enableGroup(getRobotModel());
      distanceSelfDetailedHelper(dreq, dres, state);
      res.distance = dres.minimum_distance.min_distance;
    }
  }

}//namespace constrained_ik

