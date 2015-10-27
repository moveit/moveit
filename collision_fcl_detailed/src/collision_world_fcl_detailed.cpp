#include <moveit/collision_fcl_detailed/collision_world_fcl_detailed.h>
#include <moveit/collision_fcl_detailed/collision_robot_fcl_detailed.h>

namespace collision_detection
{
  using namespace collision_detection;

  void CollisionWorldFCLDetailed::distanceRobot(const DistanceRequest &req, DistanceResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state) const
  {
    CollisionWorldFCLDetailed::distanceRobotDetailedHelper(req, res, robot, state);
  }

  void CollisionWorldFCLDetailed::distanceRobotDetailedHelper(const DistanceRequest &req, DistanceResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state) const
  {
    const CollisionRobotFCLDetailed& robot_fcl = dynamic_cast<const CollisionRobotFCLDetailed&>(robot);
    FCLObject fcl_obj;
    robot_fcl.constructFCLObject(state, fcl_obj);

    DistanceData drd(&req, &res);
    for(std::size_t i = 0; !drd.done && i < fcl_obj.collision_objects_.size(); ++i)
      manager_->distance(fcl_obj.collision_objects_[i].get(), &drd, &distanceDetailedCallback);

  }

  void CollisionWorldFCLDetailed::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
  {
    CollisionWorldFCLDetailed::checkRobotDetailedCollisionHelper(req, res, robot, state, &acm);
  }

  void CollisionWorldFCLDetailed::checkRobotDetailedCollisionHelper(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm) const
  {
    const CollisionRobotFCLDetailed &robot_fcl = dynamic_cast<const CollisionRobotFCLDetailed&>(robot);
    FCLObject fcl_obj;
    robot_fcl.constructFCLObject(state, fcl_obj);

    CollisionData cd(&req, &res, acm);
    cd.enableGroup(robot.getRobotModel());
    for (std::size_t i = 0 ; !cd.done_ && i < fcl_obj.collision_objects_.size() ; ++i)
      manager_->collide(fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);

    if (req.distance)
    {
      DistanceRequest dreq(false, true, req.group_name, acm);
      DistanceResult dres;

      dreq.enableGroup(robot.getRobotModel());
      distanceRobotDetailedHelper(dreq, dres, robot, state);
      res.distance = dres.minimum_distance.min_distance;
    }
  }

}
