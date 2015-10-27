#ifndef COLLISION_ROBOT_FCL_DETAILED_H
#define COLLISION_ROBOT_FCL_DETAILED_H

#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <eigen3/Eigen/Core>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_fcl_detailed/collision_common_detailed.h>

namespace collision_detection
{
  class CollisionRobotFCLDetailed : public collision_detection::CollisionRobotFCL
  {
    friend class CollisionWorldFCLDetailed;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef boost::shared_ptr<CollisionRobotFCLDetailed> CollisionRobotFCLDetailedPtr;

    CollisionRobotFCLDetailed(const robot_model::RobotModelConstPtr &kmodel, double padding = 0.0, double scale = 1.0): CollisionRobotFCL(kmodel, padding, scale) {}

    CollisionRobotFCLDetailed(const CollisionRobotFCLDetailed &other): CollisionRobotFCL(other) {}

    virtual ~CollisionRobotFCLDetailed() {}

    virtual void distanceSelf(const DistanceRequest &req, DistanceResult &res, const robot_state::RobotState &state) const;

    virtual void distanceSelfDetailedHelper(const DistanceRequest &req, DistanceResult &res, const robot_state::RobotState &state) const;

    virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix &acm) const;

    virtual void checkSelfDetailedCollisionHelper(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix *acm) const;

  };
} //namespace collision_detection
#endif // COLLISION_ROBOT_FCL_DETAILED_H

