#ifndef COLLISION_COMMON_DETAILED_H
#define COLLISION_COMMON_DETAILED_H

#include <fcl/distance.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <eigen3/Eigen/Core>

namespace collision_detection
{
  typedef std::map<std::string, fcl::DistanceResult> DistanceMap;

  struct DistanceRequest
  {
    DistanceRequest(): detailed(false),
                       global(true),
                       group_name(NULL),
                       active_components_only(NULL),
                       acm(NULL),
                       verbose(false) {}

    DistanceRequest(bool detailed,
                    bool global,
                    const std::set<const robot_model::LinkModel*> *active_components_only,
                    const collision_detection::AllowedCollisionMatrix *acm): detailed(detailed),
                                                                             global(global),
                                                                             group_name(NULL),
                                                                             active_components_only(active_components_only),
                                                                             acm(acm),
                                                                             verbose(false) {}
    DistanceRequest(bool detailed,
                    bool global,
                    const std::set<const robot_model::LinkModel*> &active_components_only,
                    const collision_detection::AllowedCollisionMatrix &acm): detailed(detailed),
                                                                             global(global),
                                                                             group_name(NULL),
                                                                             active_components_only(&active_components_only),
                                                                             acm(&acm),
                                                                             verbose(false) {}
    DistanceRequest(bool detailed,
                    bool global,
                    const std::string group_name,
                    const collision_detection::AllowedCollisionMatrix *acm): detailed(detailed),
                                                                             global(global),
                                                                             group_name(group_name),
                                                                             active_components_only(NULL),
                                                                             acm(acm),
                                                                             verbose(false) {}
    DistanceRequest(bool detailed,
                    bool global,
                    const std::string group_name,
                    const collision_detection::AllowedCollisionMatrix &acm): detailed(detailed),
                                                                             global(global),
                                                                             group_name(group_name),
                                                                             active_components_only(NULL),
                                                                             acm(&acm),
                                                                             verbose(false) {}

    virtual ~DistanceRequest() {}

    /// Compute \e active_components_only_ based on \e req_
    void enableGroup(const robot_model::RobotModelConstPtr &kmodel);

    bool detailed;

    bool global;

    std::string group_name;

    const std::set<const robot_model::LinkModel*> *active_components_only;

    const collision_detection::AllowedCollisionMatrix *acm;

    bool verbose;



  };

  struct DistanceResult
  {
    DistanceResult(): collision(false) {}
    virtual ~DistanceResult() {}

    bool collision;

    fcl::DistanceResult minimum_distance;

    DistanceMap distance;
  };

  struct DistanceData
  {
    DistanceData(const DistanceRequest *req, DistanceResult *res): req(req), res(res), done(false) {}
    virtual ~DistanceData() {}

    const DistanceRequest *req;

    DistanceResult *res;

    bool done;

  };

  bool distanceDetailedCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void *data, double& min_dist);

  /** @brief Containst distance information in the planning frame queried from getDistanceInfo() */
  struct DistanceInfo
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string nearest_obsticle; /**< The link name for nearest obsticle/link to request link. */
    Eigen::Vector3d link_point; /**< Point on request link */
    Eigen::Vector3d obsticle_point; /**< Point on nearest link to requested link */
    Eigen::Vector3d avoidance_vector; /**< Normilized Vector created by nearest points */
    double distance; /**< Distance between nearest points */
  };
  typedef std::map<std::string, DistanceInfo> DistanceInfoMap;

  /**
   * @brief getDistanceInfo
   * @param distance_detailed Detailed Distance Map
   * @param distance_info_map Stores the distance information for each link in DistanceDetailedMap
   * @param tf This allows for a transformation to be applied the distance data since it is always returned in the world frame from fcl.
   * @return bool, true if succesfully converted DistanceDetailedMap to DistanceInfoMap
   */
  bool getDistanceInfo(const DistanceMap &distance_detailed, DistanceInfoMap &distance_info_map, const Eigen::Affine3d &tf);

  /**
   * @brief getDistanceInfo
   * @param distance_detailed Detailed Distance Map
   * @param distance_info_map Stores the distance information for each link in DistanceDetailedMap
   * @return bool, true if succesfully converted DistanceDetailedMap to DistanceInfoMap
   */
  bool getDistanceInfo(const DistanceMap &distance_detailed, DistanceInfoMap &distance_info_map);

}
#endif // COLLISION_COMMON_DETAILED_H

