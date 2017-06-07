/**
 * @file collision_common.h
 * @brief This contains common data used during collision checking
 *
 * This add additional capability not found in the existing MoveIt
 * implementation. The add capability is the ability to make detailed
 * distance requests.
 *
 * @author Levi Armstrong
 * @date May 4, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef COLLISION_DETECTION_COLLISION_COMMON_H_
#define COLLISION_DETECTION_COLLISION_COMMON_H_

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_world.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <set>

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
                       distance_threshold(std::numeric_limits<double>::max()),
                       verbose(false) {}

    DistanceRequest(bool detailed,
                    bool global,
                    const std::set<const robot_model::LinkModel*> *active_components_only,
                    const collision_detection::AllowedCollisionMatrix *acm,
                    double distance_threshold = std::numeric_limits<double>::max()): detailed(detailed),
                                                                                     global(global),
                                                                                     active_components_only(active_components_only),
                                                                                     acm(acm),
                                                                                     distance_threshold(distance_threshold),
                                                                                     verbose(false) {}
    DistanceRequest(bool detailed,
                    bool global,
                    const std::set<const robot_model::LinkModel*> &active_components_only,
                    const collision_detection::AllowedCollisionMatrix &acm,
                    double distance_threshold = std::numeric_limits<double>::max()): detailed(detailed),
                                                                                     global(global),
                                                                                     active_components_only(&active_components_only),
                                                                                     acm(&acm),
                                                                                     distance_threshold(distance_threshold),
                                                                                     verbose(false) {}
    DistanceRequest(bool detailed,
                    bool global,
                    const std::string group_name,
                    const collision_detection::AllowedCollisionMatrix *acm,
                    double distance_threshold = std::numeric_limits<double>::max()): detailed(detailed),
                                                                                     global(global),
                                                                                     group_name(group_name),
                                                                                     active_components_only(NULL),
                                                                                     acm(acm),
                                                                                     distance_threshold(distance_threshold),
                                                                                     verbose(false) {}
    DistanceRequest(bool detailed,
                    bool global,
                    const std::string group_name,
                    const collision_detection::AllowedCollisionMatrix &acm,
                    double distance_threshold = std::numeric_limits<double>::max()): detailed(detailed),
                                                                                     global(global),
                                                                                     group_name(group_name),
                                                                                     active_components_only(NULL),
                                                                                     acm(&acm),
                                                                                     distance_threshold(distance_threshold),
                                                                                     verbose(false) {}

    virtual ~DistanceRequest() {}

    /// Compute \e active_components_only_ based on \e req_
    void enableGroup(const robot_model::RobotModelConstPtr &kmodel);

    bool detailed;

    bool global;

    std::string group_name;

    const std::set<const robot_model::LinkModel*> *active_components_only;

    const collision_detection::AllowedCollisionMatrix *acm;

    double distance_threshold;

    bool verbose;

  };

  struct DistanceResult
  {
    DistanceResult(): collision(false) {}
    virtual ~DistanceResult() {}

    bool collision;

    fcl::DistanceResult minimum_distance;

    DistanceMap distance;

    void clear()
    {
      collision = false;
      minimum_distance.clear();
      distance.clear();
    }
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

#endif
