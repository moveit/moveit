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

#ifndef MOVEIT_COLLISION_DETECTION_COLLISION_COMMON_
#define MOVEIT_COLLISION_DETECTION_COLLISION_COMMON_

#include <boost/array.hpp>
#include <boost/function.hpp>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <Eigen/Core>
#include <console_bridge/console.h>

namespace collision_detection
{
  /** \brief The types of bodies that are considered for collision */
  namespace BodyTypes
  {
    /** \brief The types of bodies that are considered for collision */
    enum Type
      {
        /** \brief A link on the robot */
        ROBOT_LINK,

        /** \brief A body attached to a robot link */
        ROBOT_ATTACHED,

        /** \brief A body in the environment */
        WORLD_OBJECT
      };
  }

  /** \brief The types of bodies that are considered for collision */
  typedef BodyTypes::Type BodyType;

  /** \brief Definition of a contact point */
  struct Contact
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** \brief contact position */
    Eigen::Vector3d pos;

    /** \brief normal unit vector at contact */
    Eigen::Vector3d normal;

    /** \brief depth (penetration between bodies) */
    double          depth;

    /** \brief The id of the first body involved in the contact */
    std::string     body_name_1;

    /** \brief The type of the first body involved in the contact */
    BodyType        body_type_1;

    /** \brief The id of the second body involved in the contact */
    std::string     body_name_2;

    /** \brief The type of the second body involved in the contact */
    BodyType        body_type_2;
  };

  /** \brief When collision costs are computed, this structure contains information about the partial cost incurred in a particular volume */
  struct CostSource
  {
    /// The minimum bound of the AABB defining the volume responsible for this partial cost
    boost::array<double, 3> aabb_min;

    /// The maximum bound of the AABB defining the volume responsible for this partial cost
    boost::array<double, 3> aabb_max;

    /// The partial cost (the probability of existance for the object there is a collision with)
    double                  cost;

    /// Get the volume of the AABB around the cost source
    double getVolume() const
    {
      return (aabb_max[0] - aabb_min[0]) * (aabb_max[1] - aabb_min[1]) * (aabb_max[2] - aabb_min[2]);
    }

    /// Order cost sources so that the most costly source is at the top
    bool operator<(const CostSource &other) const
    {
      double c1 = cost * getVolume();
      double c2 = other.cost * other.getVolume();
      if (c1 > c2)
        return true;
      if (c1 < c2)
        return false;
      if (cost < other.cost)
        return false;
      if (cost > other.cost)
        return true;
      return aabb_min < other.aabb_min;
    }
  };

  /** \brief Representation of a collision checking result */
  struct CollisionResult
  {
    CollisionResult() : collision(false),
                        distance(std::numeric_limits<double>::max()),
                        contact_count(0)
    {
    }
    typedef std::map<std::pair<std::string, std::string>, std::vector<Contact> > ContactMap;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** \brief Clear a previously stored result */
    void clear()
    {
      collision = false;
      distance = std::numeric_limits<double>::max();
      contact_count = 0;
      contacts.clear();
      cost_sources.clear();
    }

    /** \brief True if collision was found, false otherwise */
    bool                 collision;

    /** \brief Closest distance between two bodies */
    double               distance;

    /** \brief Number of contacts returned */
    std::size_t          contact_count;

    /** \brief A map returning the pairs of ids of the bodies in contact, plus information about the contacts themselves */
    ContactMap           contacts;

    /** \brief When costs are computed, the individual cost sources are  */
    std::set<CostSource> cost_sources;
  };

  /** \brief Representation of a collision checking request */
  struct CollisionRequest
  {
    CollisionRequest() : distance(false),
                         cost(false),
                         contacts(false),
                         max_contacts(1),
                         max_contacts_per_pair(1),
                         max_cost_sources(1),
                         min_cost_density(0.2),
                         verbose(false)
    {
    }
    virtual ~CollisionRequest() {}

    /** \brief The group name to check collisions for (optional; if empty, assume the complete robot) */
    std::string group_name;

    /** \brief If true, compute proximity distance */
    bool        distance;

    /** \brief If true, a collision cost is computed */
    bool        cost;

    /** \brief If true, compute contacts */
    bool        contacts;

    /** \brief Overall maximum number of contacts to compute */
    std::size_t max_contacts;

    /** \brief Maximum number of contacts to compute per pair of bodies (multiple bodies may be in contact at different configurations) */
    std::size_t max_contacts_per_pair;

    /** \brief When costs are computed, this value defines how many of the top cost sources should be returned */
    std::size_t max_cost_sources;

    /** \brief When costs are computed, this is the minimum cost density for a CostSource to be included in the results */
    double      min_cost_density;

    /** \brief Function call that decides whether collision detection should stop. */
    boost::function<bool(const CollisionResult&)>
                is_done;

    /** \brief Flag indicating whether information about detected collisions should be reported */
    bool        verbose;
  };

}

#endif
