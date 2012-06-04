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

/** Author Ioan Sucan */

#ifndef COLLISION_DETECTION_COLLISION_COMMON_
#define COLLISION_DETECTION_COLLISION_COMMON_

#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>

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
  
  /** \brief Representation of a collision checking result */
  struct CollisionResult
  {
    CollisionResult(void) : collision(false),
			    distance(0.0),
			    direction(0.0, 0.0, 0.0),
			    contact_count(0)
    {
    }
    typedef std::map<std::pair<std::string, std::string>, std::vector<Contact> > ContactMap;
    
    /** \brief Clear a previously stored result */
    void clear(void) { *this = CollisionResult(); }
    
    /** \brief True if collision was found, false otherwise */
    bool            collision;
    
    /** \brief Closest distance between two bodies */
    double          distance;
    
    /** \brief Gradient vector associated with collision */
    Eigen::Vector3d direction;
    
    /** \brief Number of contacts returned */
    std::size_t     contact_count;
    
    /** \brief A map returning the pairs of ids of the bodies in contact, plus information about the contacts themselves */
    ContactMap      contacts;
  };
  
  /** \brief Representation of a collision checking request */
  struct CollisionRequest
  {
    CollisionRequest(void) : distance(false),
			     contacts(false),
			     max_contacts(1),
			     max_contacts_per_pair(1),
			     verbose(false)
    {
    }
    
    /** \brief The group name to check collisions for (optional; if empty, assume the complete robot) */
    std::string group_name;
    
    /** \brief If true, compute proximity distance */
    bool        distance;
    
    /** \brief If true, compute contacts */
    bool        contacts;
    
    /** \brief Overall maximum number of contacts to compute*/
    std::size_t max_contacts;
    
    /** \brief Maximum number of contacts to compute per pair of bodies (multiple bodies may be in contact at different configurations) */
    std::size_t max_contacts_per_pair;
    
    /** \brief Flag indicating whether information about detected collisions should be reported */
    bool        verbose;
  };
  
}

#endif
