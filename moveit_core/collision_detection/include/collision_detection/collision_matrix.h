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

/** \author Ioan Sucan, E. Gil Jones */

#ifndef COLLISION_DETECTION_COLLISION_MATRIX_
#define COLLISION_DETECTION_COLLISION_MATRIX_

#include "collision_detection/collision_common.h"
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <boost/function.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>

namespace collision_detection
{

    /** \brief Any pair of bodies can have a collision state associated to it */
    namespace AllowedCollision
    {
        enum Type
            {
                /** \brief Collisions between the pair of bodies is never ok, i.e. if these two bodies are colliding in 
                 a particular configuration of the robot, that configuration is considered to be in collision*/
                NEVER,

                /** \brief Collisions between this pair of bodies does not imply that the robot configuration is in
                 collision. There is no need to explicitly do a computation on this pair of bodies 
                 unless contact information is desired.*/
                ALWAYS,

                /** \brief The collision is allowed depending on a predicate evaluated on the produced contact. If the 
                 predicate returns true, this particular contact is deemed ok (or allowed), i.e. the contact does not 
                imply that the two bodies are in collision*/
                CONDITIONAL
            };
    }

    /** \brief Signature of predicate that decides whether a contact is allowed or not (when AllowedCollision::Type is CONDITIONAL) */
    typedef boost::function<bool(collision_detection::Contact&)> DecideContactFn;

    /** @class AllowedCollisionMatrix
     *  @brief Definition of a structure for the allowed collision matrix. All elements in the collision world are names.
     *   This class represents which collisions are allowed to happen and which are not. */
    class AllowedCollisionMatrix
    {
    public:

        AllowedCollisionMatrix(void);

      /** @brief Instantiate using a vector of names (corresponding to all the elements in the collision world).
       *  @param names a vector of names (corresponding to all the elements in the collision world). Elements could 
       *  represent bodies or namespaces (containing a set of bodies).
       *  @param allowed If false, indicates that collisions between all elements must be checked for and no collisions 
       *  will be ignored. */
        AllowedCollisionMatrix(const std::vector<std::string>& names, bool allowed = false);

      /** @brief Construct from a message*/
        AllowedCollisionMatrix(const moveit_msgs::AllowedCollisionMatrix &msg);

      /** @brief Copy constructor*/
        AllowedCollisionMatrix(const AllowedCollisionMatrix& acm);

      /** @brief Get the type of the allowed collision between two elements.
       *  @param name1 name of first element
       *  @param name2 name of second element
       *  @param allowed_collision_type The allowed collision type*/
        bool getAllowedCollision(const std::string& name1, const std::string& name2, AllowedCollision::Type& allowed_collision_type) const;

      /** @brief Get the allowed collision flag between two elements.
       *  @param name1 name of first element
       *  @param name2 name of second element
       *  @param fn A callback function that is used to decide if collisions are allowed between the two elements
       *  if the allowed collision flag is CONDITIONAL*/
        bool getAllowedCollision(const std::string& name1, const std::string& name2, DecideContactFn& fn) const;

      /** @brief Check if the allowed collision matrix has an entry for a pair of elements
       *  @param name1 name of first element
       *  @param name2 name of second element*/
        bool hasEntry(const std::string& name1, const std::string &name2) const;

      /** @brief Remove an entry corresponding to a pair of elements
       *  @param name1 name of first element
       *  @param name2 name of second element*/
        void removeEntry(const std::string& name1, const std::string &name2);

      /** @brief Remove all entries corresponding to a namespace
       *  @param name namespace*/
        void removeEntry(const std::string& name);

      /** @brief Set an entry corresponding to a pair of elements
       *  @param name1 name of first element
       *  @param name2 name of second element
       *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions 
       *  will be ignored. If true, indicates that collisions between two elements are ok and an explicit collision 
       *  computation is not necessary (unless contacts are required).*/
        void setEntry(const std::string& name1, const std::string& name2, bool allowed);

      /** @brief Set an entry corresponding to a pair of elements
       *  @param name1 name of first element
       *  @param name2 name of second element
       *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions 
       *  will be ignored. If true, indicates that collisions between two elements are ok and an explicit collision 
       *  computation is not necessary (unless contacts are required).*/
        void setEntry(const std::string& name1, const std::string& name2, const DecideContactFn &fn);

      /** @brief Set an entry corresponding to a namespace
       *  @param name namespace
       *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions 
       *  will be ignored. If true, indicates that collisions between two elements are ok and an explicit collision 
       *  computation is not necessary (unless contacts are required).*/
        void setEntry(const std::string& name, bool allowed);

      /** @brief Set an entry corresponding to a element and a set of other elements
       *  @param name name of first element
       *  @param other_names names of all other elements (or namespaces) to pair with first element. The collision 
       *  matrix entries will be set for all such pairs.
       *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions 
       *  will be ignored. If true, indicates that collisions between two elements are ok and an explicit collision 
       *  computation is not necessary (unless contacts are required).*/
        void setEntry(const std::string& name, const std::vector<std::string>& other_names, bool allowed);

      /** @brief Set an entry corresponding to all pairs in two sets of elements
       *  @param names1 First set of names
       *  @param names2 Second set of names
       *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions 
       *  will be ignored. If true, indicates that collisions between two elements are ok and an explicit collision 
       *  computation is not necessary (unless contacts are required).*/
        void setEntry(const std::vector<std::string> &names1, const std::vector<std::string> &names2, bool allowed);

      /** @brief Set an entry corresponding to all pairs 
       *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions 
       *  will be ignored. If true, indicates that collisions between two elements are ok and an explicit collision 
       *  computation is not necessary (unless contacts are required).*/
        void setEntry(bool allowed);

      /** @brief Get the names of all entries in the collision matrix */
        void getAllEntryNames(std::vector<std::string>& names) const;

      /** @brief Get the allowed collision matrix as a message */
        void getMessage(moveit_msgs::AllowedCollisionMatrix &msg) const;

      /** @brief Clear the allowed collision matrix */
        void clear(void);

      /** @brief Get the size of the allowed collision matrix*/
        std::size_t getSize(void) const
        {
            return entries_.size();
        }

      /** @brief Print the allowed collision matrix */
        void print(std::ostream& out) const;

    private:

        std::map<std::string, std::map<std::string, AllowedCollision::Type> > entries_;
        std::map<std::string, std::map<std::string, DecideContactFn> >        allowed_contacts_;
    };

    typedef boost::shared_ptr<AllowedCollisionMatrix> AllowedCollisionMatrixPtr;
    typedef boost::shared_ptr<const AllowedCollisionMatrix> AllowedCollisionMatrixConstPtr;
}

#endif
