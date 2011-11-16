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
                /** \brief Collisions are never allowed */
                NEVER,

                /** \brief Collisions are always allowed (no need to check for collision if contacts are not needed) */
                ALWAYS,

                /** \brief The collision is allowed depending on a predicate evaluated on the produced contact */
                CONDITIONAL
            };
    }

    /** \brief Signature of predicate that decides whether a contact is allowed or not (when AllowedCollision::Type is CONDITIONAL) */
    typedef boost::function<bool(Contact&)> DecideContactFn;

    /** \brief Definition of a structure for the allowed collision matrix. All elements in the collision world are names.
        This class represents which collisions are allowed to happen and which are not. */
    class AllowedCollisionMatrix
    {
    public:

        AllowedCollisionMatrix(void);
        AllowedCollisionMatrix(const std::vector<std::string>& names, bool allowed = false);
        AllowedCollisionMatrix(const moveit_msgs::AllowedCollisionMatrix &msg);
        AllowedCollisionMatrix(const AllowedCollisionMatrix& acm);

        bool getAllowedCollision(const std::string& name1, const std::string& name2, AllowedCollision::Type& allowed_collision) const;
        bool getAllowedCollision(const std::string& name1, const std::string& name2, DecideContactFn& fn) const;

        bool hasEntry(const std::string& name1, const std::string &name2) const;

        void removeEntry(const std::string& name1, const std::string &name2);
        void removeEntry(const std::string& name);

        void setEntry(const std::string& name1, const std::string& name2, bool allowed);
        void setEntry(const std::string& name1, const std::string& name2, const DecideContactFn &fn);
        void setEntry(const std::string& name, bool allowed);
        void setEntry(const std::string& name, const std::vector<std::string>& other_names, bool allowed);
        void setEntry(const std::vector<std::string> &names1, const std::vector<std::string> &names2, bool allowed);
        void setEntry(bool allowed);

        void getAllEntryNames(std::vector<std::string>& names) const;
        void getMessage(moveit_msgs::AllowedCollisionMatrix &msg) const;

        void clear(void);

        std::size_t getSize(void) const
        {
            return entries_.size();
        }

        void print(std::ostream& out) const;

    private:

        std::map<std::string, std::map<std::string, AllowedCollision::Type> > entries_;
        std::map<std::string, std::map<std::string, DecideContactFn> >        allowed_contacts_;
    };

}

#endif
