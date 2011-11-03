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

/** \author Ioan Sucan */

#ifndef COLLISION_DETECTION_COLLISION_ROBOT_
#define COLLISION_DETECTION_COLLISION_ROBOT_

#include "collision_detection/collision_matrix.h"
#include <planning_models/kinematic_state.h>

namespace collision_detection
{

    class CollisionRobot
    {
    public:

        CollisionRobot(const planning_models::KinematicModelPtr &kmodel) : kmodel_(kmodel)
        {
        }

        /** \brief Check for self collision. Any collision between any pair of links is considered. Contacts are not computed. */
        virtual bool isSelfCollision(const planning_models::KinematicState &state) const = 0;

        /** \brief Check for self collision. Allowed collisions are ignored. Contacts are not computed. */
        virtual bool isSelfCollision(const planning_models::KinematicState &state, const AllowedCollisionMatrix &acm) const = 0;

        /** \brief Check for self collision but also get the list of contacts (collisions). Any collision between any pair of links is considered.
            The maximum total number of contacts to be returned can be specified (\e max_total),
            and the maximum number of contacts per pair of objects that are in collision can also be specified (\e max_per_pair). */
        virtual bool isSelfCollision(const planning_models::KinematicState &state, std::vector<Contact> &contacts,
                                     unsigned int max_total = 1, unsigned int max_per_pair = 1) const = 0;

        /** \brief Check for self collision but also get the list of contacts (collisions). Contacts from allowed collisions are still stored but do not count towards
            the boolean return value of the function. The maximum total number of contacts to be returned can be specified (\e max_total),
            and the maximum number of contacts per pair of objects that are in collision can also be specified (\e max_per_pair). */
        virtual bool isSelfCollision(const planning_models::KinematicState &state, const AllowedCollisionMatrix &acm,
                                     std::vector<Contact> &contacts, unsigned int max_total = 1, unsigned int max_per_pair = 1) const = 0;

        void setLinkPadding(const std::string &link_name, double padding);
        double getLinkPadding(const std::string &link_name) const;
        void setLinkPadding(const std::map<std::string, double> &padding);
        const std::map<std::string, double> &getLinkPadding(void) const;
        void setLinkScale(const std::string &link_name, double scale);
        double getLinkScale(const std::string &link_name) const;
        void setLinkScale(const std::map<std::string, double> &scale);
        const std::map<std::string, double> &getLinkScale(void) const;

    protected:

        planning_models::KinematicModelPtr kmodel_;
        std::map<std::string, double>      link_padding_;
        std::map<std::string, double>      link_scale_;
    };

}

#endif
