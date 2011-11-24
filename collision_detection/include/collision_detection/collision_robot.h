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
#include <moveit_msgs/LinkPadding.h>

namespace collision_detection
{

    class CollisionRobot
    {
    public:

        CollisionRobot(const planning_models::KinematicModelConstPtr &kmodel, double padding = 0.0, double scale = 1.0);
        CollisionRobot(const CollisionRobot &other);

        virtual ~CollisionRobot(void)
        {
        }

        /** \brief Check for self collision. Any collision between any pair of links is considered.  */
        virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state) const = 0;

        /** \brief Check for self collision. Allowed collisions are ignored. */
        virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state, const AllowedCollisionMatrix &acm) const = 0;

        /** \brief Check for collision with a different robot (possibly a different kinematic model as well). Any collision between any pair of links is considered. */
        virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                         const CollisionRobot &other_robot, const planning_models::KinematicState &other_state) const = 0;

        /** \brief Check for collision with a different robot (possibly a different kinematic model as well). Allowed collisions are ignored. */
        virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const planning_models::KinematicState &state,
                                         const CollisionRobot &other_robot, const planning_models::KinematicState &other_state,
                                         const AllowedCollisionMatrix &acm) const = 0;

        void setLinkPadding(const std::string &link_name, double padding);
        double getLinkPadding(const std::string &link_name) const;
        void setLinkPadding(const std::map<std::string, double> &padding);
        const std::map<std::string, double> &getLinkPadding(void) const;
        void setLinkScale(const std::string &link_name, double scale);
        double getLinkScale(const std::string &link_name) const;
        void setLinkScale(const std::map<std::string, double> &scale);
        const std::map<std::string, double> &getLinkScale(void) const;
        void setPadding(double padding);
        void setScale(double scale);
        void setPadding(const std::vector<moveit_msgs::LinkPadding> &padding);
        void getPadding(std::vector<moveit_msgs::LinkPadding> &padding) const;

    protected:

        virtual void updatedPaddingOrScaling(const std::vector<std::string> &links);

        planning_models::KinematicModelConstPtr kmodel_;
        std::map<std::string, double>           link_padding_;
        std::map<std::string, double>           link_scale_;
    };

    typedef boost::shared_ptr<CollisionRobot> CollisionRobotPtr;
    typedef boost::shared_ptr<const CollisionRobot> CollisionRobotConstPtr;
}

#endif
