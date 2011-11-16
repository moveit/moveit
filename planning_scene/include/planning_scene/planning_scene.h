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

#ifndef PLANNING_SCENE_PLANNING_SCENE_
#define PLANNING_SCENE_PLANNING_SCENE_

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <collision_detection/collision_world.h>
#include <moveit_msgs/PlanningScene.h>

namespace planning_scene
{

    class PlanningScene
    {
    public:
        PlanningScene(void) : configured_(false)
        {
        }

        virtual ~PlanningScene(void)
        {
        }

        bool configure(const urdf::Model &urdf_model, const srdf::Model &srdf_model);

        const std::string& getPlanningFrame(void) const
        {
            return tf_->getPlanningFrame();
        }

        const planning_models::KinematicModelPtr& getKinematicModel(void) const
        {
            return kmodel_;
        }

        const planning_models::KinematicState& getCurrentState(void) const
        {
            return *kstate_;
        }

        const planning_models::TransformsPtr& getTransforms(void) const
        {
            return tf_;
        }

        const collision_detection::CollisionWorldPtr& getCollisionWorld(void) const
        {
            return cworld_;
        }

        const collision_detection::CollisionRobotPtr& getCollisionRobot(void) const
        {
            return crobot_;
        }

        const collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrix(void) const
        {
            return acm_;
        }

        void checkCollision(const collision_detection::CollisionRequest& req, 
                            collision_detection::CollisionResult &res,
                            const planning_models::KinematicState &kstate) const;

        void checkCollision(const collision_detection::CollisionRequest& req, 
                            collision_detection::CollisionResult &res,
                            const planning_models::KinematicState &kstate,
                            const collision_detection::AllowedCollisionMatrix& acm) const;

        bool isConfigured(void) const
        {
            return configured_;
        }

        void getPlanningSceneMsg(moveit_msgs::PlanningScene &scene) const;
        void setPlanningSceneMsg(const moveit_msgs::PlanningScene &scene);

    protected:

        bool processCollisionObjectMsg(const moveit_msgs::CollisionObject &object);
        bool processAttachedCollisionObjectMsg(const moveit_msgs::AttachedCollisionObject &object);
        void processCollisionMapMsg(const moveit_msgs::CollisionMap &map);

        planning_models::KinematicModelPtr            kmodel_;
        planning_models::TransformsPtr                tf_;
        planning_models::KinematicStatePtr            kstate_;
        collision_detection::CollisionRobotPtr        crobot_;
      collision_detection::CollisionRobotPtr        crobot_unpadded_;
        collision_detection::CollisionWorldPtr        cworld_;
        collision_detection::AllowedCollisionMatrix   acm_;
        bool                                          configured_;

    };

    typedef boost::shared_ptr<PlanningScene> PlanningScenePtr;
    typedef boost::shared_ptr<const PlanningScene> PlanningSceneConstPtr;
}

#endif
