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
    
    class PlanningScene;
    typedef boost::shared_ptr<PlanningScene> PlanningScenePtr;
    typedef boost::shared_ptr<const PlanningScene> PlanningSceneConstPtr;
    
    class PlanningScene
    {
    public:
        PlanningScene(void) : configured_(false)
        {
        }
	
	PlanningScene(const PlanningSceneConstPtr &parent) : parent_(parent), configured_(false)
        {
        }

        virtual ~PlanningScene(void)
        {
        }

        bool configure(const boost::shared_ptr<const urdf::Model> &urdf_model,
                       const boost::shared_ptr<const srdf::Model> &srdf_model);

        const std::string& getPlanningFrame(void) const
        {
	    // if we have an updated set of transforms, return it; otherwise, return the parent one
	    return tf_ ? tf_->getPlanningFrame() : parent_->getPlanningFrame();
	}
	
        const planning_models::KinematicModelConstPtr& getKinematicModel(void) const
        {
	    // the kinematic model does not change
            return parent_ ? parent_->getKinematicModel() : kmodel_const_;
	}

        const planning_models::KinematicState& getCurrentState(void) const
        {
	    // if we have an updated state, return it; otherwise, return the parent one
	    return kstate_ ? *kstate_ : parent_->getCurrentState();
	}

        const collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrix(void) const
        {
            return acm_ ? *acm_ : parent_->getAllowedCollisionMatrix();
        }

        const planning_models::TransformsConstPtr& getTransforms(void) const
        {
	    // if we have updated transforms, return those
            return (tf_const_ || !parent_) ? tf_const_ : parent_->getTransforms();
	}

        const collision_detection::CollisionWorldConstPtr& getCollisionWorld(void) const
        {
	    // if we have an updated world, return that one
            return (cworld_const_ || !parent_) ? cworld_const_ : parent_->getCollisionWorld();
	}

        const collision_detection::CollisionRobotConstPtr& getCollisionRobot(void) const
        {
	    // if we have an updated robot, return that one
            return (crobot_const_ || !parent_) ? crobot_const_ : parent_->getCollisionRobot();
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
            return parent_ ? parent_->isConfigured() : configured_;
        }
	
	void getPlanningSceneDiffMsg(moveit_msgs::PlanningScene &scene) const;	
        void getPlanningSceneMsg(moveit_msgs::PlanningScene &scene) const;
        void setPlanningSceneMsg(const moveit_msgs::PlanningScene &scene);

        void setCurrentState(const moveit_msgs::RobotState &state);
        void setCurrentState(const planning_models::KinematicState &state);

        const boost::shared_ptr<const urdf::Model>& getUrdfModel(void) const
        {
            return parent_ ? parent_->getUrdfModel() : urdf_model_;
        }

        const boost::shared_ptr<const srdf::Model>& getSrdfModel(void) const
        {
            return parent_ ? parent_->getSrdfModel() : srdf_model_;
        }

	void decoupleParent(void);
	
    protected:

        bool processCollisionObjectMsg(const moveit_msgs::CollisionObject &object);
        bool processAttachedCollisionObjectMsg(const moveit_msgs::AttachedCollisionObject &object);
        void processCollisionMapMsg(const moveit_msgs::CollisionMap &map);
	void getPlanningSceneMsgAttachedBodies(moveit_msgs::PlanningScene &scene) const;
	void addPlanningSceneMsgCollisionObject(moveit_msgs::PlanningScene &scene, const std::string &ns) const;
	void getPlanningSceneMsgCollisionObjects(moveit_msgs::PlanningScene &scene) const;
	void getPlanningSceneMsgCollisionMap(moveit_msgs::PlanningScene &scene) const;
	



	PlanningSceneConstPtr                          parent_;
	
        boost::shared_ptr<const urdf::Model>           urdf_model_;
        boost::shared_ptr<const srdf::Model>           srdf_model_;

        planning_models::KinematicModelPtr             kmodel_;
        planning_models::KinematicModelConstPtr        kmodel_const_;

        planning_models::KinematicStatePtr             kstate_;

        planning_models::TransformsPtr                 tf_;
        planning_models::TransformsConstPtr            tf_const_;
	
	collision_detection::CollisionRobotPtr         crobot_unpadded_;
        collision_detection::CollisionRobotPtr         crobot_;
	collision_detection::CollisionRobotConstPtr    crobot_const_;
	
        collision_detection::CollisionWorldPtr         cworld_;
        collision_detection::CollisionWorldConstPtr    cworld_const_;
	
        collision_detection::AllowedCollisionMatrixPtr acm_;
	
        bool                                           configured_;

    };

}


#endif
