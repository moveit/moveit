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
#include <boost/noncopyable.hpp>

namespace planning_scene
{

    class PlanningScene;
    typedef boost::shared_ptr<PlanningScene> PlanningScenePtr;
    typedef boost::shared_ptr<const PlanningScene> PlanningSceneConstPtr;

    /** \brief This class maintains the representation of the
        environment as seen by a planning instance. The environment
        geometry, the robot geometry and state are maintained. */
    class PlanningScene : private boost::noncopyable
    {
    public:

        /** \brief Constructor. Allocate an empty planning scene. Before use, this instance needs to be configured
            by calling the configure() function. */
        PlanningScene(void);

        /** \brief Constructor. Allocate a planning scene that is to be maintained as a diff to a \e parent planning scene.
            This representation does not actually copy the parent scene data. Instead, it maintains diffs with respect to the
            specified parent. When data in the parent is modified, the changes are visible in the diff class as well, unless
            the modified objects were previously modified within the diff class (and a modified copy is stored instead).
            It is recommended that the \e parent planning scene is configured before use. Otherwise,
            the configure() function will have to be called on the diff class as well. */
        explicit
        PlanningScene(const PlanningSceneConstPtr &parent);

        virtual ~PlanningScene(void)
        {
        }

        /** \brief Configure this planning scene to use a particular robot model and semantic description of that robot model.
            The information passed in for this function allows the construction of a kinematic model and of all the classed that
            depend on the kinematic model (e.g., collision world/robot classes) */
        bool configure(const boost::shared_ptr<const urdf::Model> &urdf_model,
                       const boost::shared_ptr<const srdf::Model> &srdf_model);

        /** \brief Get the parent scene (whith respect to which the diffs are maintained). This may be empty */
        const PlanningSceneConstPtr& getParent(void) const
        {
            return parent_;
        }

        /** \brief Get the frame in which planning is performed */
        const std::string& getPlanningFrame(void) const
        {
            // if we have an updated set of transforms, return it; otherwise, return the parent one
            return ftf_ ? ftf_->getPlanningFrame() : parent_->getPlanningFrame();
        }

        /** \brief Get the kinematic model for which the planning scene is maintained */
        const planning_models::KinematicModelConstPtr& getKinematicModel(void) const
        {
            // the kinematic model does not change
            return parent_ ? parent_->getKinematicModel() : kmodel_const_;
        }

        /** \brief Get the state at which the robot is assumed to be */
        const planning_models::KinematicState& getCurrentState(void) const
        {
            // if we have an updated state, return it; otherwise, return the parent one
            return kstate_ ? *kstate_ : parent_->getCurrentState();
        }
        /** \brief Get the state at which the robot is assumed to be */
        planning_models::KinematicState& getCurrentState(void);

        /** \brief Get the allowed collision matrix */
        const collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrix(void) const
        {
            return acm_ ? *acm_ : parent_->getAllowedCollisionMatrix();
        }
        /** \brief Get the allowed collision matrix */
        collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrix(void);

        /** \brief Get the set of fixed transforms from known frames to the planning frame */
        const planning_models::TransformsConstPtr& getTransforms(void) const
        {
            // if we have updated transforms, return those
            return (ftf_const_ || !parent_) ? ftf_const_ : parent_->getTransforms();
        }
        /** \brief Get the set of fixed transforms from known frames to the planning frame */
        const planning_models::TransformsPtr& getTransforms(void);

        /** \brief Get the representation of the collision world */
        const collision_detection::CollisionWorldConstPtr& getCollisionWorld(void) const
        {
            // we always have a world representation
            return cworld_const_;
        }
        /** \brief Get the representation of the collision world */
        const collision_detection::CollisionWorldPtr& getCollisionWorld(void)
        {
            // we always have a world representation
            return cworld_;
        }

        /** \brief Get the representation of the collision robot */
        const collision_detection::CollisionRobotConstPtr& getCollisionRobot(void) const
        {
            // if we have an updated robot, return that one
            return (crobot_const_ || !parent_) ? crobot_const_ : parent_->getCollisionRobot();
        }
        /** \brief Get the representation of the collision robot */
        const collision_detection::CollisionRobotPtr& getCollisionRobot(void);

        /** \brief Check whether the current state is in collision */
        void checkCollision(const collision_detection::CollisionRequest& req,
                            collision_detection::CollisionResult &res) const;

        /** \brief Check whether a specified state (\e kstate) is in collision */
        void checkCollision(const collision_detection::CollisionRequest& req,
                            collision_detection::CollisionResult &res,
                            const planning_models::KinematicState &kstate) const;

        /** \brief Check whether a specified state (\e kstate) is in collision, with respect to a given
            allowd collision matrix (\e acm) */
        void checkCollision(const collision_detection::CollisionRequest& req,
                            collision_detection::CollisionResult &res,
                            const planning_models::KinematicState &kstate,
                            const collision_detection::AllowedCollisionMatrix& acm) const;

        /** \brief Check if this planning scene has been configured or not */
        bool isConfigured(void) const
        {
            return parent_ ? configured_ && parent_->isConfigured() : configured_;
        }

        /** \brief Fill the message \e scene with the differences between this instance of PlanningScene with respect to the parent.
            If there is no parent, everything is considered to be a diff and the function behaves like getPlanningSceneMsg() */
        void getPlanningSceneDiffMsg(moveit_msgs::PlanningScene &scene) const;

        /** \brief Construct a message (\e scene) with all the necessary data so that the scene can be later reconstructed to be
            exactly the same using setPlanningSceneMsg() */
        void getPlanningSceneMsg(moveit_msgs::PlanningScene &scene) const;

        /** \brief Apply changes to this planning scene as diffs. A parent is not required to exist. However, the existing data
            in the planning instance is not cleared. Data from the message is only appended (and in cases such as e.g.,
            the robot state, is overwritten). */
        void setPlanningSceneDiffMsg(const moveit_msgs::PlanningScene &scene);

        /** \brief Set this instance of a planning scene to be the same as the one serialized in the \e scene message. */
        void setPlanningSceneMsg(const moveit_msgs::PlanningScene &scene);

        /** \brief Set the current robot state to be \e state. If not
            all joint values are specified, the previously maintained
            joint values are kept. */
        void setCurrentState(const moveit_msgs::RobotState &state);

        /** \brief Set the current robot state */
        void setCurrentState(const planning_models::KinematicState &state);

        /** \brief Get the URDF model used to construct the kinematic model maintained by this planning scene */
        const boost::shared_ptr<const urdf::Model>& getUrdfModel(void) const
        {
            return parent_ ? parent_->getUrdfModel() : urdf_model_;
        }

        /** \brief Get the SRDF model used to construct the kinematic model maintained by this planning scene */
        const boost::shared_ptr<const srdf::Model>& getSrdfModel(void) const
        {
            return parent_ ? parent_->getSrdfModel() : srdf_model_;
        }

        /** \brief Make sure that all the data maintained in this
            scene is local. All unmodified data is copied from the
            parent and the pointer to the parent is discarded. */
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

        planning_models::TransformsPtr                 ftf_;
        planning_models::TransformsConstPtr            ftf_const_;

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
