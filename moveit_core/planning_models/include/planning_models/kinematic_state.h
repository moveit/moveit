/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan, E. Gil Jones */

#ifndef PLANNING_MODELS_KINEMATIC_STATE_
#define PLANNING_MODELS_KINEMATIC_STATE_

#include "planning_models/kinematic_model.h"
#include <boost/scoped_ptr.hpp>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <set>

namespace planning_models
{

    /** @brief Definition of a kinematic state - the parts of the robot
     *   state which can change. Const members are thread safe */
    class KinematicState
    {

    public:

        /** \brief Forward definition of a joint state */;
        class JointState;

        /** \brief Forward definition of a link state */
        class LinkState;

        /** \brief Forward definition of an attached body */
        class AttachedBody;

        /** \brief Forward definition of a joint group state */
        class JointStateGroup;

        /** @brief Definition of a joint state - representation of state for a single joint */
        class JointState
        {
            friend class KinematicState;
        public:

            /** brief Constructs the joint state from the model */
            JointState(const KinematicModel::JointModel* jm);
            ~JointState(void);

            /** \brief Set the value of a particular variable for this joint */
            bool setVariableValue(const std::string &variable, double value);

            /** \brief Sets the internal values from a map of joint variable names to actual values */
            void setVariableValues(const std::map<std::string, double>& value_map);

            /** \brief Sets the internal values from a map of joint
                variable names to actual values. The function also
                fills the missing vector with the variable names that
                were not set. */
            void setVariableValues(const std::map<std::string, double>& value_map, std::vector<std::string>& missing);

            /** \brief Sets the internal values from the supplied vector, which are assumed to be in the required order */
            bool setVariableValues(const std::vector<double>& value_vector);

            /** \brief Sets the internal values from the supplied
                array, which are assumed to be in the required
                order. This function is intended to be fast, does no
                input checking and should be used carefully. */
            void setVariableValues(const double *value_vector);

            /** \brief Sets the internal values from the transform */
            void setVariableValues(const Eigen::Affine3f& transform);

            /** \brief Update the joints that mimic this one. This function is called automatically by the setVariable* functions */
            void updateMimicJoints(void);

            /** \brief Specifies whether or not all values associated with a joint are defined in the
                supplied joint value map */
            bool allVariablesAreDefined(const std::map<std::string, double>& value_map) const;

            /** \brief Checks if the current joint state values are all within the bounds set in the model */
            bool satisfiesBounds(void) const;

            /** \brief Get the name of the model associated with this state */
            const std::string& getName(void) const
            {
                return joint_model_->getName();
            }

            /** \brief Get the type of joint associated with this state */
            KinematicModel::JointModel::JointType getType(void) const
            {
                return joint_model_->getType();
            }

            /** \brief Get the number of variable DOFs for this joint*/
            unsigned int getVariableCount(void) const
            {
                return joint_model_->getVariableCount();
            }

            /** \brief Get the joint state values stored in the required order */
            const std::vector<double>& getVariableValues(void) const
            {
                return joint_state_values_;
            }

            /** \brief Get the required name order for the joint state values */
            const std::vector<std::string>& getVariableNames(void) const
            {
                return joint_model_->getVariableNames();
            }

            /** \brief Get the current variable transform */
            const Eigen::Affine3f& getVariableTransform(void) const
            {
                return variable_transform_;
            }

            /** \brief Get the joint model corresponding to this state*/
            const KinematicModel::JointModel* getJointModel(void) const
            {
                return joint_model_;
            }

            /** \brief The set of variables that make up the state value of a joint are stored in some order. This map
                gives the position of each variable in that order, for each variable name */
            const std::map<std::string, unsigned int>& getVariableIndexMap(void) const
            {
                return joint_model_->getVariableIndexMap();
            }

        private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            /** \brief The joint model this state corresponds to */
            const KinematicModel::JointModel   *joint_model_;

            /** \brief Tthe local transform (computed by forward kinematics) */
            Eigen::Affine3f                     variable_transform_;

            /** \brief The joint values given in the order indicated by joint_variables_index_map_ */
            std::vector<double>                 joint_state_values_;

            /** \brief The set of joints that need to be updated when this one is */
            std::vector<JointState*>            mimic_requests_;
        };

        /** @brief A class storing properties for attached bodies */
        struct AttachedBodyProperties
        {
            /** \brief Default constructor */
            AttachedBodyProperties(void);

            /** \brief Copy constructor */
            AttachedBodyProperties(const AttachedBodyProperties &other);

            ~AttachedBodyProperties(void);

            /** \brief The geometries of the attached body */
            std::vector<shapes::Shape*> shapes_;

            /** \brief The constant transforms applied to the link (needs to be specified by user) */
            std::vector<Eigen::Affine3f>    attach_trans_;

            /** \brief The set of links this body is allowed to touch */
            std::set<std::string>       touch_links_;

            /** \brief string id for reference */
            std::string                 id_;
        };

        /** @brief Object defining bodies that can be attached to robot
         *  links. This is useful when handling objects picked up by
         *  the robot. */
        class AttachedBody
        {
            friend class KinematicState;
        public:

            /** \brief Construct an attached body for a specified \e link. The name of this body is \e id and it consists of \e shapes that
                attach to the link by the transforms \e attach_trans. The set of links that are allowed to be touched by this object is specified by \e touch_links. */
            AttachedBody(const LinkState *link, const std::string &id,
                         const std::vector<shapes::Shape*> &shapes,
                         const std::vector<Eigen::Affine3f> &attach_trans,
                         const std::vector<std::string> &touch_links);

            /** \brief Construct an attached body for a specified \e
                link, but share its properties (name, shapes, fixed
                transforms, touched links) with another instance. This
                allows more efficient copying of states. */
            AttachedBody(const LinkState *link, const boost::shared_ptr<AttachedBodyProperties> &properties);

            ~AttachedBody(void);

            /** \brief Get the name of the attached body */
            const std::string& getName(void) const
            {
                return properties_->id_;
            }

            /** \brief Get the name of the link this body is attached to */
            const std::string& getAttachedLinkName(void) const
            {
                return parent_link_state_->getName();
            }

            /** \brief Get the shapes that make up this attached body */
            const std::vector<shapes::Shape*>& getShapes(void) const
            {
                return properties_->shapes_;
            }

            /** \brief Get the fixed transform (the transforms to the shapes associated with this body) */
            const std::vector<Eigen::Affine3f>& getFixedTransforms(void) const
            {
                return properties_->attach_trans_;
            }

            /** \brief Get the links that the attached body is allowed to touch */
            const std::set<std::string>& getTouchLinks(void) const
            {
                return properties_->touch_links_;
            }

            /** \brief Get the properties of the attached body */
            const boost::shared_ptr<AttachedBodyProperties>& getProperties(void) const
            {
                return properties_;
            }

            /** \brief Get the global transforms for the collision bodies */
            const std::vector<Eigen::Affine3f>& getGlobalCollisionBodyTransforms(void) const
            {
                return global_collision_body_transforms_;
            }

            /** \brief Set the padding for the shapes of this attached object */
            void setPadding(double padding);

            /** \brief Set the scale for the shapes of this attached object */
            void setScale(double scale);

            /** \brief Recompute global_collision_body_transform */
            void computeTransform(void);

        private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            /** \brief The link that owns this attached body */
            const LinkState                          *parent_link_state_;

            /** \brief The properties of the attached body. These can be shared by multiple states (they do not change often) */
            boost::shared_ptr<AttachedBodyProperties> properties_;

            /** \brief The global transforms for these attached bodies (computed by forward kinematics) */
    std::vector<Eigen::Affine3f>                  global_collision_body_transforms_;
        };

        /** @brief The state corresponding to a link */
        class LinkState
        {
            friend class KinematicState;
        public:

            /** @brief Constructor */
            LinkState(const KinematicState *state, const KinematicModel::LinkModel* lm);
            ~LinkState(void);

            /** @brief Get the name of link corresponding to this state */
            const std::string& getName(void) const
            {
                return link_model_->getName();
            }

            /** @brief Get the kinematic state that this link state is part of*/
            const KinematicState* getKinematicState(void) const
            {
                return kinematic_state_;
            }

            /** @brief Set the link state to the input transform */
            void updateGivenGlobalLinkTransform(const Eigen::Affine3f& transform);

            /** \brief Recompute global_collision_body_transform and global_link_transform */
            void computeTransform(void);

            /** \brief Update all attached bodies given set link transforms */
            void updateAttachedBodies(void);

            /** @brief Get the link model corresponding to this state */
            const KinematicModel::LinkModel* getLinkModel(void) const
            {
                return link_model_;
            }

            /** @brief Get the joint state corresponding to the parent joint of this link */
            const JointState* getParentJointState(void) const
            {
                return parent_joint_state_;
            }

            /** @brief Get the link state corresponding to the parent link of this link */
            const LinkState* getParentLinkState(void) const
            {
                return parent_link_state_;
            }

            /** @brief Get all the bodies attached to this link */
            const std::vector<AttachedBody*>& getAttachedBodies(void) const
            {
                return attached_body_vector_;
            }

            /** @brief Get the attached body with name \e id */
            const AttachedBody* getAttachedBody(const std::string &id) const;

            /** @brief Get the global transform for this link */
            const Eigen::Affine3f& getGlobalLinkTransform(void) const
            {
                return global_link_transform_;
            }

            /** @brief Get the global transform for the collision body associated with this link */
            const Eigen::Affine3f& getGlobalCollisionBodyTransform(void) const
            {
                return global_collision_body_transform_;
            }

            /**
                @brief Attach a body to this link
                @param id The string id associated with the attached body
                @param shapes The shapes that make up the attached body
                @param attach_trans The desired transform between this link and the attached body
                @param touch_links The set of links that the attached body is allowed to touch
            */
            void attachBody(const std::string &id,
                            const std::vector<shapes::Shape*> &shapes,
                            const std::vector<Eigen::Affine3f> &attach_trans,
                            const std::vector<std::string> &touch_links);

            /**
                @brief Attach a body to this link
                @param properties The properties associated with this body
            */
            void attachBody(const boost::shared_ptr<AttachedBodyProperties> &properties);

            /**
                @brief Clear the attached body
                @param id The name of the attached body to clear
            */
            bool clearAttachedBody(const std::string &id);

            /** @brief Clear all attached bodies */
            void clearAttachedBodies(void);

        private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            /** \brief The kinematic state this link is part of */
            const KinematicState            *kinematic_state_;

            const KinematicModel::LinkModel *link_model_;

            const JointState                *parent_joint_state_;

            const LinkState                 *parent_link_state_;

            std::vector<AttachedBody*>       attached_body_vector_;

            /** \brief The global transform this link forwards (computed by forward kinematics) */
            Eigen::Affine3f                      global_link_transform_;

            /** \brief The global transform for this link (computed by forward kinematics) */
            Eigen::Affine3f                      global_collision_body_transform_;
        };



        /** @class JointStateGroup
         *  @brief The joint state corresponding to a group
         */
        class JointStateGroup
        {
        public:

            /**
             *  @brief Default constructor
             *  @param state A pointer to the kinematic state
             *  @param jmg The joint model group corresponding to this joint state
             */
            JointStateGroup(KinematicState *state, const KinematicModel::JointModelGroup *jmg);
            ~JointStateGroup(void);

            /** \brief Get the kinematic state this link is part of */
            const KinematicState* getKinematicState(void) const
            {
                return kinematic_state_;
            }

            /** \brief Get the joint model corresponding to this joint state group */
            const KinematicModel::JointModelGroup* getJointModelGroup(void)
            {
                return joint_model_group_;
            }

            /** \brief Get the name of the joint model group corresponding to this joint state*/
            const std::string& getName(void) const
            {
                return joint_model_group_->getName();
            }

            /** \brief Get the number of (active) DOFs for the joint model group corresponding to this state*/
            unsigned int getVariableCount(void) const
            {
                return joint_model_group_->getVariableCount();
            }

            /** \brief Perform forward kinematics starting at the roots
                within a group. Links that are not in the group are also
                updated, but transforms for joints that are not in the
                group are not recomputed.  */
            bool setStateValues(const std::vector<double>& joint_state_values);

            /** \brief Perform forward kinematics starting at the roots
                within a group. Links that are not in the group are also
                updated, but transforms for joints that are not in the
                group are not recomputed.  */
            void setStateValues(const std::map<std::string, double>& joint_state_map);

            /** \brief Perform forward kinematics starting at the roots
                within a group. Links that are not in the group are also
                updated, but transforms for joints that are not in the
                group are not recomputed.  */
	    void setStateValues(const sensor_msgs::JointState& js);

            /** Compute transforms using current joint values */
            void updateLinkTransforms(void);

            /** \brief Check if a joint is part of this group */
            bool hasJointState(const std::string &joint) const;

            /** \brief Check if a link is updated by this group */
            bool updatesLinkState(const std::string& joint) const;

            /** \brief Get a joint state by its name */
            JointState* getJointState(const std::string &joint) const;

            /** \brief Get current joint values */
            void getGroupStateValues(std::vector<double>& joint_state_values) const;

            /** \brief Get a map between variable names and joint state values */
            void getGroupStateValues(std::map<std::string, double>& joint_state_values) const;

            /** \brief Bring the group to a default state. All joints are
                at 0. If 0 is not within the bounds of the joint, the
                middle of the bounds is used. */
            void setToDefaultValues(void);

            /** \brief Set the group to a named default state. Return false on failure */
            bool setToDefaultState(const std::string &name);

            /** \brief Sample a random state in accordance with the type of joints employed */
            void setToRandomValues(void);

            /** \brief Get the state corresponding to root joints in this group*/
            const std::vector<JointState*>& getJointRoots(void) const
            {
                return joint_roots_;
            }

            /** \brief Get the joint names corresponding to this joint state*/
            const std::vector<std::string>& getJointNames(void) const
            {
                return joint_model_group_->getJointModelNames();
            }

            /** \brief Get the vector of joint state for this group*/
            const std::vector<JointState*>& getJointStateVector(void) const
            {
                return joint_state_vector_;
            }

            /** \brief Return the instance of a random number generator */
            random_numbers::RandomNumberGenerator& getRandomNumberGenerator(void);

        private:

            /** \brief The kinematic state this group is part of */
            KinematicState                        *kinematic_state_;

            /** \brief The model of the group that corresponds to this state */
            const KinematicModel::JointModelGroup *joint_model_group_;

            /** \brief Joint instances in the order they appear in the group state */
            std::vector<JointState*>               joint_state_vector_;

            /** \brief A map from joint names to their instances */
            std::map<std::string, JointState*>     joint_state_map_;

            /** \brief The list of joints that are roots in this group */
            std::vector<JointState*>               joint_roots_;

            /** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
            std::vector<LinkState*>                updated_links_;

            /** \brief For certain operations a group needs a random number generator. However, it may be slightly expensive
                to allocate the random number generator if many state instances are generated. For this reason, the generator
                is allocated on a need basis, by the getRandomNumberGenerator() function. Never use the rng_ member directly, but call
                getRandomNumberGenerator() instead. */
            boost::scoped_ptr<random_numbers::RandomNumberGenerator> rng_;
        };

        /** \brief Create a state corresponding to a given kinematic model */
        KinematicState(const KinematicModelConstPtr &kinematic_model);

        /** \brief Copy constructor */
        KinematicState(const KinematicState& state);

        ~KinematicState(void);

        /** @brief Set the joint state values from a vector of values.
         *  Assumes that the order of the values matches the order of the joints in the state.
         *  Should only be used for fast setting of joint values. */
        bool setStateValues(const std::vector<double>& joint_state_values);

        /** @brief Set the joint state values from a  map of values (matching string ids to actual joint values) */
        void setStateValues(const std::map<std::string, double>& joint_state_map);

        /** @brief Set the joint state values from a  map of values (matching string ids to actual joint values).
         *  Also returns the set of joint names for which joint states have not been provided.*/
        void setStateValues(const std::map<std::string, double>& joint_state_map, std::vector<std::string>& missing);

        /** @brief Set the joint state values from a joint state message */
        void setStateValues(const sensor_msgs::JointState& msg);

        /** @brief Get the joint state values. The order in which the values are specified matches the order
         *  of the joints in the KinematicModel corresponding to this state.*/
        void getStateValues(std::vector<double>& joint_state_values) const;

        /** @brief Get the joint state values as a map between joint state names and values*/
        void getStateValues(std::map<std::string, double>& joint_state_values) const;

        /** @brief Get the joint state values in a sensor_msgs::JointState msg */
        void getStateValues(sensor_msgs::JointState& msg) const;

        /** \brief Perform forward kinematics with the current values and update the link transforms.*/
        void updateLinkTransforms(void);

        /** \brief Update the state after setting a particular link to the input global transform pose.*/
        bool updateStateWithLinkAt(const std::string& link_name, const Eigen::Affine3f& transform);

        /** \brief Get the kinematic model corresponding to this state.*/
        const KinematicModelConstPtr& getKinematicModel(void) const
        {
            return kinematic_model_;
        }

        /** \brief Get the number of (active) DOFs in the model corresponding to this state.*/
        unsigned int getVariableCount(void) const
        {
            return kinematic_model_->getVariableCount();
        }

        /** \brief Set all joints to their default values*/
        void setToDefaultValues(void);

        /** \brief Sample a random state in accordance with the type of joints employed */
        void setToRandomValues(void);

        /** \brief Check if a particular set of joints satisifes its bounds.*/
        bool satisfiesBounds(const std::vector<std::string>& joints) const;

        /** \brief Check if a joint satisifes its bounds.*/
        bool satisfiesBounds(const std::string& joint) const;

        /** \brief Get a group by its name */
        const JointStateGroup* getJointStateGroup(const std::string &name) const;

        /** \brief Get a group by its name */
        JointStateGroup* getJointStateGroup(const std::string &name);

        /** \brief Check if a group exists */
        bool hasJointStateGroup(const std::string &name) const;

        /** \brief Check if a joint is part of this state */
        bool hasJointState(const std::string &joint) const;

        /** \brief Check if a link is updated by this state */
        bool hasLinkState(const std::string& joint) const;

        /** \brief Get a joint state by its name */
        JointState* getJointState(const std::string &joint) const;

        /** \brief Get a link state by its name */
        LinkState* getLinkState(const std::string &link) const;

        /** \brief Get a vector of joint state corresponding to this kinematic state */
        const std::vector<JointState*>& getJointStateVector(void) const
        {
            return joint_state_vector_;
        }

        /** \brief Get all the maintained link states, in the same order as the link models maintained by the kinematic model */
        const std::vector<LinkState*>& getLinkStateVector(void) const
        {
            return link_state_vector_;
        }

        /** \brief Get a map that returns JointStateGroups corresponding to names*/
        const std::map<std::string, JointStateGroup*>& getJointStateGroupMap(void) const
        {
            return joint_state_group_map_;
        }

        /** \brief Get the names of all joint groups in the model corresponding to this state*/
        void getJointStateGroupNames(std::vector<std::string>& names) const;

        /** \brief Get all bodies attached to the model corresponding to this state */
        void getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const;

        /** \brief Clear all attached bodies */
        void clearAttachedBodies(void);

        /** \brief Print information about the constructed model */
        void printStateInfo(std::ostream &out = std::cout) const;

        /** \brief Print the pose of every link */
        void printTransforms(std::ostream &out = std::cout) const;

        void printTransform(const std::string &st, const Eigen::Affine3f &t, std::ostream &out = std::cout) const;

        /** \brief Get the global transform applied to the entire tree of links */
        const Eigen::Affine3f& getRootTransform(void) const;

        /** \brief Set the global transform applied to the entire tree of links */
        void setRootTransform(const Eigen::Affine3f &transform);

        /** \brief Return the instance of a random number generator */
        random_numbers::RandomNumberGenerator& getRandomNumberGenerator(void);

        /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
         *  @param color The color for the marker
         *  @param ns The namespace for the markers
         *  @param dur The ros::Duration for which the markers should stay visible
         *  @param arr The returned marker array
         *  @param link_names The list of link names for which the markers should be created.
         */
        void getRobotMarkers(const std_msgs::ColorRGBA& color,
                             const std::string& ns,
                             const ros::Duration& dur,
                             visualization_msgs::MarkerArray& arr,
                             const std::vector<std::string> &link_names) const;

        /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
         *  @param arr The returned marker array
         *  @param link_names The list of link names for which the markers should be created.
         */
        void getRobotMarkers(visualization_msgs::MarkerArray& arr,
                             const std::vector<std::string> &link_names) const;

        /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
         *  @param color The color for the marker
         *  @param ns The namespace for the markers
         *  @param dur The ros::Duration for which the markers should stay visible
         *  @param arr The returned marker array
         */
        void getRobotMarkers(const std_msgs::ColorRGBA& color,
                             const std::string& ns,
                             const ros::Duration& dur,
                             visualization_msgs::MarkerArray& arr) const;

        /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
         *  @param arr The returned marker array
         */
        void getRobotMarkers(visualization_msgs::MarkerArray& arr) const;

      // void getPaddedRobotMarkers(const std::map<std::string, double>& link_padding_map,
      //                            const std_msgs::ColorRGBA& color,
      //                            const std::string& ns,
      //                            const ros::Duration& dur
      //                            visualization_msgs::MarkerArray& arr);


      // void getPaddedRobotMarkers(visualization_msgs::MarkerArray& arr,
      //                            const std::vector<std::string>* link_names = NULL) const;


        KinematicState& operator=(const KinematicState &other);

    private:

        void buildState(void);
        void copyFrom(const KinematicState &ks);

        KinematicModelConstPtr                  kinematic_model_;

        std::vector<JointState*>                joint_state_vector_;
        std::map<std::string, JointState*>      joint_state_map_;

        /** \brief The states for all the links in the robot */
        std::vector<LinkState*>                 link_state_vector_;

        /** \brief A map from link names to their corresponding states */
        std::map<std::string, LinkState*>       link_state_map_;

        /** \brief Additional transform to be applied to the tree of links */
        Eigen::Affine3f                             root_transform_;

        /** \brief A map from group names to instances of the group state */
        std::map<std::string, JointStateGroup*> joint_state_group_map_;

        /** \brief For certain operations a state needs a random number generator. However, it may be slightly expensive
            to allocate the random number generator if many state instances are generated. For this reason, the generator
            is allocated on a need basis, by the getRandomNumberGenerator() function. Never use the rng_ member directly, but call
            getRandomNumberGenerator() instead. */
        boost::scoped_ptr<random_numbers::RandomNumberGenerator> rng_;
    };

    typedef boost::shared_ptr<KinematicState> KinematicStatePtr;
    typedef boost::shared_ptr<const KinematicState> KinematicStateConstPtr;

}

#endif
