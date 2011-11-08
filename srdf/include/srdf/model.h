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

/* Author Ioan Sucan */

#ifndef SRDF_MODEL_
#define SRDF_MODEL_

#include <map>
#include <string>
#include <vector>
#include <utility>
#include <urdf_interface/model.h>
#include <tinyxml.h>

/// Main namespace
namespace srdf
{

    /** \brief Representation of semantic information about the robot */
    class Model
    {
    public:

        Model(void)
        {
        }

        ~Model(void)
        {
        }

        /// \brief Load Model from TiXMLElement
        bool initXml(const urdf::ModelInterface &urdf_model, TiXmlElement *xml);
        /// \brief Load Model from TiXMLDocument
        bool initXml(const urdf::ModelInterface &urdf_model, TiXmlDocument *xml);
        /// \brief Load Model given a filename
        bool initFile(const urdf::ModelInterface &urdf_model, const std::string& filename);
        /// \brief Load Model from a XML-string
        bool initString(const urdf::ModelInterface &urdf_model, const std::string& xmlstring);

        /** \brief A group consists of a set of joints and the
            corresponding descendant links. There are multiple ways to
            specify a group. Directly specifying joints, links or
            chains, or referring to other defined groups. */
        struct Group
        {
            /// The name of the group
            std::string                                       name_;

            /// Directly specified joints to be included in the
            /// group. Descendent links should be implicitly
            /// considered to be part of the group, although this
            /// parsed does not add them to links_. The joints are
            /// checked to be in the corresponding URDF.
            std::vector<std::string>                          joints_;

            /// Directly specified links to be included in the
            /// group. Parent joints should be implicitly considered
            /// to be part of the group. The links are checked to be
            /// in the corresponding URDF.
            std::vector<std::string>                          links_;

            /// Specify a chain of links (and the implicit joints) to
            /// be added to the group. Each chain is specified as a
            /// pair of base link and tip link. It is checked that the
            /// chain is indeed a chain in the specified URDF.
            std::vector<std::pair<std::string, std::string> > chains_;

            /// It is sometimes convenient to refer to the content of
            /// another group. A group can include the content of the
            /// referenced groups
            std::vector<std::string>                          subgroups_;
        };

        /// In addition to the joints specified in the URDF it is
        /// sometimes convenient to add special (virtual) joints. For
        /// example, to connect the robot to the environment in a
        /// meaningful way.
        struct VirtualJoint
        {
            /// The name of the new joint
            std::string name_;

            /// The type of this new joint. This can be "fixed" (0 DOF), "planar" (3 DOF: x,y,yaw) or "floating" (6DOF)
            std::string type_;

            /// The transform applied by this joint to the robot model brings that model to a particular frame.
            std::string parent_frame_;

            /// The link this joint applies to
            std::string child_link_;
        };

        /// Representation of an end effector
        struct EndEffector
        {
            /// The name of the end effector
            std::string name_;

            /// The name of the link this end effector connects to
            std::string parent_link_;

            /// The name of the group that includes the joints & links this end effector consists of
            std::string component_group_;
        };

        /// A named state for a particular group
        struct GroupState
        {
            /// The name of the state
            std::string                                 name_;

            /// The name of the group this state is specified for
            std::string                                 group_;

            /// The values of joints for this state. Each joint can have a value. We use a vector for the 'value' to support multi-DOF joints
            std::map<std::string, std::vector<double> > joint_values_;
        };

        /// Get the name of this model
        const std::string& getName(void) const
        {
            return name_;
        }

        /// Get the list of pairs of links that need not be checked for collisions (because they can never touch given the geometry and kinematics of the robot)
        const std::vector<std::pair<std::string, std::string> >& getDisabledCollisions(void) const
        {
            return disabled_collisions_;
        }

        /// Get the list of groups defined for this model
        const std::vector<Group>& getGroups(void) const
        {
            return groups_;
        }

        /// Get the list of virtual joints defined for this model
        const std::vector<VirtualJoint>& getVirtualJoints(void) const
        {
            return virtual_joints_;
        }

        /// Get the list of end effectors defined for this model
        const std::vector<EndEffector>& getEndEffectors(void) const
        {
            return end_effectors_;
        }

        /// Get the list of group states defined for this model
        const std::vector<GroupState>& getGroupStates(void) const
        {
            return group_states_;
        }

        /// Clear the model
        void clear(void);

    private:

        void loadVirtualJoints(const urdf::ModelInterface &urdf_model, TiXmlElement *robot_xml);
        void loadGroups(const urdf::ModelInterface &urdf_model, TiXmlElement *robot_xml);
        void loadGroupStates(const urdf::ModelInterface &urdf_model, TiXmlElement *robot_xml);
        void loadEndEffectors(const urdf::ModelInterface &urdf_model, TiXmlElement *robot_xml);
        void loadDisabledCollisions(const urdf::ModelInterface &urdf_model, TiXmlElement *robot_xml);

        std::string                                       name_;
        std::vector<Group>                                groups_;
        std::vector<GroupState>                           group_states_;
        std::vector<VirtualJoint>                         virtual_joints_;
        std::vector<EndEffector>                          end_effectors_;
        std::vector<std::pair<std::string, std::string> > disabled_collisions_;
    };

}
#endif
