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

#ifndef SRDF_MODEL_
#define SRDF_MODEL_

#include <string>
#include <vector>
#include <utility>

namespace srdf
{

    class Model
    {
    public:

        struct Group
        {
            std::string                                       name_;
            std::vector<std::pair<std::string, std::string> > chains_;
            std::vector<std::string>                          joints_;
            std::vector<std::string>                          links_;
            std::vector<std::string>                          subgroups_;
        };

        struct VirtualJoint
        {
            std::string name_;
            std::string type_;
            std::string parent_frame_;
            std::string child_link_;
        };

        struct EndEffector
        {
            std::string name_;
            std::string parent_link_;
            std::string component_group_;
        };

        const std::string& getName(void) const
        {
            return name_;
        }

        const std::vector<std::pair<std::string, std::string> >& getDisabledCollisions(void) const
        {
            return disabled_collisions_;
        }

        const std::vector<Group>& getGroups(void) const
        {
            return groups_;
        }

        const std::vector<VirtualJoint>& getVirtualJoints(void) const
        {
            return virtual_joints_;
        }

        const std::vector<EndEffector>& getEndEffectors(void) const
        {
            return end_effectors_;
        }

        //    private:

        std::string                                       name_;
        std::vector<Group>                                groups_;
        std::vector<VirtualJoint>                         virtual_joints_;
        std::vector<EndEffector>                          end_effectors_;
        std::vector<std::pair<std::string, std::string> > disabled_collisions_;

    };

}
#endif
