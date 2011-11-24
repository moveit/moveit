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

#ifndef PLANNING_MODELS_TRANSFORMS_
#define PLANNING_MODELS_TRANSFORMS_

#include <LinearMath/btTransform.h>
#include <planning_models/kinematic_state.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <map>

namespace planning_models
{
    bool quatFromMsg(const geometry_msgs::Quaternion &qmsg, btQuaternion &q);
    bool poseFromMsg(const geometry_msgs::Pose &tmsg, btTransform &t);
    void msgFromPose(const btTransform &t, geometry_msgs::Pose &tmsg);

    class Transforms
    {
    public:
        Transforms(const std::string &target_frame);  
	Transforms(const Transforms &other);
        ~Transforms(void);

        const std::string& getPlanningFrame(void) const;
        bool isFixedFrame(const std::string &frame) const;
        const std::map<std::string, btTransform>& getAllTransforms(void) const;

        const btTransform& getTransformToTargetFrame(const std::string &from_frame) const;
        void transformVector3(btVector3 &v_out, const btVector3 &v_in, const std::string &from_frame) const;
        void transformQuaternion(btQuaternion &q_out, const btQuaternion &q_in, const std::string &from_frame) const;
        void transformMatrix(btMatrix3x3 &m_out, const btMatrix3x3 &m_in, const std::string &from_frame) const;
        void transformTransform(btTransform &t_out, const btTransform &t_in, const std::string &from_frame) const;

        const btTransform& getTransformToTargetFrame(const planning_models::KinematicState &kstate, const std::string &from_frame) const;
        void transformVector3(const planning_models::KinematicState &kstate, btVector3 &v_out, const btVector3 &v_in, const std::string &from_frame) const;
        void transformQuaternion(const planning_models::KinematicState &kstate, btQuaternion &q_out, const btQuaternion &q_in, const std::string &from_frame) const;
        void transformMatrix(const planning_models::KinematicState &kstate, btMatrix3x3 &m_out, const btMatrix3x3 &m_in, const std::string &from_frame) const;
        void transformTransform(const planning_models::KinematicState &kstate, btTransform &t_out, const btTransform &t_in, const std::string &from_frame) const;

        void recordTransformFromFrame(const btTransform &t, const std::string &from_frame);
        void recordTransform(const geometry_msgs::TransformStamped &transform);
        void recordTransforms(const std::vector<geometry_msgs::TransformStamped> &transforms);

        void getTransforms(std::vector<geometry_msgs::TransformStamped> &transforms) const;

    private:

        std::string                        target_frame_;
        std::map<std::string, btTransform> transforms_;
    };

    typedef boost::shared_ptr<Transforms> TransformsPtr;
    typedef boost::shared_ptr<const Transforms> TransformsConstPtr;
}

#endif
