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

#include <planning_models/transforms.h>
#include <ros/console.h>

planning_models::Transforms::Transforms(const std::string &target_frame) : target_frame_(target_frame)
{
    btTransform t;
    t.setIdentity();
    transforms_[target_frame_] = t;
}

planning_models::Transforms::~Transforms(void)
{
}

const std::string& planning_models::Transforms::getPlanningFrame(void) const
{
    return target_frame_;
}

bool planning_models::Transforms::isFixedFrame(const std::string &frame) const
{
    return transforms_.find(frame) != transforms_.end();
}

const btTransform& planning_models::Transforms::getTransformToTargetFrame(const std::string &from_frame) const
{
    std::map<std::string, btTransform>::const_iterator it = transforms_.find(from_frame);
    if (it != transforms_.end())
        return it->second;
    throw std::runtime_error("Unable to transform from frame '" + from_frame + "' to frame '" + target_frame_ + "'");
}

const btTransform& planning_models::Transforms::getTransformToTargetFrame(const planning_models::KinematicState &kstate, const std::string &from_frame) const
{
    std::map<std::string, btTransform>::const_iterator it = transforms_.find(from_frame);
    if (it != transforms_.end())
        return it->second;
    if (kstate.getKinematicModel()->getModelFrame() != target_frame_)
        ROS_ERROR("Target frame is assumed to be '%s' but the model of the kinematic state places the robot in frame '%s'",
                  target_frame_.c_str(), kstate.getKinematicModel()->getModelFrame().c_str());
    if (const planning_models::KinematicState::LinkState *state = kstate.getLinkState(from_frame))
        return state->getGlobalLinkTransform();
    ROS_FATAL_STREAM("Unable to transform from frame '" + from_frame + "' to frame '" + target_frame_ + "'");
    // return identity
    return transforms_.find(target_frame_)->second;
}

void planning_models::Transforms::transformVector3(btVector3 &v_out, const btVector3 &v_in, const std::string &from_frame) const
{
    v_out = getTransformToTargetFrame(from_frame) * v_in;
}

void planning_models::Transforms::transformQuaternion(btQuaternion &q_out, const btQuaternion &q_in, const std::string &from_frame) const
{
    q_out = getTransformToTargetFrame(from_frame) * q_in;
}

void planning_models::Transforms::transformMatrix(btMatrix3x3 &m_out, const btMatrix3x3 &m_in, const std::string &from_frame) const
{
    m_out = getTransformToTargetFrame(from_frame).getBasis() * m_in;
}

void planning_models::Transforms::transformTransform(btTransform &t_out, const btTransform &t_in, const std::string &from_frame) const
{
    t_out = getTransformToTargetFrame(from_frame) * t_in;
}

// specify the kinematic state
void planning_models::Transforms::transformVector3(const planning_models::KinematicState &kstate,
                                                   btVector3 &v_out, const btVector3 &v_in, const std::string &from_frame) const
{
    v_out = getTransformToTargetFrame(kstate, from_frame) * v_in;
}

void planning_models::Transforms::transformQuaternion(const planning_models::KinematicState &kstate,
                                                      btQuaternion &q_out, const btQuaternion &q_in, const std::string &from_frame) const
{
    q_out = getTransformToTargetFrame(kstate, from_frame) * q_in;
}

void planning_models::Transforms::transformMatrix(const planning_models::KinematicState &kstate,
                                                  btMatrix3x3 &m_out, const btMatrix3x3 &m_in, const std::string &from_frame) const
{
    m_out = getTransformToTargetFrame(kstate, from_frame).getBasis() * m_in;
}

void planning_models::Transforms::transformTransform(const planning_models::KinematicState &kstate,
                                                     btTransform &t_out, const btTransform &t_in, const std::string &from_frame) const
{
    t_out = getTransformToTargetFrame(kstate, from_frame) * t_in;
}

void planning_models::Transforms::recordTransformFromFrame(const btTransform &t, const std::string &from_frame)
{
    transforms_[from_frame] = t;
}
