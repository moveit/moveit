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

#include "kinematic_constraints/constraint_samplers.h"
#include <boost/math/constants/constants.hpp>
#include <ros/console.h>
#include <algorithm>
#include <set>

kinematic_constraints::ConstraintSampler::ConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg) : jmg_(jmg)
{
    if (!jmg_)
        ROS_FATAL("NULL group specified for constraint sampler");
}

kinematic_constraints::ConstraintSampler::~ConstraintSampler(void)
{
}

kinematic_constraints::JointConstraintSampler::JointConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
                                                                      const std::vector<JointConstraint> &jc) : ConstraintSampler(jmg)
{
    // find and keep the constraints that operate on the group we sample
    // also keep bounds for joints as convenient
    const std::map<std::string, unsigned int> &vim = jmg_->getJointVariablesIndexMap();
    std::set<const planning_models::KinematicModel::JointModel*> bounded;
    for (std::size_t i = 0 ; i < jc.size() ; ++i)
    {
        if (!jc[i].enabled())
            continue;
        const planning_models::KinematicModel::JointModel *jm = jc[i].getJointModel();
        if (!jmg_->hasJointModel(jm->getName()))
            continue;
        std::pair<double, double> bounds;
        jm->getVariableBounds(jm->getName(), bounds);
        bounds.first = std::max(bounds.first, jc[i].getDesiredJointPosition() - jc[i].getJointToleranceBelow());
        bounds.second = std::min(bounds.second, jc[i].getDesiredJointPosition() + jc[i].getJointToleranceAbove());
        if (bounds.first > bounds.second)
            ROS_WARN_STREAM("The constraints for joint '" << jm->getName() << "' are such that there are no possible values for this joint. Ignoring constraint.");
        else
        {
            bounded.insert(jm);        
            bounds_.push_back(bounds);
            index_.push_back(vim.find(jm->getName())->second);
        }
    }
    
    // get a separate list of joints that are not bounded; we will sample these randomly
    const std::vector<const planning_models::KinematicModel::JointModel*> &joints = jmg_->getJointModels();
    for (std::size_t i = 0 ; i < joints.size() ; ++i)
        if (bounded.find(joints[i]) == bounded.end())
        {
            unbounded_.push_back(joints[i]);
            uindex_.push_back(vim.find(joints[i]->getName())->second);
        }
}

bool kinematic_constraints::JointConstraintSampler::sample(std::vector<double> &values, unsigned int /* max_attempts */, 
                                                           const planning_models::KinematicState * /* ks */)
{
    values.resize(jmg_->getVariableCount());    
    // enforce the constraints for the constrained components (could be all of them)
    for (std::size_t i = 0 ; i < bounds_.size() ; ++i)
        values[index_[i]] = rng_.uniformReal(bounds_[i].first, bounds_[i].second);
    
    // sample the rest of the components
    for (std::size_t i = 0 ; i < unbounded_.size() ; ++i)
    {
        std::vector<double> v;
        unbounded_[i]->getRandomValues(rng_, v);
        for (std::size_t j = 0 ; j < v.size() ; ++j)
            values[uindex_[i] + j] = v[j];
    }
    
    // we are always successful
    return true;
}

kinematic_constraints::IKConstraintSampler::IKConstraintSampler(const IKAllocator &ik_alloc,
                                                                const planning_models::KinematicModel::JointModelGroup *jmg,
                                                                const PositionConstraint &pc, const OrientationConstraint &oc) :
    ConstraintSampler(jmg), ik_alloc_(ik_alloc), pc_(new PositionConstraint(pc)), oc_(new OrientationConstraint(oc)), ik_timeout_(0.5)
{
    if (pc_->getLinkModel()->getName() != oc_->getLinkModel()->getName())
        ROS_FATAL("Position and orientation constraints need to be specified for the same link in order to use IK-based sampling");
}

        
kinematic_constraints::IKConstraintSampler::IKConstraintSampler(const IKAllocator &ik_alloc,
                                                                const planning_models::KinematicModel::JointModelGroup *jmg,
                                                                const PositionConstraint &pc) :
    ConstraintSampler(jmg), ik_alloc_(ik_alloc), pc_(new PositionConstraint(pc)), ik_timeout_(0.5)
{
}


kinematic_constraints::IKConstraintSampler::IKConstraintSampler(const IKAllocator &ik_alloc,
                                                                const planning_models::KinematicModel::JointModelGroup *jmg,
                                                                const OrientationConstraint &oc) :
    ConstraintSampler(jmg), ik_alloc_(ik_alloc), oc_(new OrientationConstraint(oc)), ik_timeout_(0.5)
{
}

bool kinematic_constraints::IKConstraintSampler::loadIKSolver(void)
{
    if (!ik_alloc_)
    {
        ROS_ERROR("No IK allocator specified");
        return false;
    }
    
    // allocate the solver
    kb_ = ik_alloc_(jmg_);
    if (!kb_)
    {
        ROS_ERROR("Failed to allocate IK solver");
        return false;
    }
    else
        ROS_DEBUG("IKConstraintSampler successfully loaded IK solver");
    
    // the ik solver must cover the same joints as the group
    const std::vector<std::string> &ik_jnames = kb_->getJointNames();
    const std::map<std::string, unsigned int> &g_map = jmg_->getJointVariablesIndexMap();
    
    if (ik_jnames.size() != g_map.size())
    {
        ROS_ERROR_STREAM("Group '" << jmg_->getName() << "' does not have the same set of joints as the employed IK solver");
        return false;
    }
    
    // compute a mapping between the group state and the IK solution
    ik_joint_bijection_.clear();    
    for (std::size_t i = 0 ; i < ik_jnames.size() ; ++i)
    {
        std::map<std::string, unsigned int>::const_iterator it = g_map.find(ik_jnames[i]);
        if (it == g_map.end())
        {
            ROS_ERROR_STREAM("IK solver computes joint values for joint '" << ik_jnames[i] << "' but group '" << jmg_->getName() << "' does not contain such a joint.");
            return false;
        }
        const planning_models::KinematicModel::JointModel *jm = jmg_->getJointModel(ik_jnames[i]);
        for (unsigned int k = 0 ; k < jm->getVariableCount() ; ++k)
            ik_joint_bijection_.push_back(it->second + k);
    }

    // check if we need to transform the request into the coordinate frame expected by IK
    ik_frame_ = kb_->getBaseFrame();
    transform_ik_ = ik_frame_ != jmg_->getParentModel()->getModelFrame();
    if (transform_ik_)
        if (!jmg_->getParentModel()->hasLinkModel(ik_frame_))
        {
            ROS_ERROR_STREAM("The IK solver expects requests in frame '" << ik_frame_ << "' but this frame is not known to the sampler. Ignoring transformation (IK may fail)");
            transform_ik_ = false;
        }

    // check if IK is performed for the desired link
    bool wrong_link = false;
    if (pc_)
    {
        if (kb_->getTipFrame() != pc_->getLinkModel()->getName())
            wrong_link = true;
    }
    else
        if (kb_->getTipFrame() != oc_->getLinkModel()->getName())
            wrong_link = true;
    if (wrong_link)
    {
        ROS_ERROR("IK cannot be performed for link '%s'. The solver can report IK solutions for link '%s'.", 
                  pc_ ? pc_->getLinkModel()->getName().c_str() : oc_->getLinkModel()->getName().c_str(), kb_->getTipFrame().c_str());
        return false;
    }
    
    return true;
}

bool kinematic_constraints::IKConstraintSampler::sample(std::vector<double> &values, unsigned int max_attempts, 
                                                        const planning_models::KinematicState *ks)
{
    // make sure we at least have a chance of sampling using IK; we need at least some kind of constraint
    if (!pc_ && !oc_)
        return false;

    // load an IK solver if we need to 
    if (!kb_)
        if (!loadIKSolver())
        {
            kb_.reset();
            return false;
        }    
    
    for (unsigned int a = 0 ; a < max_attempts ; ++a)
    {
        // sample a point in the constraint region
        btVector3 point;
        if (pc_)
        {
            if (!pc_->getConstraintRegion()->samplePointInside(rng_, max_attempts, point))
                return false;
        }
        else
        {
            // do FK for rand state
            if (!ks)
            {
                ROS_WARN("An initial kinematic state is needed to sample possible link positions");
                return false;
            }
            planning_models::KinematicState tempState(*ks);
            planning_models::KinematicState::JointStateGroup *tmp = tempState.getJointStateGroup(jmg_->getName());
            if (tmp)
            {
                tmp->setRandomValues();
                point = tempState.getLinkState(oc_->getLinkModel()->getName())->getGlobalLinkTransform().getOrigin();
            }
            else
            {
                ROS_ERROR("Passed a JointStateGroup for a mismatching JointModelGroup");
                return false;
            }
        }
        
        btQuaternion quat;
        if (oc_)
        {
            // sample a rotation matrix within the allowed bounds
            double rpy[3];
            rng_.eulerRPY(rpy);
            btMatrix3x3 diff;
            diff.setEulerYPR(rpy[2] * oc_->getYawTolerance() / boost::math::constants::pi<double>(),
                             rpy[1] * oc_->getPitchTolerance() / boost::math::constants::pi<double>(),
                             rpy[0] * oc_->getRollTolerance() / boost::math::constants::pi<double>());
            (oc_->getDesiredRotationMatrix() * diff).getRotation(quat);
        }
        else
        {
            // sample a random orientation
            double q[4];
            rng_.quaternion(q);
            quat = btQuaternion(q[0], q[1], q[2], q[3]);
        }
        
        // if there is an offset, we need to undo the induced rotation in the sampled transform origin (point)
        if (pc_ && pc_->hasLinkOffset())
            // the rotation matrix that corresponds to the desired orientation
            point = point - btMatrix3x3(quat) * pc_->getLinkOffset();

        // we now have the transform we wish to perform IK for, in the planning frame
        
        if (transform_ik_ && ks)
        {
            // we need to convert this transform to the frame expected by the IK solver
            // both the planning frame and the frame for the IK are assumed to be robot links
            btTransform ikq(quat, point);
            
            const planning_models::KinematicState::LinkState *ls = ks->getLinkState(ik_frame_);
            ikq = ls->getGlobalLinkTransform().inverse() * ikq;
            
            point = ikq.getOrigin();
            quat = ikq.getRotation();
        }

        geometry_msgs::Pose ik_query;
        ik_query.position.x = point.x();
        ik_query.position.y = point.y();
        ik_query.position.z = point.z();
        ik_query.orientation.x = quat.x();
        ik_query.orientation.y = quat.y();
        ik_query.orientation.z = quat.z();
        ik_query.orientation.w = quat.w();        
        
        if (callIK(ik_query, ik_timeout_, values))
            return true;
    }
    return false;
}

bool kinematic_constraints::IKConstraintSampler::callIK(const geometry_msgs::Pose &ik_query, double timeout, std::vector<double> &solution)
{
    // sample a seed value
    std::vector<double> vals;
    jmg_->getRandomValues(rng_, vals);
    ROS_ASSERT(vals.size() == ik_joint_bijection_.size());
    std::vector<double> seed(ik_joint_bijection_.size(), 0.0);
    for (std::size_t i = 0 ; i < ik_joint_bijection_.size() ; ++i)
        seed[ik_joint_bijection_[i]] = vals[i];
    
    std::vector<double> ik_sol;
    moveit_msgs::MoveItErrorCodes error;
    
    if (kb_->searchPositionIK(ik_query, seed, timeout, ik_sol, error))
    {
        ROS_ASSERT(ik_sol.size() == ik_joint_bijection_.size());
        solution.resize(ik_joint_bijection_.size());
        for (std::size_t i = 0 ; i < ik_joint_bijection_.size() ; ++i)
            solution[i] = ik_sol[ik_joint_bijection_[i]];
        return true;
    }
    else
        ROS_DEBUG("IK solver failed with error %d", error.val);
    return false;
}
