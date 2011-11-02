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

#include <kinematic_constraints/kinematic_constraint.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <planning_models/conversions.h>
#include <boost/scoped_ptr.hpp>
#include <limits>

kinematic_constraints::KinematicConstraint::KinematicConstraint(const planning_models::KinematicModel &model, const planning_models::Transforms &tf) :
    model_(&model), tf_(&tf), constraint_weight_(std::numeric_limits<double>::epsilon())
{
}

kinematic_constraints::KinematicConstraint::~KinematicConstraint(void)
{
}

bool kinematic_constraints::JointConstraint::use(const moveit_msgs::JointConstraint &jc)
{
    joint_model_ = model_->getJointModel(jc.joint_name);
    joint_is_continuous_ = false;
    if (joint_model_)
    {
        // check if we have to wrap angles when computing distances
        const planning_models::KinematicModel::RevoluteJointModel *revolute_joint = dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(joint_model_);
        if (revolute_joint && revolute_joint->isContinuous())
        {
            joint_is_continuous_ = true;
            joint_position_ = btNormalizeAngle(jc.position);
        }
        else
            joint_position_ = jc.position;

        // check if the joint has 1 DOF (the only kind we can handle)
        if (joint_model_->getVariableCount() == 0)
        {
            ROS_ERROR_STREAM("Joint '" << jc.joint_name << "' has no parameters to constrain");
            joint_model_ = NULL;
        }
        else
            if (joint_model_->getVariableCount() > 1)
            {
                ROS_ERROR_STREAM("Joint '" << jc.joint_name << "' has more than one parameter to constrain. This type of constraint is not supported.");
                joint_model_ = NULL;
            }
        joint_tolerance_above_ = jc.tolerance_above;
        joint_tolerance_below_ = jc.tolerance_below;

        if (jc.weight <= std::numeric_limits<double>::epsilon())
            ROS_WARN_STREAM("The weight on constraint for joint '" << jc.joint_name << "' should be positive");
        else
            constraint_weight_ = jc.weight;
    }
    return joint_model_ != NULL;
}

std::pair<bool, double> kinematic_constraints::JointConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
    if (!joint_model_)
        return std::make_pair(true, 0.0);

    const planning_models::KinematicState::JointState *joint = state.getJointState(joint_model_->getName());

    if (!joint)
    {
        ROS_WARN_STREAM("No joint in state with name '" << joint_model_->getName() << "'");
        return std::make_pair(false, 0.0);
    }

    double current_joint_position = joint->getVariableValues()[0];
    double dif = 0.0;

    // compute signed shortest distance for continuous joints
    if (joint_is_continuous_)
    {
        dif = btNormalizeAngle(current_joint_position) - joint_position_;

        if (dif > SIMD_PI)
            dif = SIMD_2_PI - dif;
        else
            if (dif < -SIMD_PI)
                dif += SIMD_2_PI; // we include a sign change to have dif > 0
        // however, we want to include proper sign for diff, as the tol below is may be different from tol above
        if (current_joint_position < joint_position_)
            dif = -dif;
    }
    else
        dif = current_joint_position - joint_position_;

    // check bounds
    bool result = dif <= joint_tolerance_above_ && dif >= -joint_tolerance_below_;
    if (verbose)
        ROS_INFO("Constraint %s:: Joint name: '%s', actual value: %f, desired value: %f, tolerance_above: %f, tolerance_below: %f",
                 result ? "satisfied" : "violated", joint->getName().c_str(), current_joint_position, joint_position_, joint_tolerance_above_, joint_tolerance_below_);

    return std::make_pair(result, constraint_weight_ * fabs(dif));
}

bool kinematic_constraints::JointConstraint::enabled(void) const
{
    return joint_model_;
}

void kinematic_constraints::JointConstraint::clear(void)
{
    joint_model_ = NULL;
}

void kinematic_constraints::JointConstraint::print(std::ostream &out) const
{
    if (joint_model_)
    {
        out << "Joint constraint for joint " << joint_model_->getName() << ": " << std::endl;
        out << "  value = ";
        out << joint_position_ << "; ";
        out << "  tolerance below = ";
        out << joint_tolerance_below_ << "; ";
        out << "  tolerance above = ";
        out << joint_tolerance_above_ << "; ";
        out << std::endl;
    }
    else
        out << "No constraint" << std::endl;
}

bool kinematic_constraints::PositionConstraint::use(const moveit_msgs::PositionConstraint &pc)
{
    link_model_ = model_->getLinkModel(pc.link_name);
    offset_ = btVector3(pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z);
    has_offset_ = offset_.length2() > std::numeric_limits<double>::epsilon();
    boost::scoped_ptr<shapes::Shape> shape(shapes::constructShapeFromMsg(pc.constraint_region_shape));
    if (shape)
        constraint_region_.reset(bodies::createBodyFromShape(shape.get()));

    if (link_model_ && constraint_region_)
    {
        const geometry_msgs::Pose &msg = pc.constraint_region_pose.pose;
        btQuaternion qr;
        if (!planning_models::quatFromMsg(msg.orientation, qr))
            ROS_WARN("Incorrect specification of orientation in pose for link '%s'. Assuming identity quaternion.", pc.link_name.c_str());
        constraint_region_pose_ = btTransform(qr, btVector3(msg.position.x, msg.position.y, msg.position.z));

        if (tf_->isFixedFrame(pc.constraint_region_pose.header.frame_id))
        {
            tf_->transformTransform(constraint_region_pose_, constraint_region_pose_, pc.constraint_region_pose.header.frame_id);
            constraint_frame_id_ = tf_->getPlanningFrame();
            constraint_region_->setPose(constraint_region_pose_);
            mobile_frame_ = false;
        }
        else
        {
            constraint_frame_id_ = pc.constraint_region_pose.header.frame_id;
            mobile_frame_ = true;
        }


        if (pc.weight <= std::numeric_limits<double>::epsilon())
            ROS_WARN_STREAM("The weight on position constraint for link '" << pc.link_name << "' should be positive");
        else
            constraint_weight_ = pc.weight;

        return true;
    }
    else
        return false;
}

namespace kinematic_constraints
{
    // helper function to avoid code duplication
    static inline std::pair<bool, double> finishPositionConstraintDecision(const btVector3 &pt, const btVector3 &desired, const std::string &name,
                                                                           double weight, bool result, bool verbose)
    {
        if (verbose)
            ROS_INFO("Position constraint %s on link '%s'. Desired: %f, %f, %f, current: %f, %f, %f",
                     result ? "satisfied" : "violated", name.c_str(), desired.x(), desired.y(), desired.z(), pt.x(), pt.y(), pt.z());
        double dx = desired.x() - pt.x();
        double dy = desired.y() - pt.y();
        double dz = desired.z() - pt.z();
        return std::make_pair(result, weight * sqrt(dx * dx + dy * dy + dz * dz));
    }
}

std::pair<bool, double> kinematic_constraints::PositionConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
    if (!link_model_ || !constraint_region_)
        return std::make_pair(true, 0.0);

    const planning_models::KinematicState::LinkState *link_state = state.getLinkState(link_model_->getName());

    if (!link_state)
    {
        ROS_WARN_STREAM("No link in state with name '" << link_model_->getName() << "'");
        return std::make_pair(false, 0.0);
    }

    const btVector3 &pt = link_state->getGlobalLinkTransform()(offset_);

    if (mobile_frame_)
    {
        btTransform tmp;
        tf_->transformTransform(state, tmp, constraint_region_pose_, constraint_frame_id_);
        bool result = constraint_region_->cloneAt(tmp)->containsPoint(pt);
        return finishPositionConstraintDecision(pt, tmp.getOrigin(), link_model_->getName(), constraint_weight_, result, verbose);
    }
    else
    {
        bool result = constraint_region_->containsPoint(pt);
        return finishPositionConstraintDecision(pt, constraint_region_->getPose().getOrigin(), link_model_->getName(), constraint_weight_, result, verbose);
    }
}

void kinematic_constraints::PositionConstraint::print(std::ostream &out) const
{
    if (link_model_ && constraint_region_)
    {
        out << "Position constraint on link '" << link_model_->getName() << "'" << std::endl;
        if (constraint_region_->getType() == shapes::SPHERE)
            out << "Spherical constraint region of radius " << constraint_region_->getDimensions()[0] << std::endl;
        else if (constraint_region_->getType() == shapes::BOX)
        {
            const std::vector<double> &s = constraint_region_->getDimensions();
            out << "Box constraint region with dimensions " << s[0] << " x " << s[1] << " x "  <<  s[2] << std::endl;
        }
        else if (constraint_region_->getType() == shapes::CYLINDER)
        {
            const std::vector<double> &s = constraint_region_->getDimensions();
            out << "Cylinder constraint region with radius " << s[0] << " and length "  << s[1] << std::endl;
        }
        else if (constraint_region_->getType() == shapes::MESH)
            out << "Mesh type constraint region." << std::endl;
    }
    else
        out << "No constraint" << std::endl;
}

void kinematic_constraints::PositionConstraint::clear(void)
{
    link_model_ = NULL;
    constraint_region_.reset();
}

bool kinematic_constraints::PositionConstraint::enabled(void) const
{
    return link_model_ && constraint_region_;
}

bool kinematic_constraints::OrientationConstraint::use(const moveit_msgs::OrientationConstraint &oc)
{
    link_model_ = model_->getLinkModel(oc.link_name);
    btQuaternion q;
    if (!planning_models::quatFromMsg(oc.orientation.quaternion, q))
        ROS_WARN("Orientation constraint for link '%s' is probably incorrect: %f, %f, %f, %f. Assuming identity instead.", oc.link_name.c_str(),
                 oc.orientation.quaternion.x, oc.orientation.quaternion.y, oc.orientation.quaternion.z, oc.orientation.quaternion.w);

    if (tf_->isFixedFrame(oc.orientation.header.frame_id))
    {
        tf_->transformQuaternion(q, q, oc.orientation.header.frame_id);
        desired_rotation_frame_id_ = tf_->getPlanningFrame();
        desired_rotation_matrix_ = btMatrix3x3(q);
        desired_rotation_matrix_inv_ = desired_rotation_matrix_.inverse();
        mobile_frame_ = false;
    }
    else
    {
        desired_rotation_frame_id_ = oc.orientation.header.frame_id;
        desired_rotation_matrix_ = btMatrix3x3(q);
        mobile_frame_ = true;
    }

    if (oc.weight <= std::numeric_limits<double>::epsilon())
        ROS_WARN_STREAM("The weight on orientation constraint for link '" << oc.link_name << "' should be positive");
    else
        constraint_weight_ = oc.weight;
    absolute_yaw_tolerance_ = fabs(oc.absolute_yaw_tolerance);
    absolute_pitch_tolerance_ = fabs(oc.absolute_pitch_tolerance);
    absolute_roll_tolerance_ = fabs(oc.absolute_roll_tolerance);

    return link_model_ != NULL;
}

void kinematic_constraints::OrientationConstraint::clear(void)
{
    link_model_ = NULL;
}

bool kinematic_constraints::OrientationConstraint::enabled(void) const
{
    return link_model_;
}

std::pair<bool, double> kinematic_constraints::OrientationConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
    if (!link_model_)
        return std::make_pair(true, 0.0);

    const planning_models::KinematicState::LinkState *link_state = state.getLinkState(link_model_->getName());

    if (!link_state)
    {
        ROS_WARN_STREAM("No link in state with name '" << link_model_->getName() << "'");
        return std::make_pair(false, 0.0);
    }

    btScalar yaw, pitch, roll;
    if (mobile_frame_)
    {
        btMatrix3x3 tmp;
        tf_->transformMatrix(state, tmp, desired_rotation_matrix_, desired_rotation_frame_id_);
        btMatrix3x3 diff = tmp.inverse() * link_state->getGlobalLinkTransform().getBasis();
        diff.getEulerYPR(yaw, pitch, roll);
    }
    else
    {
        btMatrix3x3 diff = desired_rotation_matrix_inv_ * link_state->getGlobalLinkTransform().getBasis();
        diff.getEulerYPR(yaw, pitch, roll);
    }
    
    bool result = fabs(roll) < absolute_roll_tolerance_ && fabs(pitch) < absolute_pitch_tolerance_ && fabs(yaw) < absolute_yaw_tolerance_;

    if (verbose)
    {
        btQuaternion q_act, q_des;
        link_state->getGlobalLinkTransform().getBasis().getRotation(q_act);
        desired_rotation_matrix_.getRotation(q_des);
        ROS_INFO("Orientation constraint %s for link '%s'. Quaternion desired: %f %f %f %f, quaternion actual: %f %f %f %f, error: roll=%f, pitch=%f, yaw=%f, tolerance: roll=%f, pitch=%f, yaw=%f",
                 result ? "satisfied" : "violated", link_model_->getName().c_str(),
                 q_des.getX(), q_des.getY(), q_des.getZ(), q_des.getW(),
                 q_act.getX(), q_act.getY(), q_act.getZ(), q_act.getW(), roll, pitch, yaw,
                 absolute_roll_tolerance_, absolute_pitch_tolerance_, absolute_yaw_tolerance_);
    }

    return std::make_pair(result, constraint_weight_ * (fabs(roll) + fabs(pitch) + fabs(yaw)));
}

void kinematic_constraints::OrientationConstraint::print(std::ostream &out) const
{
    if (link_model_)
    {
        out << "Orientation constraint on link '" << link_model_->getName() << "'" << std::endl;
        btQuaternion q_des;
        desired_rotation_matrix_.getRotation(q_des);
        out << "Desired orientation:" << q_des.getX() << "," <<  q_des.getY() << ","  <<  q_des.getZ() << "," << q_des.getW() << std::endl;
    }
    else
        out << "No constraint" << std::endl;
}

/*

void kinematic_constraints::VisibilityConstraint::clear(void)
{
}

bool kinematic_constraints::VisibilityConstraint::use(const moveit_msgs::VisibilityConstraint &vc)
{
    vc_ = vc;
    tf::poseMsgToTF(m_vc.sensor_pose.pose,m_sensor_offset_pose);
    return true;
}

bool kinematic_constraints::VisibilityConstraint::decide(const planning_models::KinematicState* state, bool verbose) const
{
    const planning_models::KinematicState::LinkState* link_state = state->getLinkState(vc_.target_link);

    if (!link_state)
    {
        ROS_WARN_STREAM("No link state for link '" << vc_.target_link << "'");
        return false;
    }

    btTransform sensor_pose = link_state->getGlobalLinkTransform() * sensor_offset_pose;
    double dx = m_vc.target.point.x - sensor_pose.getOrigin().x();
    double dy = m_vc.target.point.y - sensor_pose.getOrigin().y();
    double dz = m_vc.target.point.z - sensor_pose.getOrigin().z();

    btVector3 x_axis(1,0,0);
    btVector3 target_vector(dx,dy,dz);
    btVector3 sensor_x_axis = sensor_pose.getBasis()*x_axis;

    double angle = fabs(target_vector.angle(sensor_x_axis));
    if(angle < m_vc.absolute_tolerance)
        return true;
    else
        return false;
}

void kinematic_constraints::VisibilityConstraint::print(std::ostream &out) const
{
    out << "Visibility constraint for sensor on link '" << vc_.sensor_pose.header.frame_id << "'" << std::endl;
}


*/
void kinematic_constraints::KinematicConstraintSet::clear(void)
{
    kce_.clear();
    jc_.clear();
    pc_.clear();
    oc_.clear();
    //    vc_.clear();
}

bool kinematic_constraints::KinematicConstraintSet::add(const std::vector<moveit_msgs::JointConstraint> &jc)
{
    bool result = true;
    for (unsigned int i = 0 ; i < jc.size() ; ++i)
    {
        JointConstraint *ev = new JointConstraint(model_, tf_);
        bool u = ev->use(jc[i]);
        result = result && u;
        kce_.push_back(KinematicConstraintPtr(ev));
        jc_.push_back(jc[i]);
    }
    return result;
}

bool kinematic_constraints::KinematicConstraintSet::add(const std::vector<moveit_msgs::PositionConstraint> &pc)
{
    bool result = true;
    for (unsigned int i = 0 ; i < pc.size() ; ++i)
    {
        PositionConstraint *ev = new PositionConstraint(model_, tf_);
        bool u = ev->use(pc[i]);
        result = result && u;
        kce_.push_back(KinematicConstraintPtr(ev));
        pc_.push_back(pc[i]);
    }
    return result;
}

bool kinematic_constraints::KinematicConstraintSet::add(const std::vector<moveit_msgs::OrientationConstraint> &oc)
{
    bool result = true;
    for (unsigned int i = 0 ; i < oc.size() ; ++i)
    {
        OrientationConstraint *ev = new OrientationConstraint(model_, tf_);
        bool u = ev->use(oc[i]);
        result = result && u;
        kce_.push_back(KinematicConstraintPtr(ev));
        oc_.push_back(oc[i]);
    }
    return result;
}

bool kinematic_constraints::KinematicConstraintSet::add(const moveit_msgs::Constraints &c)
{
    bool j = add(c.joint_constraints);
    bool p = add(c.position_constraints);
    bool o = add(c.orientation_constraints);
    return j && p && o;
}

/*
bool kinematic_constraints::KinematicConstraintSet::add(const std::vector<moveit_msgs::VisibilityConstraint> &vc)
{
    bool result = true;
    for (unsigned int i = 0 ; i < vc.size() ; ++i)
    {
        VisibilityConstraint *ev = new VisibilityConstraint(model_);
        bool u = ev->use(vc[i]);
        result = result && u;
        kce_.push_back(ev);
        vc_.push_back(vc[i]);
    }
    return result;
}
*/

std::pair<bool, double> kinematic_constraints::KinematicConstraintSet::decide(const planning_models::KinematicState &state, bool verbose) const
{
    bool result = true;
    double d = 0.0;
    for (unsigned int i = 0 ; i < kce_.size() ; ++i)
    {
        const std::pair<bool, double> &r = kce_[i]->decide(state, verbose);
        if (!r.first)
            result = false;
        d += r.second;
    }

    return std::make_pair(result, d);
}

void kinematic_constraints::KinematicConstraintSet::print(std::ostream &out) const
{
    out << kce_.size() << " kinematic constraints" << std::endl;
    for (unsigned int i = 0 ; i < kce_.size() ; ++i)
        kce_[i]->print(out);
}

moveit_msgs::Constraints kinematic_constraints::mergeConstraints(const moveit_msgs::Constraints &first, const moveit_msgs::Constraints &second)
{
    moveit_msgs::Constraints r = first;

    // merge joint constraints
    for (std::size_t i = 0 ; i < second.joint_constraints.size() ; ++i)
    {
        bool keep = true;
        for (std::size_t j = 0 ; j < first.joint_constraints.size() ; ++j)
            if (second.joint_constraints[i].joint_name == first.joint_constraints[j].joint_name)
            {
                keep = false;
                break;
            }
        if (keep)
            r.joint_constraints.push_back(second.joint_constraints[i]);
    }

    // merge rest of constraints
    for (std::size_t i = 0 ; i < second.position_constraints.size() ; ++i)
        r.position_constraints.push_back(second.position_constraints[i]);

    for (std::size_t i = 0 ; i < second.orientation_constraints.size() ; ++i)
        r.orientation_constraints.push_back(second.orientation_constraints[i]);

    for (std::size_t i = 0 ; i < second.visibility_constraints.size() ; ++i)
        r.visibility_constraints.push_back(second.visibility_constraints[i]);

    return r;
}
