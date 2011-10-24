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
#include <limits>

bool kinematic_constraints::JointConstraint::use(const moveit_msgs::JointConstraint &jc)
{
    jc_ = jc;
    joint_model_ = model_.getJointModel(jc_.joint_name);
    cont_ = false;
    if (joint_model_)
    {
	// check if we have to wrap angles when computing distances
	const planning_models::KinematicModel::RevoluteJointModel *revolute_joint = dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(joint_model_);
	if (revolute_joint && revolute_joint->isContinuous())
	    cont_ = true;

	// check if the joint has 1 DOF (the only kind we can handle)
	if (joint_model_->getVariableCount() == 0)
	{
	    ROS_ERROR_STREAM("Joint " << jc_.joint_name << " has no parameters to constrain");
	    joint_model_ = NULL;
	}
	else
	    if (joint_model_->getVariableCount() > 1)
	    {
		ROS_ERROR_STREAM("Joint " << jc_.joint_name << " has more than one parameter to constrain. This type of constraint is not appropriate.");
		joint_model_ = NULL;
	    }
    }    
    return joint_model_ != NULL;
}

std::pair<bool, double> kinematic_constraints::JointConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
    if (!joint_model_)
	return std::make_pair(true, 0.0);
    
    const planning_models::KinematicState::JointState *joint = state.getJointState(jc_.joint_name);
    
    if (!joint)
    {
	ROS_WARN_STREAM("No joint in state with name " << jc_.joint_name);
	return std::make_pair(false, 0.0);
    }
    
    double current_joint_position = joint->getJointStateValues()[0];
    double dif = 0.0;
    
    // compute signed shortest distance for continuous joints
    if (cont_)
    {
	dif = btNormalizeAngle(current_joint_position) - btNormalizeAngle(jc_.position);
	if (dif > SIMD_PI)
	    dif = SIMD_2_PI - dif; 
	else
	    if (dif < -SIMD_PI)
		dif = -dif - SIMD_2_PI;
    }
    else
	dif = current_joint_position - jc_.position;
    
    // check bounds
    bool result = dif <= jc_.tolerance_above && dif >= -jc_.tolerance_below;
    if (verbose)
	ROS_INFO("Constraint %s:: Joint name: %s, actual value: %f, desired value: %f, tolerance_above: %f, tolerance_below: %f",
		 result ? "satisfied" : "violated", joint->getName().c_str(), current_joint_position, jc_.position, jc_.tolerance_above, jc_.tolerance_below);
    
    return std::make_pair(result, jc_.weight * fabs(dif));
}

void kinematic_constraints::JointConstraint::clear(void)
{
    joint_model_ = NULL;
}

const moveit_msgs::JointConstraint& kinematic_constraints::JointConstraint::getConstraintMessage(void) const
{
    return jc_;    
}

void kinematic_constraints::JointConstraint::print(std::ostream &out) const
{		
    if (joint_model_)
    {
	out << "Joint constraint for joint " << jc_.joint_name << ": " << std::endl;
	out << "  value = ";	    
	out << jc_.position << "; ";
	out << "  tolerance below = ";	    
	out << jc_.tolerance_below << "; ";	
	out << "  tolerance above = ";
	out << jc_.tolerance_above << "; ";
	out << std::endl;
    }
    else
	out << "No constraint" << std::endl;
}

bool kinematic_constraints::PositionConstraint::use(const moveit_msgs::PositionConstraint &pc)
{
    pc_ = pc;
    link_model_ = model_.getLinkModel(pc_.link_name);
    offset_ = btVector3(pc_.target_point_offset.x, pc_.target_point_offset.y, pc_.target_point_offset.z);
    constraint_region_.reset(bodies::createBodyFromShape(shapes::constructShapeFromMsg(pc_.constraint_region_shape)));

    if (link_model_ && constraint_region_)
    {
	const geometry_msgs::Pose &msg = pc_.constraint_region_pose.pose;
	btQuaternion qr;
	if (!planning_models::quatFromMsg(msg.orientation, qr))
	    ROS_WARN("Incorrect specification of orientation in pose for link '%s'. Assuming identity quaternion.", pc_.link_name.c_str());	
	constraint_region_pose_ = btTransform(qr, btVector3(msg.position.x, msg.position.y, msg.position.z));
	
	if (tf_.isFixedFrame(pc_.constraint_region_pose.header.frame_id))
	{
	    tf_.transformTransform(constraint_region_pose_, constraint_region_pose_, pc_.constraint_region_pose.header.frame_id);
	    constraint_region_->setPose(constraint_region_pose_);
	    mobileFrame_ = false;
	}
	else
       	    mobileFrame_ = true;
	return true;
    }
    else
	return false;
}

namespace kinematic_constraints
{
    // helper function to avoid code duplication
    static inline std::pair<bool, double> finishPositionConstraintDecision(const btVector3 &pt, const btVector3 &desired,
									   const moveit_msgs::PositionConstraint &pc, bool result, bool verbose)
    {
	if (verbose)
	    ROS_INFO("Position constraint %s on link '%s'. Desired: %f, %f, %f, current: %f, %f, %f",
		     result ? "satisfied" : "violated", pc.link_name.c_str(),
		     desired.x(), desired.y(), desired.z(), pt.x(), pt.y(), pt.z());
	double dx = desired.x() - pt.x();
	double dy = desired.y() - pt.y();
	double dz = desired.z() - pt.z();
	return std::make_pair(result, pc.weight * sqrt(dx * dx + dy * dy + dz * dz));
    }
}

std::pair<bool, double> kinematic_constraints::PositionConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
    if (!link_model_ || !constraint_region_)
	return std::make_pair(true, 0.0);
    
    const planning_models::KinematicState::LinkState *link_state = state.getLinkState(pc_.link_name);
    
    if (!link_state) 
    {
	ROS_WARN_STREAM("No link in state with name " << pc_.link_name);
	return std::make_pair(false, 0.0);
    }
    
    const btVector3 &pt = link_state->getGlobalLinkTransform()(offset_);

    if (mobileFrame_)
    {	
	// in this case we need to lock as we are modifying the internal state of the class (the mutable members)
	btTransform tmp;
	lock_.lock();
	tf_.setKinematicState(state);
	tf_.transformTransform(tmp, constraint_region_pose_, pc_.constraint_region_pose.header.frame_id);
	constraint_region_->setPose(tmp);
	bool result = constraint_region_->containsPoint(pt, false);  
	lock_.unlock();
	return finishPositionConstraintDecision(pt, tmp.getOrigin(), pc_, result, verbose);
    }
    else
    {
	bool result = constraint_region_->containsPoint(pt, false);  
	return finishPositionConstraintDecision(pt, constraint_region_->getPose().getOrigin(), pc_, result, verbose);
    }
}

void kinematic_constraints::PositionConstraint::print(std::ostream &out) const
{
    if (link_model_ && constraint_region_)
    {
	out << "Position constraint on link '" << pc_.link_name << "'" << std::endl;
	if (pc_.constraint_region_shape.type == moveit_msgs::Shape::SPHERE)
	{
	    if (pc_.constraint_region_shape.dimensions.empty())
		out << "No radius specified for spherical constraint region.";
	    else
		out << "Spherical constraint region with radius " << pc_.constraint_region_shape.dimensions[0] << std::endl;
	}
	else if (pc_.constraint_region_shape.type == moveit_msgs::Shape::BOX)
	{
	    if (pc_.constraint_region_shape.dimensions.size() < 3)
		out << "Length, width, height must be specified for box constraint region.";
	    else
		out << "Box constraint region with dimensions " << pc_.constraint_region_shape.dimensions[0] << " x "  
		    << pc_.constraint_region_shape.dimensions[1] << " x "  <<  pc_.constraint_region_shape.dimensions[2] << std::endl;
	}
	else if (pc_.constraint_region_shape.type == moveit_msgs::Shape::CYLINDER)
	{
	    if(pc_.constraint_region_shape.dimensions.size() < 2)
		out << "Radius and height must be specified for cylinder constraint region.";
	    else
		out << "Cylinder constraint region with radius " << pc_.constraint_region_shape.dimensions[0] << " and length "  
		    << pc_.constraint_region_shape.dimensions[1] << std::endl;
	}
	else if (pc_.constraint_region_shape.type == moveit_msgs::Shape::MESH)
	{
	    out << "Mesh type constraint region.";
	}
    }
    else
	out << "No constraint" << std::endl;
}

const moveit_msgs::PositionConstraint& kinematic_constraints::PositionConstraint::getConstraintMessage(void) const
{
    return pc_;
}

void kinematic_constraints::PositionConstraint::clear(void)
{
    link_model_ = NULL;
    constraint_region_.reset();
}

bool kinematic_constraints::OrientationConstraint::use(const moveit_msgs::OrientationConstraint &oc)
{
    oc_ = oc;
    link_model_ = model_.getLinkModel(oc_.link_name);
    btQuaternion q;
    if (!planning_models::quatFromMsg(oc_.orientation.quaternion, q))
	ROS_WARN("Orientation constraint is probably incorrect: %f, %f, %f, %f. Assuming identity instead.", 
		 oc_.orientation.quaternion.x, oc_.orientation.quaternion.y, oc_.orientation.quaternion.z, oc_.orientation.quaternion.w);
    
    if (tf_.isFixedFrame(oc_.orientation.header.frame_id))
    {
	tf_.transformQuaternion(q, q, oc_.orientation.header.frame_id);
	rotation_matrix_ = btMatrix3x3(q);
	rotation_matrix_inv_ = rotation_matrix_.inverse();
	mobileFrame_ = false;
    }
    else
    {
	rotation_matrix_ = btMatrix3x3(q);
	mobileFrame_ = true;
    }
    
    return link_model_ != NULL;
}

void kinematic_constraints::OrientationConstraint::clear(void)
{
    link_model_ = NULL;
}

std::pair<bool, double> kinematic_constraints::OrientationConstraint::decide(const planning_models::KinematicState &state, bool verbose) const
{
    if (!link_model_)
	return std::make_pair(true, 0.0);
    
    const planning_models::KinematicState::LinkState *link_state = state.getLinkState(oc_.link_name);
    
    if (!link_state) 
    {
	ROS_WARN_STREAM("No link in state with name " << oc_.link_name);
	return std::make_pair(false, 0.0);
    }
    
    btScalar yaw, pitch, roll;
    if (mobileFrame_)
    {
	btMatrix3x3 tmp;
	lock_.lock();
	tf_.setKinematicState(state);
	tf_.transformMatrix(tmp, rotation_matrix_, oc_.orientation.header.frame_id);
	lock_.unlock();
	btMatrix3x3 diff = tmp.inverse() * link_state->getGlobalLinkTransform().getBasis();
	diff.getEulerYPR(yaw, pitch, roll);	
    }
    else
    {
	btMatrix3x3 diff = rotation_matrix_inv_ * link_state->getGlobalLinkTransform().getBasis();
	diff.getEulerYPR(yaw, pitch, roll);
    }
    
    bool result = fabs(roll) < oc_.absolute_roll_tolerance && fabs(pitch) < oc_.absolute_pitch_tolerance && fabs(yaw) < oc_.absolute_yaw_tolerance;
    
    if (verbose)
    {
	btQuaternion quat;
	link_state->getGlobalLinkTransform().getBasis().getRotation(quat);
	ROS_INFO("Orientation constraint %s for link '%s'. Quaternion desired: %f %f %f %f, quaternion actual: %f %f %f %f, error: roll=%f, pitch=%f, yaw=%f, tolerance: roll=%f, pitch=%f, yaw=%f",
		 result ? "satisfied" : "violated", oc_.link_name.c_str(),
		 oc_.orientation.quaternion.x, oc_.orientation.quaternion.y, oc_.orientation.quaternion.z, oc_.orientation.quaternion.w,
		 quat.getX(), quat.getY(), quat.getZ(), quat.getW(), roll, pitch, yaw, 
		 oc_.absolute_roll_tolerance, oc_.absolute_pitch_tolerance, oc_.absolute_yaw_tolerance);
    }
    
    return std::make_pair(result, oc_.weight * (fabs(roll) + fabs(pitch) + fabs(yaw)));
}

const moveit_msgs::OrientationConstraint& kinematic_constraints::OrientationConstraint::getConstraintMessage(void) const
{
    return oc_;
}

void kinematic_constraints::OrientationConstraint::print(std::ostream &out) const
{
    if (link_model_)
    {
	out << "Orientation constraint on link '" << oc_.link_name << "'" << std::endl;
	out << "Desired orientation:" << oc_.orientation.quaternion.x << "," <<  oc_.orientation.quaternion.y << "," 
	    <<  oc_.orientation.quaternion.z << "," << oc_.orientation.quaternion.w << std::endl;
    }
    else
	out << "No constraint" << std::endl;
}

/*
const moveit_msgs::VisibilityConstraint& kinematic_constraints::VisibilityConstraint::getConstraintMessage(void) const
{
    return vc_;
}

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
	ROS_WARN_STREAM("No link state for link " << vc_.target_link);
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

