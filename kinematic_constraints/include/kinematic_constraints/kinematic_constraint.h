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

#ifndef KINEMATIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_
#define KINEMATIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>

#include <geometric_shapes/bodies.h>
#include <moveit_msgs/Constraints.h>

#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace kinematic_constraints
{
    
    class KinematicConstraint
    {
    public:
	
	KinematicConstraint(const planning_models::KinematicModel &model, const planning_models::Transforms &tf);
	virtual ~KinematicConstraint(void);
	
	/** \brief Clear the stored constraint */
	virtual void clear(void) = 0;
	
	/** \brief Decide whether the constraint is satisfied in the indicated state */
	virtual std::pair<bool, double> decide(const planning_models::KinematicState &state, bool verbose = false) const = 0;
	
	/** \brief This function returns true if this constraint is
	    configured and able to decide whether states do meet the
	    constraint or not. If this function returns false it means
	    that decide() will always return true -- there is no
	    constraint to be checked. */
	virtual bool enabled(void) const = 0;
	
	/** \brief Print the constraint data */
	virtual void print(std::ostream &out = std::cout) const
	{
	}
	
	/** \brief The weight of a constraint is a multiplicative factor associated to the distance computed by the decide() function. */
	double getConstraintWeight(void) const
	{
	    return constraint_weight_;
	}
	
    protected:
	
	const planning_models::KinematicModel *model_;
	const planning_models::Transforms     *tf_;
	double                                 constraint_weight_;	
    };

    typedef boost::shared_ptr<KinematicConstraint> KinematicConstraintPtr;

    class JointConstraint : public KinematicConstraint
    {
    public:  
	
	JointConstraint(const planning_models::KinematicModel &model, const planning_models::Transforms &tf) : KinematicConstraint(model, tf), joint_model_(NULL)
	{
	}

	bool use(const moveit_msgs::JointConstraint &jc);
	virtual std::pair<bool, double> decide(const planning_models::KinematicState &state, bool verbose = false) const;
	virtual bool enabled(void) const;
	virtual void clear(void);
	virtual void print(std::ostream &out = std::cout) const;
	
	/** \brief Get the joint model this constraint operates on */
	const planning_models::KinematicModel::JointModel* getJointModel(void) const
	{
	    return joint_model_;
	}
	
	double getDesiredJointPosition(void) const
	{
	    return joint_position_;
	}
	
	double getJointToleranceAbove(void) const
	{
	    return joint_tolerance_above_;
	}
	
	double getJointToleranceBelow(void) const
	{
	    return joint_tolerance_below_;
	}
	
    protected:
	
	const planning_models::KinematicModel::JointModel *joint_model_;	
	bool                                               joint_is_continuous_;
	double                                             joint_position_, joint_tolerance_above_, joint_tolerance_below_;
    };
    
    
    class OrientationConstraint : public KinematicConstraint
    {
    public:
	
	OrientationConstraint(const planning_models::KinematicModel &model, const planning_models::Transforms &tf) : KinematicConstraint(model, tf), link_model_(NULL)
	{
	}
	
	bool use(const moveit_msgs::OrientationConstraint &pc);	
	virtual void clear(void);
	virtual std::pair<bool, double> decide(const planning_models::KinematicState &state, bool verbose = false) const;
	virtual bool enabled(void) const;
	void print(std::ostream &out = std::cout) const;
	
	const btMatrix3x3& getDesiredRotationMatrix(void) const
	{
	    return desired_rotation_matrix_;
	}
	
	double getYawTolerance(void) const
	{
	    return absolute_yaw_tolerance_;
	}
	
	double getPitchTolerance(void) const
	{	    
	    return absolute_pitch_tolerance_;
	}

	double getRollTolerance(void) const
	{
	    return absolute_roll_tolerance_;
	}
	
    protected:

	const planning_models::KinematicModel::LinkModel *link_model_;
	btMatrix3x3                                       desired_rotation_matrix_;
	btMatrix3x3                                       desired_rotation_matrix_inv_;
	std::string                                       desired_rotation_frame_id_;
	bool                                              mobile_frame_;
	double                                            absolute_roll_tolerance_, absolute_pitch_tolerance_, absolute_yaw_tolerance_;
    };
    
    class PositionConstraint : public KinematicConstraint
    {
    public:
	
	PositionConstraint(const planning_models::KinematicModel &model, const planning_models::Transforms &tf) : KinematicConstraint(model, tf), link_model_(NULL)
	{
	}
	
	bool use(const moveit_msgs::PositionConstraint &pc);	
	virtual void clear(void);
	virtual std::pair<bool, double> decide(const planning_models::KinematicState &state, bool verbose = false) const;	
	virtual bool enabled(void) const;
	void print(std::ostream &out = std::cout) const;	

	const boost::shared_ptr<bodies::Body>& getConstraintRegion(void) const
	{
	    return constraint_region_;
	}
	
    protected:
	
	btVector3                                         offset_;
	boost::shared_ptr<bodies::Body>                   constraint_region_;
	btTransform                                       constraint_region_pose_;
	bool                                              mobile_frame_;	
	std::string                                       constraint_frame_id_;
	const planning_models::KinematicModel::LinkModel *link_model_;		
    };

    /*    
    class VisibilityConstraint : public KinematicConstraint
    {
    public:
	
	VisibilityConstraint(const planning_models::KinematicModel &model) : KinematicConstraint(model, tf)
	{
	}
	
	/// \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed 
	bool use(const moveit_msgs::VisibilityConstraint &vc);
	
	/// \brief Clear the stored constraint 
	virtual void clear(void);
	
	/// \brief Decide whether the constraint is satisfied in the indicated state
	virtual std::pair<bool, double> decide(const planning_models::KinematicState* state, bool verbose = false) const;
	
	/// \brief Print the constraint data
	void print(std::ostream &out = std::cout) const;
	
	/// \brief Get the constraint message 
	const moveit_msgs::VisibilityConstraint& getConstraintMessage(void) const;
	
    protected:	
	btTransform                                       sensor_offset_pose_;
    };
    */

    class KinematicConstraintSet
    {
    public:
	
	KinematicConstraintSet(const planning_models::KinematicModel &model, const planning_models::Transforms &tf) : model_(model), tf_(tf)
	{
	}
	
	~KinematicConstraintSet(void)
	{
	    clear();
	}
	
	/** \brief Clear the stored constraints */
	void clear(void);

	/** \brief Add all known constraints */
	bool add(const moveit_msgs::Constraints &c);

	/** \brief Add a set of joint constraints */
	bool add(const std::vector<moveit_msgs::JointConstraint> &jc);
	
	/** \brief Add a set of position constraints */
	bool add(const std::vector<moveit_msgs::PositionConstraint> &pc);
	
	/** \brief Add a set of orientation constraints */
	bool add(const std::vector<moveit_msgs::OrientationConstraint> &pc);
	
	/** \brief Add a set of orientation constraints */
	//	bool add(const std::vector<moveit_msgs::VisibilityConstraint> &pc);
	
	/** \brief Decide whether the set of constraints is satisfied  */
	std::pair<bool, double> decide(const planning_models::KinematicState &state, bool verbose = false) const;
	
	/** \brief Print the constraint data */
	void print(std::ostream &out = std::cout) const;
	
	/** \brief Get the active position constraints */
	const std::vector<moveit_msgs::PositionConstraint>& getPositionConstraints(void) const
	{
	    return pc_;
	}
	
	/** \brief Get the active orientation constraints */
	const std::vector<moveit_msgs::OrientationConstraint>& getOrientationConstraints(void) const
	{
	    return oc_;
	}
	
	/** \brief Get the active pose constraints */
	const std::vector<moveit_msgs::JointConstraint>& getJointConstraints(void) const
	{
	    return jc_;
	}

	/*
	/// \brief Get the active visibility constraints 
	const std::vector<moveit_msgs::VisibilityConstraint>& getVisibilityConstraints(void) const
	{
	    return vc_;
	}
	*/
    protected:

	const planning_models::KinematicModel          &model_;
	const planning_models::Transforms              &tf_;
	
	std::vector<KinematicConstraintPtr>             kce_;

	std::vector<moveit_msgs::JointConstraint>       jc_;
	std::vector<moveit_msgs::PositionConstraint>    pc_;
	std::vector<moveit_msgs::OrientationConstraint> oc_;
	//	std::vector<moveit_msgs::VisibilityConstraint>  vc_;
    };    
    
    typedef boost::shared_ptr<KinematicConstraintSet> KinematicConstraintSetPtr;
    
}


#endif
