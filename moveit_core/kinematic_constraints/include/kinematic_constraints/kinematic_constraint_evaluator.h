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

#ifndef KINEMATIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_EVALUATOR_
#define KINEMATIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_EVALUATOR_

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_coordinates/transforms.h>

#include <geometric_shapes/bodies.h>
#include <LinearMath/btTransform.h>
#include <moveit_msgs/Constraints.h>

#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace kinematic_constraints
{
    
    class KinematicConstraintEvaluator
    {
    public:
	
	KinematicConstraintEvaluator(const planning_models::KinematicModel &model, const planning_coordinates::Transforms &tf) : model_(model), tf_(tf)
	{
	}
	
	virtual ~KinematicConstraintEvaluator(void)
	{
	}
	
	/** \brief Clear the stored constraint */
	virtual void clear(void) = 0;
	
	/** \brief Decide whether the constraint is satisfied in the indicated state */
	virtual std::pair<bool, double> decide(const planning_models::KinematicState &state, bool verbose = false) const = 0;
	
	/** \brief Print the constraint data */
	virtual void print(std::ostream &out = std::cout) const
	{
	}
	
    protected:
	
	const planning_models::KinematicModel    &model_;
	mutable planning_coordinates::Transforms  tf_;	
	mutable boost::mutex                      lock_;
    };
    
    class JointConstraintEvaluator : public KinematicConstraintEvaluator
    {
    public:  
	
	JointConstraintEvaluator(const planning_models::KinematicModel &model, const planning_coordinates::Transforms &tf) : KinematicConstraintEvaluator(model, tf), joint_model_(NULL), cont_(false)
	{
	}
	
	/** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	bool use(const moveit_msgs::JointConstraint &jc);

	/** \brief Decide whether the constraint is satisfied in the indicated state */
	virtual std::pair<bool, double> decide(const planning_models::KinematicState &state, bool verbose = false) const;
	
	/** \brief Clear the stored constraint */
	virtual void clear(void);
	
	/** \brief Print the constraint data */
	virtual void print(std::ostream &out = std::cout) const;
	
	/** \brief Get the constraint message */
	const moveit_msgs::JointConstraint& getConstraintMessage(void) const;
	
    protected:
	
	const planning_models::KinematicModel::JointModel *joint_model_;	
	bool                                               cont_;
	moveit_msgs::JointConstraint                       jc_;
    };
    
    
    class OrientationConstraintEvaluator : public KinematicConstraintEvaluator
    {
    public:
	
	OrientationConstraintEvaluator(const planning_models::KinematicModel &model, const planning_coordinates::Transforms &tf) : KinematicConstraintEvaluator(model, tf), link_model_(NULL)
	{
	}
	
	/** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	bool use(const moveit_msgs::OrientationConstraint &pc);
	
	/** \brief Clear the stored constraint */
	virtual void clear(void);
	
	/** \brief Decide whether the constraint is satisfied in the indicated state */
	virtual std::pair<bool, double> decide(const planning_models::KinematicState &state, bool verbose = false) const;
	
	/** \brief Print the constraint data */
	void print(std::ostream &out = std::cout) const;
	
	/** \brief Get the constraint message */
	const moveit_msgs::OrientationConstraint& getConstraintMessage(void) const;
	
    protected:

	const planning_models::KinematicModel::LinkModel *link_model_;		
	moveit_msgs::OrientationConstraint                oc_;	
	btMatrix3x3                                       rotation_matrix_;
	btMatrix3x3                                       rotation_matrix_inv_;
	bool                                              mobileFrame_;
    };
    
    class PositionConstraintEvaluator : public KinematicConstraintEvaluator
    {
    public:
	
	PositionConstraintEvaluator(const planning_models::KinematicModel &model, const planning_coordinates::Transforms &tf) : KinematicConstraintEvaluator(model, tf), link_model_(NULL)
	{
	}
	
	/** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	bool use(const moveit_msgs::PositionConstraint &pc);
	
	/** \brief Clear the stored constraint */
	virtual void clear(void);
	
	/** \brief Decide whether the constraint is satisfied in the indicated state */
	virtual std::pair<bool, double> decide(const planning_models::KinematicState &state, bool verbose = false) const;
	
	/** \brief Print the constraint data */
	void print(std::ostream &out = std::cout) const;
	
	/** \brief Get the constraint message */
	const moveit_msgs::PositionConstraint& getConstraintMessage(void) const;
	
    protected:
	
	moveit_msgs::PositionConstraint                   pc_;
	btVector3                                         offset_;
	boost::shared_ptr<bodies::Body>                   constraint_region_;
	btTransform                                       constraint_region_pose_;
	bool                                              mobileFrame_;	
	const planning_models::KinematicModel::LinkModel *link_model_;		
    };

    /*    
    class VisibilityConstraintEvaluator : public KinematicConstraintEvaluator
    {
    public:
	
	VisibilityConstraintEvaluator(const planning_models::KinematicModel &model) : KinematicConstraintEvaluator(model, tf)
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
	moveit_msgs::VisibilityConstraint                 vc_;
	btTransform                                       sensor_offset_pose_;
    };
    */

    class KinematicConstraintEvaluatorSet
    {
    public:
	
	KinematicConstraintEvaluatorSet(const planning_models::KinematicModel &model, const planning_coordinates::Transforms &tf) : model_(model), tf_(tf)
	{
	}
	
	~KinematicConstraintEvaluatorSet(void)
	{
	    clear();
	}
	
	/** \brief Clear the stored constraints */
	void clear(void);
	
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
	const planning_coordinates::Transforms         &tf_;
	
	std::vector<KinematicConstraintEvaluator*>      kce_;

	std::vector<moveit_msgs::JointConstraint>       jc_;
	std::vector<moveit_msgs::PositionConstraint>    pc_;
	std::vector<moveit_msgs::OrientationConstraint> oc_;
	//	std::vector<moveit_msgs::VisibilityConstraint>  vc_;
    };
} // planning_environment


#endif
