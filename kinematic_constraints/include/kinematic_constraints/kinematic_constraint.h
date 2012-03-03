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

#ifndef KINEMATIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_
#define KINEMATIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <collision_detection/collision_world.h>

#include <geometric_shapes/bodies.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/ConstraintEvalResults.h>

#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace kinematic_constraints
{

class KinematicConstraint
{
public:
  
  enum ConstraintType
    {
      UNKNOWN_CONSTRAINT, JOINT_CONSTRAINT, POSITION_CONSTRAINT, ORIENTATION_CONSTRAINT, VISIBILITY_CONSTRAINT
    };
  
  KinematicConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf);
  virtual ~KinematicConstraint(void);
  
  /** \brief Clear the stored constraint */
  virtual void clear(void) = 0;
  
  /** \brief Decide whether the constraint is satisfied in the indicated state */
  virtual bool decide(const planning_models::KinematicState &state, double &distance, bool verbose = false) const = 0;
  
  /** \brief This function returns true if this constraint is
      configured and able to decide whether states do meet the
      constraint or not. If this function returns false it means
      that decide() will always return true -- there is no
      constraint to be checked. */
  virtual bool enabled(void) const = 0;
  
  /** \brief Check if two constraints are the same */
  virtual bool equal(const KinematicConstraint &other, double margin) const = 0;
  
  /** \brief Get the type of constraint */
  ConstraintType getType(void) const
  {
    return type_;
  }
  
  /** \brief Print the constraint data */
  virtual void print(std::ostream &out = std::cout) const
  {
  }
  
  /** \brief The weight of a constraint is a multiplicative factor associated to the distance computed by the decide() function. */
  double getConstraintWeight(void) const
  {
    return constraint_weight_;
  }
  
  const planning_models::KinematicModelConstPtr& getKinematicModel(void) const
  {
    return model_;
  }
  
  const planning_models::TransformsConstPtr& getTransforms(void) const
  {
    return tf_;
  }
  
protected:
  
  ConstraintType                          type_;
  planning_models::KinematicModelConstPtr model_;
  planning_models::TransformsConstPtr     tf_;
  double                                  constraint_weight_;
};

typedef boost::shared_ptr<KinematicConstraint> KinematicConstraintPtr;
typedef boost::shared_ptr<const KinematicConstraint> KinematicConstraintConstPtr;

class JointConstraint : public KinematicConstraint
{
public:
  
  JointConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
    KinematicConstraint(model, tf), joint_model_(NULL)
  {
    type_ = JOINT_CONSTRAINT;
  }
  
  bool configure(const moveit_msgs::JointConstraint &jc);
  virtual bool equal(const KinematicConstraint &other, double margin) const;
  virtual bool decide(const planning_models::KinematicState &state, double &distance, bool verbose = false) const;
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OrientationConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
    KinematicConstraint(model, tf), link_model_(NULL)
  {
    type_ = ORIENTATION_CONSTRAINT;
  }
  
  bool configure(const moveit_msgs::OrientationConstraint &pc);
  virtual bool equal(const KinematicConstraint &other, double margin) const;
  virtual void clear(void);
  virtual bool decide(const planning_models::KinematicState &state, double &distance, bool verbose = false) const;
  virtual bool enabled(void) const;
  void print(std::ostream &out = std::cout) const;
  
  const planning_models::KinematicModel::LinkModel* getLinkModel(void) const
  {
    return link_model_;
  }
  
  const std::string& getReferenceFrame(void) const
  {
    return desired_rotation_frame_id_;
  }
  
  bool mobileReferenceFrame(void) const
  {
    return mobile_frame_;
  }
  
  const Eigen::Matrix3d& getDesiredRotationMatrix(void) const
  {
    return desired_rotation_matrix_;
  }
  
  double getXAxisTolerance(void) const
  {
    return absolute_x_axis_tolerance_;
  }
  
  double getYAxisTolerance(void) const
  {
    return absolute_y_axis_tolerance_;
  }
  
  double getZAxisTolerance(void) const
  {
    return absolute_z_axis_tolerance_;
  }
  
protected:
  
  const planning_models::KinematicModel::LinkModel *link_model_;
  Eigen::Matrix3d                                   desired_rotation_matrix_;
  Eigen::Matrix3d                                   desired_rotation_matrix_inv_;
  std::string                                       desired_rotation_frame_id_;
  bool                                              mobile_frame_;
  double                                            absolute_x_axis_tolerance_, absolute_y_axis_tolerance_, absolute_z_axis_tolerance_;
};

class PositionConstraint : public KinematicConstraint
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PositionConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
    KinematicConstraint(model, tf), link_model_(NULL)
  {
    type_ = POSITION_CONSTRAINT;
  }
  
  bool configure(const moveit_msgs::PositionConstraint &pc);
  virtual bool equal(const KinematicConstraint &other, double margin) const;
  virtual void clear(void);
  virtual bool decide(const planning_models::KinematicState &state, double &distance, bool verbose = false) const;
  virtual bool enabled(void) const;
  void print(std::ostream &out = std::cout) const;
  
  const planning_models::KinematicModel::LinkModel* getLinkModel(void) const
  {
    return link_model_;
  }
  
  const Eigen::Vector3d& getLinkOffset(void) const
  {
    return offset_;
  }
  
  bool hasLinkOffset(void) const
  {
    return has_offset_;
  }
  
  const boost::shared_ptr<bodies::Body>& getConstraintRegion(void) const
  {
    return constraint_region_;
  }
  
  const std::string& getReferenceFrame(void) const
  {
    return constraint_frame_id_;
  }
  
  bool mobileReferenceFrame(void) const
  {
    return mobile_frame_;
  }
  
protected:
  
  Eigen::Vector3d                                   offset_;
  bool                                              has_offset_;
  boost::shared_ptr<bodies::Body>                   constraint_region_;
  Eigen::Affine3d                                   constraint_region_pose_;
  bool                                              mobile_frame_;
  std::string                                       constraint_frame_id_;
  const planning_models::KinematicModel::LinkModel *link_model_;
};

class VisibilityConstraint : public KinematicConstraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisibilityConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf);
  
  bool configure(const moveit_msgs::VisibilityConstraint &vc);
  virtual bool equal(const KinematicConstraint &other, double margin) const;
  virtual void clear(void);
  shapes::Mesh* getVisibilityCone(const planning_models::KinematicState &state) const;
  void getMarkers(const planning_models::KinematicState &state, visualization_msgs::MarkerArray &markers) const;

  virtual bool decide(const planning_models::KinematicState &state, double &distance, bool verbose = false) const;
  virtual bool enabled(void) const;
  void print(std::ostream &out = std::cout) const;
  
protected:
  bool decideContact(collision_detection::Contact &contact) const;

  collision_detection::CollisionRobotPtr collision_robot_;
  collision_detection::CollisionWorldPtr collision_world_;
  bool                                   mobile_sensor_frame_;
  bool                                   mobile_target_frame_;
  std::string                            target_frame_id_;
  std::string                            sensor_frame_id_;
  Eigen::Affine3d                        sensor_pose_;
  int                                    sensor_view_direction_;
  Eigen::Affine3d                        target_pose_;
  unsigned int                           cone_sides_;
  std::vector<Eigen::Vector3d>           points_;
  double                                 target_radius_;
  double                                 max_view_angle_;
  double                                 max_range_angle_;
};

class KinematicConstraintSet
{
public:
  
  KinematicConstraintSet(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
    model_(model), tf_(tf)
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
  bool add(const std::vector<moveit_msgs::VisibilityConstraint> &pc);
  
  /** \brief Decide whether the set of constraints is satisfied  */
  bool decide(const planning_models::KinematicState &state, double &distance, bool verbose = false) const;
  
  /// \todo document this
  bool decide(const planning_models::KinematicState &state,
              moveit_msgs::ConstraintEvalResults& results,
              bool verbose) const;
  
  bool equal(const KinematicConstraintSet &other, double margin) const;
  
  /** \brief Print the constraint data */
  void print(std::ostream &out = std::cout) const;
  
  /** \brief Get the active position constraints */
  const std::vector<moveit_msgs::PositionConstraint>& getPositionConstraints(void) const
  {
    return position_constraints_;
  }
  
  /** \brief Get the active orientation constraints */
  const std::vector<moveit_msgs::OrientationConstraint>& getOrientationConstraints(void) const
  {
    return orientation_constraints_;
  }
  
  /** \brief Get the active pose constraints */
  const std::vector<moveit_msgs::JointConstraint>& getJointConstraints(void) const
  {
    return joint_constraints_;
  }
  
  /** \brief Get the active visibility constraints */
  const std::vector<moveit_msgs::VisibilityConstraint>& getVisibilityConstraints(void) const
  {
    return visibility_constraints_;
  }
  
  /** \brief Get all the contained constraints */
  const moveit_msgs::Constraints& getAllConstraints(void) const
  {
    return all_constraints_;
  }
  
  bool empty(void) const
  {
    return kinematic_constraints_.empty();
  }
  
protected:
  
  planning_models::KinematicModelConstPtr         model_;
  planning_models::TransformsConstPtr             tf_;
  
  std::vector<KinematicConstraintPtr>             kinematic_constraints_;
  
  std::vector<moveit_msgs::JointConstraint>       joint_constraints_;
  std::vector<moveit_msgs::PositionConstraint>    position_constraints_;
  std::vector<moveit_msgs::OrientationConstraint> orientation_constraints_;
  std::vector<moveit_msgs::VisibilityConstraint>  visibility_constraints_;
  moveit_msgs::Constraints                        all_constraints_;
  
};

typedef boost::shared_ptr<KinematicConstraintSet> KinematicConstraintSetPtr;
typedef boost::shared_ptr<const KinematicConstraintSet> KinematicConstraintSetConstPtr;

bool doesKinematicStateObeyConstraints(const planning_models::KinematicState& state,
                                       const planning_models::TransformsConstPtr& tf,
                                       const moveit_msgs::Constraints& constraints,
                                       bool verbose);

bool doesKinematicStateObeyConstraints(const planning_models::KinematicState& state,
                                       const planning_models::TransformsConstPtr& tf,
                                       const moveit_msgs::Constraints& constraints,
                                       moveit_msgs::ConstraintEvalResults& status,
                                       bool verbose);

}


#endif
