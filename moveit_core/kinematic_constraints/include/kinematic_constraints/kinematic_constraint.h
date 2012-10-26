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

#ifndef MOVEIT_KINEMATIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_
#define MOVEIT_KINEMATIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_

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

/** \brief Representation and evaluation of kinematic constraints */
namespace kinematic_constraints
{

/// \brief Struct for containing the results of constraint evaluation
struct ConstraintEvaluationResult
{
  /** 
   * \brief Constructor
   * 
   * @param [in] result_satisfied True if the constraint evaluated to true, otherwise false
   * @param [in] dist The distance from 
   * 
   * @return 
   */
  ConstraintEvaluationResult(bool result_satisfied = false, double dist = 0.0) : satisfied(result_satisfied), distance(dist)
  {
  }
  
  bool   satisfied;             /**< \brief Whether or not the constraint is satisfied */
  double distance;              /**< \brief The distance from the constraint */
};

/// \brief Base class for representing a kinematic constraint
class KinematicConstraint
{
public:
  
  /// \brief Enum for representing a constraint
  enum ConstraintType
    {
      UNKNOWN_CONSTRAINT, JOINT_CONSTRAINT, POSITION_CONSTRAINT, ORIENTATION_CONSTRAINT, VISIBILITY_CONSTRAINT
    };
  
  /** 
   * \brief Constructor
   * 
   * @param [in] model The kinematic model used for constraint evaluation
   * @param [in] tf The transform set used for constraint evaluation
   */
  KinematicConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf);
  virtual ~KinematicConstraint(void);
  
  /** \brief Clear the stored constraint */
  virtual void clear(void) = 0;
  
  /** 
   * \brief Decide whether the constraint is satisfied in the indicated state
   * 
   * @param [in] state The kinematic state used for evaluation
   * @param [in] verbose Whether or not to print output
   * 
   * @return 
   */
  virtual ConstraintEvaluationResult decide(const planning_models::KinematicState &state, bool verbose = false) const = 0;
  
  /** \brief This function returns true if this constraint is
      configured and able to decide whether states do meet the
      constraint or not. If this function returns false it means
      that decide() will always return true -- there is no
      constraint to be checked. */
  virtual bool enabled(void) const = 0;
  
  /** 
   * \brief Check if two constraints are the same.  This means that
   * the types are the same, the subject of the constraint is the
   * same, and all values associated with the constraint are within a
   * margin.  The other constraint must also be enabled.
   * 
   * @param [in] other The other constraint to test
   * @param [in] margin The margin to apply to all values associated with constraint
   * 
   * @return True if equal, otherwise false
   */
  virtual bool equal(const KinematicConstraint &other, double margin) const = 0;
  
  /** 
   * \brief Get the type of constraint
   * 
   * 
   * @return The constraint type
   */
  ConstraintType getType(void) const
  {
    return type_;
  }
  
  /** 
   * \brief Print the constraint data
   * 
   * @param [in] out The file descriptor for printing
   */
  virtual void print(std::ostream &out = std::cout) const
  {
  }
  
  /** 
   * 
   * \brief The weight of a constraint is a multiplicative factor associated to the distance computed by the decide() function 
   * 
   * @return The constraint weight
   */
  double getConstraintWeight(void) const
  {
    return constraint_weight_;
  }
  
  /** 
   * 
   * 
   * 
   * @return The kinematic model associated with this constraint
   */
  const planning_models::KinematicModelConstPtr& getKinematicModel(void) const
  {
    return kmodel_;
  }
  /** 
   * 
   * 
   * 
   * @return The transforms associated with this constraint
   */
  const planning_models::TransformsConstPtr& getTransforms(void) const
  {
    return tf_;
  }
  
protected:
  
  ConstraintType                          type_; /**< \brief The type of the constraint */
  planning_models::KinematicModelConstPtr kmodel_; /**< \brief The kinematic model associated with this constraint */
  planning_models::TransformsConstPtr     tf_; /**< \brief The transforms associated with the constraint */
  double                                  constraint_weight_; /**< \brief The weight of a constraint is a multiplicative factor associated to the distance computed by the decide() function  */
};

typedef boost::shared_ptr<KinematicConstraint> KinematicConstraintPtr; /**< \brief boost::shared_ptr to a Kinematic Constraint */
typedef boost::shared_ptr<const KinematicConstraint> KinematicConstraintConstPtr; /**< \brief boost::shared_ptr to a Const Kinematic Constraint */

/**
 * \brief Class for handling single DOF joint constraints.
 *
 * This class handles single DOF constraints expressed as a tolerance
 * above and below a target position.  Multi-DOF joints can be
 * accomodated by using local name formulations - i.e. for a planar
 * joint specifying a constraint in terms of <planar_joint_name>/x.
 * Continuous revolute single DOF joints will be evaluated based on
 * wrapping around 3.14 and -3.14.  Type will be
 * JOINT_CONSTRAINT. TODO - specify whether or not tolerances can be
 * negative.
 */
class JointConstraint : public KinematicConstraint
{
public:

  JointConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
    KinematicConstraint(model, tf), joint_model_(NULL)
  {
    type_ = JOINT_CONSTRAINT;
  }

  /** 
   * \brief Configure the constraint based on a
   * moveit_msgs::JointConstraint
   *
   * For the configure command to be successful, the joint must exist
   * in the kinematic model, the joint must not be a multi-DOF joint
   * (for these joints, local variables should be used), and TODO the
   * tolerance values must be positive.
   * 
   * @param [in] jc JointConstraint for configuration
   * 
   * @return True if constraint can be configured from jc
   */  
  bool configure(const moveit_msgs::JointConstraint &jc);

  /** 
   * \brief Check if two joint constraints are the same.  
   *
   * This means that the types are the same, the subject of the
   * constraint is the same, and all values associated with the
   * constraint are within a margin.  The other constraint must also
   * be enabled.  For this to be true of joint constraints, they must
   * act on the same joint, and the position and tolerance values must
   * be within the margins.
   * 
   * @param [in] other The other constraint to test
   * @param [in] margin The margin to apply to all values associated with constraint
   * 
   * @return True if equal, otherwise false
   */
  virtual bool equal(const KinematicConstraint &other, double margin) const;
  virtual ConstraintEvaluationResult decide(const planning_models::KinematicState &state, bool verbose = false) const;
  virtual bool enabled(void) const;
  virtual void clear(void);
  virtual void print(std::ostream &out = std::cout) const;
  
  /** 
   * \brief Get the joint model for which this constraint operates
   * 
   * @return The relevant joint model if enabled, and otherwise NULL
   */
  const planning_models::KinematicModel::JointModel* getJointModel(void) const
  {
    return joint_model_;
  }
  /** 
   * \brief Gets the local variable name if this constraint was
   * configured for a part of a multi-DOF joint
   * 
   * 
   * @return The component of the joint name after the slash, or the
   * empty string if there is no local variable name
   */  
  const std::string& getLocalVariableName(void) const
  {
    return local_variable_name_;
  }

  /** 
   *  \brief Gets the joint variable name.  
   *
   * This will be the part of the name in front of the slash if the
   * joint is multi-DOF and will other just be the joint set in the
   * joint field.
   * 
   * 
   * @return The joint variable name
   */
  const std::string& getJointVariableName(void) const
  {
    return joint_variable_name_;
  }
  
  /** 
   * \brief Gets the desired position component of the constraint 
   * 
   * 
   * @return The desired joint position
   */
  double getDesiredJointPosition(void) const
  {
    return joint_position_;
  }

  /** 
   * \brief Gets the upper tolerance component of the joint constraint
   * 
   * 
   * @return The above joint tolerance
   */
  double getJointToleranceAbove(void) const
  {
    return joint_tolerance_above_;
  }

  /** 
   * \brief Gets the lower tolerance component of the joint constraint
   * 
   * 
   * @return The below joint tolerance
   */
  double getJointToleranceBelow(void) const
  {
    return joint_tolerance_below_;
  }
  
protected:
  
  const planning_models::KinematicModel::JointModel *joint_model_; /**< \brief The joint from the kinematic model for this constraint */
  bool                                               joint_is_continuous_; /**< \brief Whether or not the joint is continuous */
  std::string                                        local_variable_name_; /**< \brief The local variable name for a multi DOF joint, if any */
  std::string                                        joint_variable_name_; /**< \brief The joint variable name */
  double                                             joint_position_, joint_tolerance_above_, joint_tolerance_below_; /**< \brief Position and tolerance values*/
};

/**
 * \brief Class for constraints on the orientation of a link
 *
 * This class expresses an orientation constraint on a particular
 * link.  The constraint is specified in terms of a quaternion, with
 * tolerances on X,Y, and Z axes.  The rotation difference is computed
 * based on the ZXZ Euler angle formulation.  The header on the
 * quaternion can be specified in terms of either a fixed frame or a
 * mobile frame.  The type value will return ORIENTATION_CONSTRAINT.
 *
 */
class OrientationConstraint : public KinematicConstraint
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OrientationConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
    KinematicConstraint(model, tf), link_model_(NULL)
  {
    type_ = ORIENTATION_CONSTRAINT;
  }

  /** 
   * \brief Configure the constraint based on a
   * moveit_msgs::OrientationConstraint
   *
   * For the configure command to be successful, the link must exist
   * in the kinematic model. TODO - more conditions
   * 
   * @param [in] oc OrientationConstraint for configuration
   * 
   * @return True if constraint can be configured from oc
   */  
  bool configure(const moveit_msgs::OrientationConstraint &oc);

  /** 
   * \brief Check if two orientation constraints are the same.  

   * This means that the types are the same, the subject of the
   * constraint is the same, and all values associated with the
   * constraint are within a margin.  The other constraint must also
   * be enabled.  For this to be true of orientation constraints, they
   * must act on the same link, the rotations specified by the
   * quaternions must be within the margin, and the tolerances must
   * all be within the margin.
   * 
   * @param [in] other The other constraint to test
   * @param [in] margin The margin to apply to all values associated with constraint
   * 
   * @return True if equal, otherwise false
   */
  virtual bool equal(const KinematicConstraint &other, double margin) const;
  virtual void clear(void);
  virtual ConstraintEvaluationResult decide(const planning_models::KinematicState &state, bool verbose = false) const;
  virtual bool enabled(void) const;
  virtual void print(std::ostream &out = std::cout) const;
  
  /** 
   * \brief Gets the subject link model
   * 
   * 
   * @return Returns the current link model
   */
  const planning_models::KinematicModel::LinkModel* getLinkModel(void) const
  {
    return link_model_;
  }
  
  /** 
   * \brief The target frame of the planning_models::Transforms class,
   * for interpreting the rotation frame.
   *
   * @return The reference frame.
   */
  const std::string& getReferenceFrame(void) const
  {
    return desired_rotation_frame_id_;
  }
  
  /** 
   * \brief Whether or not a mobile reference frame is being employed.
   * 
   * @return True if a mobile reference frame is being employed, and
   * otherwise false.
   */
  bool mobileReferenceFrame(void) const
  {
    return mobile_frame_;
  }

  /** 
   * \brief The rotation target in the reference frame.
   * 
    * @return The target rotation
   */
  const Eigen::Matrix3d& getDesiredRotationMatrix(void) const
  {
    return desired_rotation_matrix_;
  }
  
  /** 
   * \brief Gets the X axis tolerance
   * 
   * 
   * @return The X axis tolerance
   */
  double getXAxisTolerance(void) const
  {
    return absolute_x_axis_tolerance_;
  }

  /** 
   * \brief Gets the Y axis tolerance
   * 
   * 
   * @return The Y axis tolerance
   */  
  double getYAxisTolerance(void) const
  {
    return absolute_y_axis_tolerance_;
  }
  
  /** 
   * \brief Gets the Z axis tolerance
   * 
   * 
   * @return The Z axis tolerance
   */
  double getZAxisTolerance(void) const
  {
    return absolute_z_axis_tolerance_;
  }
  
protected:
  
  const planning_models::KinematicModel::LinkModel *link_model_; /**< \brief The target link model */
  Eigen::Matrix3d                                   desired_rotation_matrix_; /**< \brief The desired rotation matrix in the tf frame */
  Eigen::Matrix3d                                   desired_rotation_matrix_inv_; /**< \brief The inverse of the desired rotation matrix, precomputed for efficiency */
  std::string                                       desired_rotation_frame_id_; /**< \brief The target frame of the transform tree */
  bool                                              mobile_frame_; /**< \brief Whether or not the header frame is mobile or fixed */
  double                                            absolute_x_axis_tolerance_, absolute_y_axis_tolerance_, absolute_z_axis_tolerance_; /**< \brief Storage for the tolerances */
};


/**
 * \brief Class for constraints on the XYZ position of a link
 *
 * This class expresses X,Y,Z position constraints of a link.  The
 * position area is specified as a bounding volume consisting of one
 * or more shapes - either solid primitives or meshes. The pose
 * information in the volumes will be interpreted by using the header
 * information.  The header may either specify a fixed frame or a
 * mobile frame.  Additionally, a target offset specified in the frame
 * of the link being constrained can be specified.  The type value
 * will return POSITION_CONSTRAINT.
 *
 */
class PositionConstraint : public KinematicConstraint
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PositionConstraint(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
    KinematicConstraint(model, tf), link_model_(NULL)
  {
    type_ = POSITION_CONSTRAINT;
  }

  /** 
   * \brief Configure the constraint based on a
   * moveit_msgs::PositionConstraint 
   *
   * For the configure command to be successful, the link must be
   * specified in the model, and one or more constrained regions must
   * be correctly specified, which requires containing a valid shape
   * and a pose for that shape.  If an invalid quaternion is passed
   * for a shape, the identity quaternion will be substituted.
   * 
   * @param [in] pc moveit_msgs::PositionConstraint for configuration
   * 
   * @return True if constraint can be configured from pc
   */  
  bool configure(const moveit_msgs::PositionConstraint &pc);

  /** 
   * \brief Check if two constraints are the same.  For position
   * constraints this means that the types are the same, the link
   * model is the same, the frame of the constraint is the same, the
   * target offsets are no further than the margin apart, that the
   * constraint region poses and volumes are within the margin, and
   * that constrained regions are in the same order.
   * 
   * @param [in] other The other constraint to test
   * @param [in] margin The margin to apply to all values associated with constraint
   * 
   * @return True if equal, otherwise false
   */
  virtual bool equal(const KinematicConstraint &other, double margin) const;
  virtual void clear(void);
  virtual ConstraintEvaluationResult decide(const planning_models::KinematicState &state, bool verbose = false) const;
  virtual bool enabled(void) const;
  virtual void print(std::ostream &out = std::cout) const;

  /** 
   * \brief Returns the associated link model, or NULL if not enabled
   * 
   * 
   * @return The link model
   */  
  const planning_models::KinematicModel::LinkModel* getLinkModel(void) const
  {
    return link_model_;
  }
  
  /** 
   * \brief Returns the target offset
   * 
   * 
   * @return The target offset
   */
  const Eigen::Vector3d& getLinkOffset(void) const
  {
    return offset_;
  }
  
  /** 
   * \brief If the constraint is enabled and the link offset is
   * substantially different than zero
   * 
   * 
   * @return Whether or not there is a link offset
   */
  bool hasLinkOffset(void) const
  {
    if(!enabled()) return false;
    return has_offset_;
  }
  

  /** 
   * \brief Returns all the constraint regions
   * 
   * 
   * @return The constraint regions
   */
  const std::vector<bodies::BodyPtr>& getConstraintRegions(void) const
  {
    return constraint_region_;
  }
  
  /** 
   * \brief Returns the reference frame
   * 
   * 
   * @return The reference frame
   */
  const std::string& getReferenceFrame(void) const
  {
    return constraint_frame_id_;
  }
  
  /** 
   * \brief If enabled and the specified frame is a mobile frame,
   * return true.  Otherwise, returns false.
   * 
   * 
   * @return Whether a mobile reference frame is being employed
   */
  bool mobileReferenceFrame(void) const
  {
    if(!enabled()) return false;
    return mobile_frame_;
  }
  
protected:
  
  Eigen::Vector3d                                   offset_; /**< \brief The target offset */
  bool                                              has_offset_; /**< \brief Whether the offset is substantially different than 0.0 */
  std::vector<bodies::BodyPtr>                      constraint_region_; /**< \brief The constraint region vector */
  EigenSTL::vector_Affine3d                         constraint_region_pose_; /**< \brief The constraint region pose vector */
  bool                                              mobile_frame_; /**< \brief Whether or not a mobile frame is employed*/
  std::string                                       constraint_frame_id_; /**< \brief The constraint frame id */
  const planning_models::KinematicModel::LinkModel *link_model_; /**< \brief The link model constraint subject */
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

  virtual ConstraintEvaluationResult decide(const planning_models::KinematicState &state, bool verbose = false) const;
  virtual bool enabled(void) const;
  void print(std::ostream &out = std::cout) const;
  
protected:

  bool decideContact(collision_detection::Contact &contact) const;

  collision_detection::CollisionRobotPtr collision_robot_;
  bool                                   mobile_sensor_frame_;
  bool                                   mobile_target_frame_;
  std::string                            target_frame_id_;
  std::string                            sensor_frame_id_;
  Eigen::Affine3d                        sensor_pose_;
  int                                    sensor_view_direction_;
  Eigen::Affine3d                        target_pose_;
  unsigned int                           cone_sides_;
  EigenSTL::vector_Vector3d              points_;
  double                                 target_radius_;
  double                                 max_view_angle_;
  double                                 max_range_angle_;
};

/**
 * \brief A class that contains many different constraints, and can
 * check KinematicState versus the full set.  A set is satisfied if
 * and only if all constraints are satisfied.
 * 
 * The set may contain any number of different kinds of constraints.
 * All constraints, including invalid ones, are stored internally.  
 */
class KinematicConstraintSet
{
public:

  /** 
   * \brief Constructor
   * 
   * @param [in] model The kinematic model used for constraint evaluation
   * @param [in] tf The transform set used for constraint evaluation
   */
  KinematicConstraintSet(const planning_models::KinematicModelConstPtr &model, const planning_models::TransformsConstPtr &tf) :
    kmodel_(model), tf_(tf)
  {
  }
  
  ~KinematicConstraintSet(void)
  {
    clear();
  }
  
  /** \brief Clear the stored constraints */
  void clear(void);
  
  /** 
   * \brief Add all known constraints
   * 
   * @param [in] c A message potentially contain vectors of constraints of add types 
   * 
   * @return Whether or not all constraints could be successfully
   * configured given the contents of the message.  The
   * KinematicConstraintSet can still be used even if the addition
   * returns false.
   */  
  bool add(const moveit_msgs::Constraints &c);
  
  /** 
   * \brief Add a vector of joint constraints
   * 
   * @param [in] jc A vector of joint constraints
   * 
   * @return Will return true only if all constraints are valid, and false otherwise
   */
  bool add(const std::vector<moveit_msgs::JointConstraint> &jc);
  
  /** 
   * \brief Add a vector of position constraints
   * 
   * @param [in] pc A vector of position constraints
   * 
   * @return Will return true only if all constraints are valid, and false otherwise
   */
  bool add(const std::vector<moveit_msgs::PositionConstraint> &pc);
  
  /** 
   * \brief Add a vector of orientation constraints
   * 
   * @param [in] oc A vector of orientation constraints
   * 
   * @return Will return true only if all constraints are valid, and false otherwise
   */
  bool add(const std::vector<moveit_msgs::OrientationConstraint> &pc);
  
  /** 
   * \brief Add a vector of visibility constraints
   * 
   * @param [in] vc A vector of visibility constraints
   * 
   * @return Will return true only if all constraints are valid, and false otherwise
   */
  bool add(const std::vector<moveit_msgs::VisibilityConstraint> &pc);
  
  /** 
   * \brief Determines whether all constraints are satisfied by state,
   * returning a single evaluation result
   * 
   * @param [in] state The state to test
   * @param [in] verbose Whether or not to make each constraint give debug output
   * 
   * @return A single constraint evaluation result, where it will
   * report satisfied only if all constraints are satisfied, and with
   * a distance that is the sum of all individual distances.
   */
  ConstraintEvaluationResult decide(const planning_models::KinematicState &state, bool verbose = false) const;

  /** 
   * 
   * \brief Determines whether all constraints are satisfied by state,
   * returning a vector of results through a parameter in addition to
   * a summed result.
   * 
   * @param [in] state The state to test 
   * 
   * @param [out] results The individual results from constraint
   * evaluation on each constraint contained in the set.  
   *
   * @param [in] verbose Whether to print the results of each constraint
   * check.
   * 
   * @return A single constraint evaluation result, where it will
   * report satisfied only if all constraints are satisfied, and with
   * a distance that is the sum of all individual distances.
   */
  ConstraintEvaluationResult decide(const planning_models::KinematicState &state, std::vector<ConstraintEvaluationResult> &results, bool verbose = false) const;
  
  /** 
   * \brief Whether or not another KinematicConstraintSet is equal to
   * this one.
   * 
   * Equality means that for each constraint in this set there is a
   * constraint in the other set for which equal() is true with the
   * given margin.  Multiple constraints in this set can be matched to
   * single constraints in the other set.  Some constraints in the
   * other set may not be matched to constraints in this set.
   *
   * @param [in] other The other set against which to test 
   * @param [in] margin The margin to apply to all individual constraint equality tests
   * 
   * @return True if all constraints are matched, false otherwise
   */
  bool equal(const KinematicConstraintSet &other, double margin) const;
  
  /** 
   * \brief Print the constraint data
   * 
   * @param [in] out The file stream for printing 
   */
  void print(std::ostream &out = std::cout) const;
  
  /** 
   * \brief Get all position constraints in the set
   * 
   * 
   * @return All position constraints
   */
  const std::vector<moveit_msgs::PositionConstraint>& getPositionConstraints(void) const
  {
    return position_constraints_;
  }
  
  /** 
   * \brief Get all orientation constraints in the set
   * 
   * 
   * @return All orientation constraints
   */
  const std::vector<moveit_msgs::OrientationConstraint>& getOrientationConstraints(void) const
  {
    return orientation_constraints_;
  }
  
  /** 
   * \brief Get all joint constraints in the set
   * 
   * 
   * @return All joint constraints
   */
  const std::vector<moveit_msgs::JointConstraint>& getJointConstraints(void) const
  {
    return joint_constraints_;
  }
  
  /** 
   * \brief Get all visibility constraints in the set
   * 
   * 
   * @return All visibility constraints
   */
  const std::vector<moveit_msgs::VisibilityConstraint>& getVisibilityConstraints(void) const
  {
    return visibility_constraints_;
  }
  
  /** 
   * \brief Get all constraints in the set
   * 
   * 
   * @return All constraints in a single message
   */
  const moveit_msgs::Constraints& getAllConstraints(void) const
  {
    return all_constraints_;
  }
  
  /** 
   * \brief Returns whether or not there are any constraints in the set
   * 
   * 
   * @return True if there are no constraints, otherwise false.
   */
  bool empty(void) const
  {
    return kinematic_constraints_.empty();
  }
  
protected:
  
  planning_models::KinematicModelConstPtr         kmodel_; /**< \brief The kinematic model used for by the Set */
  planning_models::TransformsConstPtr             tf_; /**< \brief The transforms used by the Set */
  
  std::vector<KinematicConstraintPtr>             kinematic_constraints_; /**<  \brief Shared pointers to all the member constraints */
  
  std::vector<moveit_msgs::JointConstraint>       joint_constraints_; /**<  \brief Messages corresponding to all internal joint constraints */
  std::vector<moveit_msgs::PositionConstraint>    position_constraints_;/**<  \brief Messages corresponding to all internal position constraints */
  std::vector<moveit_msgs::OrientationConstraint> orientation_constraints_;/**<  \brief Messages corresponding to all internal orientation constraints */
  std::vector<moveit_msgs::VisibilityConstraint>  visibility_constraints_;/**<  \brief Messages corresponding to all internal visibility constraints */
  moveit_msgs::Constraints                        all_constraints_; /**<  \brief Messages corresponding to all internal constraints */
  
};

typedef boost::shared_ptr<KinematicConstraintSet> KinematicConstraintSetPtr; /**< \brief boost::shared_ptr to a KinematicConstraintSetPtr */
typedef boost::shared_ptr<const KinematicConstraintSet> KinematicConstraintSetConstPtr; /**< \brief boost::shared_ptr to a KinematicConstraintSet Const */

}


#endif
