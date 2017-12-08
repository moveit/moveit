/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman */

#ifndef MOVEIT_KINEMATICS_BASE_KINEMATICS_BASE_
#define MOVEIT_KINEMATICS_BASE_KINEMATICS_BASE_

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/macros/class_forward.h>
#include <ros/node_handle.h>
#include <console_bridge/console.h>

#include <boost/function.hpp>
#include <string>

namespace moveit
{
namespace core
{
class JointModelGroup;
class RobotState;
}
}

/** @brief API for forward and inverse kinematics */
namespace kinematics
{
/*
 * @enum DiscretizationMethods
 *
 * @brief Flags for choosing the type discretization method applied on the redundant joints during an ik query
 */
namespace DiscretizationMethods
{
enum DiscretizationMethod
{
  NO_DISCRETIZATION = 1, /**< The redundant joints will be fixed at their current value. */
  ALL_DISCRETIZED,       /**< All redundant joints will be discretized uniformly */
  SOME_DISCRETIZED, /**< Some redundant joints will be discretized uniformly. The unused redundant joints will be fixed
                       at their
                         current value */
  ALL_RANDOM_SAMPLED, /**< the discretization for each redundant joint will be randomly generated.*/
  SOME_RANDOM_SAMPLED /**< the discretization for some redundant joint will be randomly generated.
                           The unused redundant joints will be fixed at their current value. */
};
}
typedef DiscretizationMethods::DiscretizationMethod DiscretizationMethod;

/*
 * @enum KinematicErrors
 * @brief Kinematic error codes that occur in a ik query
 */
namespace KinematicErrors
{
enum KinematicError
{
  OK = 1,                              /**< No errors*/
  UNSUPORTED_DISCRETIZATION_REQUESTED, /**< Discretization method isn't supported by this implementation */
  DISCRETIZATION_NOT_INITIALIZED,      /**< Discretization values for the redundancy has not been set. See
                                            setSearchDiscretization(...) method*/
  MULTIPLE_TIPS_NOT_SUPPORTED,         /**< Only single tip link support is allowed */
  EMPTY_TIP_POSES,                     /**< Empty ik_poses array passed */
  IK_SEED_OUTSIDE_LIMITS,              /**< Ik seed is out of bounds*/
  SOLVER_NOT_ACTIVE,                   /**< Solver isn't active */
  NO_SOLUTION                          /**< A valid joint solution that can reach this pose(s) could not be found */

};
}
typedef KinematicErrors::KinematicError KinematicError;

/**
 * @struct KinematicsQueryOptions
 * @brief A set of options for the kinematics solver
 */
struct KinematicsQueryOptions
{
  KinematicsQueryOptions()
    : lock_redundant_joints(false)
    , return_approximate_solution(false)
    , discretization_method(DiscretizationMethods::NO_DISCRETIZATION)
  {
  }

  bool lock_redundant_joints;                 /**<  KinematicsQueryOptions#lock_redundant_joints. */
  bool return_approximate_solution;           /**<  KinematicsQueryOptions#return_approximate_solution. */
  DiscretizationMethod discretization_method; /**< Enumeration value that indicates the method for discretizing the
                                                 redundant. joints KinematicsQueryOptions#discretization_method. */
};

/*
 * @struct KinematicsResult
 * @brief Reports result details of an ik query
 *
 * This struct is used as an output argument of the getPositionIK(...) method that returns multiple joint solutions.
 * It contains the type of error that led to a failure or KinematicErrors::OK when a set of joint solutions is found.
 * The solution percentage shall provide a ration of solutions found over solutions searched.
 *
 */
struct KinematicsResult
{
  KinematicError kinematic_error; /**< Error code that indicates the type of failure */
  double solution_percentage;     /**< The percentage of solutions achieved over the total number
                                       of solutions explored. */
};

MOVEIT_CLASS_FORWARD(KinematicsBase);

/**
 * @class KinematicsBase
 * @brief Provides an interface for kinematics solvers.
 */
class KinematicsBase
{
public:
  static const double DEFAULT_SEARCH_DISCRETIZATION; /* = 0.1 */
  static const double DEFAULT_TIMEOUT;               /* = 1.0 */

  /** @brief The signature for a callback that can compute IK */
  typedef boost::function<void(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_solution,
                               moveit_msgs::MoveItErrorCodes& error_code)>
      IKCallbackFn;

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool
  getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const = 0;

  /**
   * @brief Given a desired pose of the end-effector, compute the set joint angles solutions that are able to reach it.
   *
   * This is a default implementation that returns only one solution and so its result is equivalent to calling
   * 'getPositionIK(...)' with a zero initialized seed.
   *
   * @param ik_poses  The desired pose of each tip link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solutions A vector of vectors where each entry is a valid joint solution
   * @param result A struct that reports the results of the query
   * @param options An option struct which contains the type of redundancy discretization used. This default
   *                implementation only supports the KinematicSearches::NO_DISCRETIZATION method; requesting any
   *                other will result in failure.
   * @return True if a valid set of solutions was found, false otherwise.
   */
  virtual bool getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                             std::vector<std::vector<double> >& solutions, KinematicsResult& result,
                             const kinematics::KinematicsQueryOptions& options) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const = 0;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the
   * current seed state
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const = 0;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const = 0;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the
   * current seed state
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const = 0;

  /**
   * @brief Given a set of desired poses for a planning group with multiple end-effectors, search for the joint angles
   * required to reach them. This is useful for e.g. biped robots that need to perform whole-body IK.
   * Not necessary for most robots that have kinematic chains.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_poses the desired pose of each tip link, in the same order as the getTipFrames() vector
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the
   * current seed state
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param options container for other IK options
   * @param context_state (optional) the context in which this request
   *        is being made.  The position values corresponding to
   *        joints in the current group may not match those in
   *        ik_seed_state.  The values in ik_seed_state are the ones
   *        to use.  This is passed just to provide the \em other
   *        joint values, in case they are needed for context, like
   *        with an IK solver that computes a balanced result for a
   *        biped.
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool
  searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                   double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
                   const moveit::core::RobotState* context_state = NULL) const
  {
    // For IK solvers that do not support multiple poses, fall back to single pose call
    if (ik_poses.size() == 1)
    {
      // Check if a solution_callback function was provided and call the corresponding function
      if (solution_callback)
      {
        return searchPositionIK(ik_poses[0], ik_seed_state, timeout, consistency_limits, solution, solution_callback,
                                error_code, options);
      }
      else
      {
        return searchPositionIK(ik_poses[0], ik_seed_state, timeout, consistency_limits, solution, error_code, options);
      }
    }

    // Otherwise throw error because this function should have been implemented
    logError("moveit.kinematics_base: This kinematic solver does not support searchPositionIK with multiple poses");
    return false;
  }

  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   * @param link_names A set of links for which FK needs to be computed
   * @param joint_angles The state for which FK is being computed
   * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::Pose>& poses) const = 0;

  /**
   * @brief Set the parameters for the solver, for use with kinematic chain IK solvers
   * @param robot_description This parameter can be used as an identifier for the robot kinematics it is computed for;
   * For example, the name of the ROS parameter that contains the robot description;
   * @param group_name The group for which this solver is being configured
   * @param base_frame The base frame in which all input poses are expected.
   * This may (or may not) be the root frame of the chain that the solver operates on
   * @param tip_frame The tip of the chain
   * @param search_discretization The discretization of the search when the solver steps through the redundancy
   */
  virtual void setValues(const std::string& robot_description, const std::string& group_name,
                         const std::string& base_frame, const std::string& tip_frame, double search_discretization);

  /**
   * @brief Set the parameters for the solver, for use with non-chain IK solvers
   * @param robot_description This parameter can be used as an identifier for the robot kinematics it is computed for;
   * For example, the name of the ROS parameter that contains the robot description;
   * @param group_name The group for which this solver is being configured
   * @param base_frame The base frame in which all input poses are expected.
   * This may (or may not) be the root frame of the chain that the solver operates on
   * @param tip_frames A vector of tips of the kinematic tree
   * @param search_discretization The discretization of the search when the solver steps through the redundancy
   */
  virtual void setValues(const std::string& robot_description, const std::string& group_name,
                         const std::string& base_frame, const std::vector<std::string>& tip_frames,
                         double search_discretization);

  /**
   * @brief  Initialization function for the kinematics, for use with kinematic chain IK solvers
   * @param robot_description This parameter can be used as an identifier for the robot kinematics it is computed for;
   * For example, the name of the ROS parameter that contains the robot description;
   * @param group_name The group for which this solver is being configured
   * @param base_frame The base frame in which all input poses are expected.
   * This may (or may not) be the root frame of the chain that the solver operates on
   * @param tip_frame The tip of the chain
   * @param search_discretization The discretization of the search when the solver steps through the redundancy
   * @return True if initialization was successful, false otherwise
   */
  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& base_frame, const std::string& tip_frame,
                          double search_discretization) = 0;

  /**
   * @brief  Initialization function for the kinematics, for use with non-chain IK solvers
   * @param robot_description This parameter can be used as an identifier for the robot kinematics is computed for;
   * For example, rhe name of the ROS parameter that contains the robot description;
   * @param group_name The group for which this solver is being configured
   * @param base_frame The base frame in which all input poses are expected.
   * This may (or may not) be the root frame of the chain that the solver operates on
   * @param tip_frames A vector of tips of the kinematic tree
   * @param search_discretization The discretization of the search when the solver steps through the redundancy
   * @return True if initialization was successful, false otherwise
   */
  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& base_frame, const std::vector<std::string>& tip_frames,
                          double search_discretization)
  {
    // For IK solvers that do not support multiple tip frames, fall back to single pose call
    if (tip_frames.size() == 1)
    {
      return initialize(robot_description, group_name, base_frame, tip_frames[0], search_discretization);
    }

    logError("moveit.kinematics_base: This kinematic solver does not support initialization with more than one tip "
             "frames");
    return false;
  }

  /**
   * @brief  Return the name of the group that the solver is operating on
   * @return The string name of the group that the solver is operating on
   */
  virtual const std::string& getGroupName() const
  {
    return group_name_;
  }

  /**
   * @brief  Return the name of the frame in which the solver is operating. This is usually a link name.
   * No namespacing (e.g., no "/" prefix) should be used.
   * @return The string name of the frame in which the solver is operating
   */
  virtual const std::string& getBaseFrame() const
  {
    return base_frame_;
  }

  /**
   * @brief  Return the name of the tip frame of the chain on which the solver is operating. This is usually a link
   * name.
   * No namespacing (e.g., no "/" prefix) should be used.
   * Deprecated in favor of getTipFrames(), but will remain for foreseeable future for backwards compatibility
   * @return The string name of the tip frame of the chain on which the solver is operating
   */
  virtual const std::string& getTipFrame() const
  {
    if (tip_frames_.size() > 1)
      logError("moveit.kinematics_base: This kinematic solver has more than one tip frame, do not call getTipFrame()");

    return tip_frame_;  // for backwards-compatibility. should actually use tip_frames_[0]
  }

  /**
   * @brief  Return the names of the tip frames of the kinematic tree on which the solver is operating.
   * This is usually a link name. No namespacing (e.g., no "/" prefix) should be used.
   * @return The vector of names of the tip frames of the kinematic tree on which the solver is operating
   */
  virtual const std::vector<std::string>& getTipFrames() const
  {
    return tip_frames_;
  }

  /**
   * @brief Set a set of redundant joints for the kinematics solver to use.
   * This can fail, depending on the IK solver and choice of redundant joints!. Also, it sets
   * the discretization values for each redundant joint to a default value.
   * @param redundant_joint_indices The set of redundant joint indices (corresponding to
   * the list of joints you get from getJointNames()).
   * @return False if any of the input joint indices are invalid (exceed number of
   * joints)
   */
  virtual bool setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices);

  /**
   * @brief Set a set of redundant joints for the kinematics solver to use.
   * This function is just a convenience function that calls the previous definition of setRedundantJoints()
   * @param redundant_joint_names The set of redundant joint names.
   * @return False if any of the input joint indices are invalid (exceed number of
   * joints)
   */
  bool setRedundantJoints(const std::vector<std::string>& redundant_joint_names);

  /**
   * @brief Get the set of redundant joints
   */
  virtual void getRedundantJoints(std::vector<unsigned int>& redundant_joint_indices) const
  {
    redundant_joint_indices = redundant_joint_indices_;
  }

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  virtual const std::vector<std::string>& getJointNames() const = 0;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  virtual const std::vector<std::string>& getLinkNames() const = 0;

  /**
   * \brief Check if this solver supports a given JointModelGroup.
   *
   * Override this function to check if your kinematics solver
   * implementation supports the given group.
   *
   * The default implementation just returns jmg->isChain(), since
   * solvers written before this function was added all supported only
   * chain groups.
   *
   * \param jmg the planning group being proposed to be solved by this IK solver
   * \param error_text_out If this pointer is non-null and the group is
   *          not supported, this is filled with a description of why it's not
   *          supported.
   * \return True if the group is supported, false if not.
   */
  virtual bool supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out = NULL) const;

  /**
   * @brief  Set the search discretization value for all the redundant joints
   */
  void setSearchDiscretization(double sd)
  {
    redundant_joint_discretization_.clear();
    for (std::vector<unsigned int>::iterator i = redundant_joint_indices_.begin(); i != redundant_joint_indices_.end();
         i++)
    {
      redundant_joint_discretization_[*i] = sd;
    }
  }

  /**
   * @brief Sets individual discretization values for each redundant joint.
   *
   * Calling this method replaces previous discretization settings.
   *
   * @param discretization a map of joint indices and discretization value pairs.
   */
  void setSearchDiscretization(const std::map<int, double>& discretization)
  {
    redundant_joint_discretization_.clear();
    redundant_joint_indices_.clear();
    for (std::map<int, double>::const_iterator i = discretization.begin(); i != discretization.end(); i++)
    {
      redundant_joint_discretization_.insert(*i);
      redundant_joint_indices_.push_back(i->first);
    }
  }

  /**
   * @brief  Get the value of the search discretization
   */
  double getSearchDiscretization(int joint_index = 0) const
  {
    if (redundant_joint_discretization_.count(joint_index) > 0)
    {
      return redundant_joint_discretization_.at(joint_index);
    }
    else
    {
      return 0.0;  // returned when there aren't any redundant joints
    }
  }

  /**
   * @brief Returns the set of supported kinematics discretization search types.  This implementation only supports
   * the DiscretizationMethods::ONE search.
   */
  std::vector<DiscretizationMethod> getSupportedDiscretizationMethods() const
  {
    return supported_methods_;
  }

  /** @brief For functions that require a timeout specified but one is not specified using arguments,
      a default timeout is used, as set by this function (and initialized to KinematicsBase::DEFAULT_TIMEOUT) */
  void setDefaultTimeout(double timeout)
  {
    default_timeout_ = timeout;
  }

  /** @brief For functions that require a timeout specified but one is not specified using arguments,
      this default timeout is used */
  double getDefaultTimeout() const
  {
    return default_timeout_;
  }

  /**
   * @brief  Virtual destructor for the interface
   */
  virtual ~KinematicsBase()
  {
  }

  KinematicsBase()
    : tip_frame_("DEPRECATED")
    ,  // help users understand why this variable might not be set
       // (if multiple tip frames provided, this variable will be unset)
    search_discretization_(DEFAULT_SEARCH_DISCRETIZATION)
    , default_timeout_(DEFAULT_TIMEOUT)
  {
    supported_methods_.push_back(DiscretizationMethods::NO_DISCRETIZATION);
  }

protected:
  std::string robot_description_;
  std::string group_name_;
  std::string base_frame_;
  std::vector<std::string> tip_frames_;
  std::string tip_frame_;  // DEPRECATED - this variable only still exists for backwards compatibility with
                           // previously generated custom ik solvers like IKFast

  double search_discretization_;  // DEPRECATED - this variable only still exists for backwards compatibility
                                  // with previous implementations.  Discretization values for each joint are
                                  // now stored in the redundant_joint_discretization_ member

  double default_timeout_;
  std::vector<unsigned int> redundant_joint_indices_;
  std::map<int, double> redundant_joint_discretization_;
  std::vector<DiscretizationMethod> supported_methods_;

  /**
   * @brief Enables kinematics plugins access to parameters that are defined
   * for the private namespace and inside 'robot_description_kinematics'.
   * Parameters are searched in the following locations and order
   *
   * ~/<group_name>/<param>
   * ~/<param>
   * robot_description_kinematics/<group_name>/<param>
   * robot_description_kinematics/<param>
   *
   * This order maintains default behavior by keeping the private namespace
   * as the predominant configuration but also allows groupwise specifications.
   */
  template <typename T>
  inline bool lookupParam(const std::string& param, T& val, const T& default_val) const
  {
    ros::NodeHandle pnh("~");
    if (pnh.hasParam(group_name_ + "/" + param))
    {
      val = pnh.param(group_name_ + "/" + param, default_val);
      return true;
    }

    if (pnh.hasParam(param))
    {
      val = pnh.param(param, default_val);
      return true;
    }

    ros::NodeHandle nh;
    if (nh.hasParam("robot_description_kinematics/" + group_name_ + "/" + param))
    {
      val = nh.param("robot_description_kinematics/" + group_name_ + "/" + param, default_val);
      return true;
    }

    if (nh.hasParam("robot_description_kinematics/" + param))
    {
      val = nh.param("robot_description_kinematics/" + param, default_val);
      return true;
    }

    val = default_val;

    return false;
  }

private:
  std::string removeSlash(const std::string& str) const;
};
};

#endif
