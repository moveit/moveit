/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef MOVEIT_KINEMATICS_BASE_KINEMATICS_BASE_
#define MOVEIT_KINEMATICS_BASE_KINEMATICS_BASE_

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <boost/function.hpp>

/** @brief API for forward and inverse kinematics */
namespace kinematics {

/**
 * @class KinematicsBase
 * @brief Provides an interface for kinematics solvers.
 */
class KinematicsBase{
public:
      
  /** @brief The signature for a callback that can compute IK */
  typedef boost::function<void(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_solution, moveit_msgs::MoveItErrorCodes &error_code)> IKCallbackFn;
      
  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                             const std::vector<double> &ik_seed_state,
                             std::vector<double> &solution,
                             moveit_msgs::MoveItErrorCodes &error_code) const = 0;      

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                std::vector<double> &solution,
                                moveit_msgs::MoveItErrorCodes &error_code) const = 0;      

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limits the distance that any joint can be from the corresponding joints in the current seed state
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                moveit_msgs::MoveItErrorCodes &error_code) const = 0;      

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param desired_pose_callback A callback function for the desired link pose - could be used, e.g. to check for collisions for the end-effector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code) const = 0;      

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limit the distance that any joint can be from the current seed state
   * @param solution the solution vector
   * @param desired_pose_callback A callback function for the desired link pose - could be used, e.g. to check for collisions for the end-effector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code) const = 0;      
    
  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   * @param link_names A set of links for which FK needs to be computed
   * @param joint_angles The state for which FK is being computed
   * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool getPositionFK(const std::vector<std::string> &link_names,
                             const std::vector<double> &joint_angles, 
                             std::vector<geometry_msgs::Pose> &poses) const = 0;

  /**
   * @brief Set the parameters for the solver
   * @param group_name The group for which this solver is being configured
   * @param base_frame The base frame in which all input poses are expected. 
   * This may (or may not) be the root frame of the chain that the solver operates on
   * @param tip_frame The tip of the chain
   * @param search_discretization The discretization of the search when the solver steps through the redundancy
   */
  virtual void setValues(const std::string& group_name,
                         const std::string& base_frame,
                         const std::string& tip_frame,
                         double search_discretization) {
    group_name_ = group_name;
    base_frame_ = base_frame;
    tip_frame_ = tip_frame;
    search_discretization_ = search_discretization;
  }

  /**
   * @brief  Initialization function for the kinematics
   * @param group_name The group for which this solver is being configured
   * @param base_frame The base frame in which all input poses are expected. 
   * This may (or may not) be the root frame of the chain that the solver operates on
   * @param tip_frame The tip of the chain
   * @param search_discretization The discretization of the search when the solver steps through the redundancy
   * @return True if initialization was successful, false otherwise
   */
  virtual bool initialize(const std::string& group_name,
                          const std::string& base_frame,
                          const std::string& tip_frame,
                          double search_discretization) = 0;
    
  /**
   * @brief  Return the name of the group that the solver is operating on
   * @return The string name of the group that the solver is operating on
   */
  virtual const std::string& getGroupName() const {
    return group_name_;    
  }
    
  /**
   * @brief  Return the name of the frame in which the solver is operating
   * @return The string name of the frame in which the solver is operating
   */
  virtual const std::string& getBaseFrame() const {
    return base_frame_;
  }

  /**
   * @brief  Return the name of the tip frame of the chain on which the solver is operating
   * @return The string name of the tip frame of the chain on which the solver is operating
   */
  virtual const std::string& getTipFrame() const {
    return tip_frame_;
  }

  /**
   * @brief Set a set of redundant joints for the kinematics solver to use. 
   * @param redundant_joint_indices The set of redundant joint indices (corresponding to 
   * the list of joints you get from getJointNames()). 
   * @return False if any of the input joint indices are invalid (exceed number of 
   * joints)
   */
  virtual bool setRedundantJoints(const std::vector<unsigned int> &redundant_joint_indices)
  {
    for(std::size_t i=0; i < redundant_joint_indices.size(); ++i)
    {
      if(redundant_joint_indices[i] >= getJointNames().size())
      {
        return false;
      }      
    }    
    redundant_joint_indices_ = redundant_joint_indices;
    return true;    
  }
      
  /**
   * @brief Get the set of redundant joints
   */
  virtual void getRedundantJoints(std::vector<unsigned int> &redundant_joint_indices) const
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
   * @brief  Set the search discretization
   */
  void setSearchDiscretization(double sd) {
    search_discretization_ = sd;
  }

  /**
   * @brief  Get the value of the search discretization
   */
  double getSearchDiscretization() const {
    return search_discretization_;
  }

  /**
   * @brief  Virtual destructor for the interface
   */
  virtual ~KinematicsBase(){}

protected:
  std::string group_name_;
  std::string base_frame_;
  std::string tip_frame_;
  double search_discretization_;
  std::vector<unsigned int> redundant_joint_indices_;
  KinematicsBase(){}
};

typedef boost::shared_ptr<KinematicsBase> KinematicsBasePtr;
typedef boost::shared_ptr<const KinematicsBase> KinematicsBaseConstPtr;

};

#endif
