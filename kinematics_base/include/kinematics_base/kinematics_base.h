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
#ifndef KINEMATICS_BASE_
#define KINEMATICS_BASE_

#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/RobotState.h>

#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>

namespace kinematics {

  static const int SUCCESS = 1;
  static const int TIMED_OUT = -1;
  static const int NO_IK_SOLUTION = -2;
  static const int FRAME_TRANSFORM_FAILURE = -3;
  static const int IK_LINK_INVALID = -4;
  static const int IK_LINK_IN_COLLISION = -5;
  static const int STATE_IN_COLLISION = -6;
  static const int INVALID_LINK_NAME = -7;
  static const int GOAL_CONSTRAINTS_VIOLATED = -7;
  static const int INACTIVE = -8;

  /**
   * @class KinematicsBase
   * @brief Provides an interface for kinematics solvers.
   */
  class KinematicsBase{
    public:
      /**
       * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
       * @param ik_link_name - the name of the link for which IK is being computed
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @return True if a valid solution was found, false otherwise
       */
       virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  std::vector<double> &solution,
                                  int &error_code) = 0;      

      /**
       * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
       * This particular method is intended for "searching" for a solutions by stepping through the redundancy
       * (or other numerical routines).
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @return True if a valid solution was found, false otherwise
       */
       virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                     const std::vector<double> &ik_seed_state,
                                     const double &timeout,
                                     std::vector<double> &solution,
                                     int &error_code) = 0;      

      /**
       * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
       * This particular method is intended for "searching" for a solutions by stepping through the redundancy
       * (or other numerical routines).
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @return True if a valid solution was found, false otherwise
       */
       virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                     const std::vector<double> &ik_seed_state,
                                     const double &timeout,
                                     std::vector<double> &solution,
                                     const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                     const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                     int &error_code) = 0;      

      /**
       * @brief Given a set of joint angles and a set of links, compute their pose
       * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a timeout
       * @param response - the response contains stamped pose information for all the requested links
       * @return True if a valid solution was found, false otherwise
       */
       virtual bool getPositionFK(const std::vector<std::string> &link_names,
                                  const std::vector<double> &joint_angles, 
                                  std::vector<geometry_msgs::Pose> &poses) = 0;

      /**
       * @brief  Initialization function for the kinematics
       * @return True if initialization was successful, false otherwise
       */
      virtual bool initialize(std::string name) = 0;

      /**
       * @brief  Return the frame in which the kinematics is operating
       * @return the string name of the frame in which the kinematics is operating
       */
      virtual std::string getBaseFrame() = 0;

      /**
       * @brief  Return the links for which kinematics can be computed
       */
      virtual std::string getToolFrame() = 0;

      /**
       * @brief  Return all the joint names in the order they are used internally
       */
      virtual std::vector<std::string> getJointNames() = 0;

      /**
       * @brief  Return all the link names in the order they are represented internally
       */
      virtual std::vector<std::string> getLinkNames() = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~KinematicsBase(){}

    protected:
    KinematicsBase(){}
  };
};

#endif
