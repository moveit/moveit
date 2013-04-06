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

/* Author: Ioan Sucan, Sachin Chitta */

#ifndef MOVEIT_PLANNERS_OMPL_INTERFACE_ROS_
#define MOVEIT_PLANNERS_OMPL_INTERFACE_ROS_

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <ros/ros.h>

namespace ompl_interface
{
   /** @class OMPLInterfaceROS */
    class OMPLInterfaceROS : public OMPLInterface
    {
    public:
      /** @brief Constructor
       *  @param scene A pointer to the planning scene*/
      OMPLInterfaceROS(const robot_model::RobotModelConstPtr &kmodel);

      /** @brief Look up param server 'constraint_approximations' and use its value as the path to save constraint approximations to */
      bool saveConstraintApproximations();
      
      /** @brief Look up param server 'constraint_approximations' and use its value as the path to load constraint approximations to */
      bool loadConstraintApproximations();
      
      /** @brief Print the status of this node*/
      void printStatus();

    protected:
	
      /** @brief Configure everything using the param server */
      void loadParams();

      /** @brief Configure the planners*/
      void loadPlannerConfigurations();
      
      /** @brief Load the additional plugins for sampling constraints */
      void loadConstraintSamplers();
      
      ros::NodeHandle nh_; /// The ROS node handle

    private:
      
      constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
    };

}

#endif
