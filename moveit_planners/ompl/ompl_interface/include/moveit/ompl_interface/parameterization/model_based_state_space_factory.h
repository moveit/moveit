/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_STATE_SPACE_FACTORY_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_STATE_SPACE_FACTORY_

#include <moveit/macros/class_forward.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit_msgs/MotionPlanRequest.h>

namespace ompl_interface
{
MOVEIT_CLASS_FORWARD(ModelBasedStateSpaceFactory);

class ModelBasedStateSpaceFactory
{
public:
  ModelBasedStateSpaceFactory()
  {
  }

  virtual ~ModelBasedStateSpaceFactory()
  {
  }

  ModelBasedStateSpacePtr getNewStateSpace(const ModelBasedStateSpaceSpecification& space_spec) const;

  const std::string& getType() const
  {
    return type_;
  }

  /** \brief Decide whether the type of state space constructed by this factory could represent problems specified by
     the user
      request \e req for group \e group. The group \e group must always be specified and takes precedence over \e
     req.group_name, which may be different */
  virtual int canRepresentProblem(const std::string& group, const moveit_msgs::MotionPlanRequest& req,
                                  const robot_model::RobotModelConstPtr& kmodel) const = 0;

protected:
  virtual ModelBasedStateSpacePtr allocStateSpace(const ModelBasedStateSpaceSpecification& space_spec) const = 0;
  std::string type_;
};
}

#endif
