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

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_STATE_SPACE_FACTORY_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_MODEL_BASED_STATE_SPACE_FACTORY_

#include "ompl_interface/parameterization/model_based_state_space.h"
#include <moveit_msgs/MotionPlanRequest.h>

namespace ompl_interface
{

class ModelBasedStateSpaceFactory;
typedef boost::shared_ptr<ModelBasedStateSpaceFactory> ModelBasedStateSpaceFactoryPtr;

typedef std::map<const pm::KinematicModel::JointModelGroup*, std::pair<kc::KinematicsAllocator, kc::KinematicsSubgroupAllocator> > AvailableKinematicsSolvers;

class ModelBasedStateSpaceFactory
{
public:
  
  ModelBasedStateSpaceFactory(void)
  {
  }
  
  virtual ~ModelBasedStateSpaceFactory(void)
  {
  }
  
  ModelBasedStateSpacePtr getNewStateSpace(const ModelBasedStateSpaceSpecification &space_spec) const;

  const std::string& getType(void) const
  {
    return type_;
  }

  virtual int canRepresentProblem(const moveit_msgs::MotionPlanRequest &req,
				  const pm::KinematicModelConstPtr &kmodel,
				  const AvailableKinematicsSolvers &aks) const = 0;

protected:
  
  virtual ModelBasedStateSpacePtr allocStateSpace(const ModelBasedStateSpaceSpecification &space_spec) const = 0;
  std::string type_;
};

}

#endif
