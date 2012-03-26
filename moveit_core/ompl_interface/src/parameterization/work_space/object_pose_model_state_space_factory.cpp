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

#include "ompl_interface/parameterization/work_space/object_pose_model_state_space_factory.h"

int ompl_interface::ObjectPoseModelStateSpaceFactory::canRepresentProblem(const moveit_msgs::MotionPlanRequest &req, const pm::KinematicModelConstPtr &kmodel, const AvailableKinematicsSolvers &aks) const
{
  const pm::KinematicModel::JointModelGroup *jmg = kmodel->getJointModelGroup(req.group_name);
  if (jmg)
  {
    AvailableKinematicsSolvers::const_iterator it = aks.find(jmg);
    if (it != aks.end())
    {
      bool ik = false;
      // check that we have a direct means to compute IK
      if (it->second.first)
        ik = true;
      else
        if (!it->second.second.empty())
        {
          // or an IK solver for each of the subgroups
          unsigned int vc = 0;
          for (kc::KinematicsSubgroupAllocator::const_iterator jt = it->second.second.begin() ; jt != it->second.second.end() ; ++jt)
            vc += jt->first->getVariableCount();
          if (vc == jmg->getVariableCount())
            ik = true;
        }
      
      if (ik)
      {
        // if we have path constraints, we prefer interpolating in pose space
        if ((!req.path_constraints.position_constraints.empty() || !req.path_constraints.orientation_constraints.empty()) &&
            req.path_constraints.joint_constraints.empty() && req.path_constraints.visibility_constraints.empty())
        {
          /*
            struct Info
            {
            int pc;
            int oc;
            };
            std::map<std::string, Info> info;
            for (std::size_t i = 0 ; i < req.path_constraints.position_constraints.size() ; ++i)
            info[req.path_constraints.position_constraints.link_name].pc = i;
            for (std::size_t i = 0 ; i < req.path_constraints.orientation_constraints.size() ; ++i)
            info[req.path_constraints.orientation_constraints.link_name].oc = i;
            std::string dep;
            for (std::map<std::string, Info>::const_iterator kt = info.begin() ; kt != info.end() ; ++kt)
            if (kt->second.pc > 0)
          */
          
          return 150;
        }
      }
    }
  }
  
  return -1;
}

ompl_interface::ModelBasedStateSpacePtr ompl_interface::ObjectPoseModelStateSpaceFactory::allocStateSpace(const ModelBasedStateSpaceSpecification &space_spec) const
{
  return ModelBasedStateSpacePtr(new ObjectPoseModelStateSpace(space_spec));
}
