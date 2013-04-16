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

#ifndef MOVEIT_OMPL_INTERFACE_OMPL_INTERFACE_
#define MOVEIT_OMPL_INTERFACE_OMPL_INTERFACE_

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/ompl_interface/constraints_library.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <string>
#include <map>

/** \brief The MoveIt interface to OMPL */
namespace ompl_interface
{

/** @class OMPLInterface
 *  This class defines the interface to the motion planners in OMPL*/
class OMPLInterface
{
public: 
  
  OMPLInterface(const robot_model::RobotModelConstPtr &kmodel);
  virtual ~OMPLInterface();
  
  /** @brief Specify configurations for the planners.
      @param pconfig Configurations for the different planners */
  void setPlanningConfigurations(const std::vector<PlanningConfigurationSettings> &pconfig)
  {
    context_manager_.setPlanningConfigurations(pconfig);
  }
  
  /** @brief Solve the planning problem */
  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest &req, planning_interface::MotionPlanResponse &res) const;

  /** @brief Solve the planning problem but give a more detailed response */
  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest &req, planning_interface::MotionPlanDetailedResponse &res) const;
  
  /** @brief Benchmark the planning problem*/
  /*
  bool benchmark(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::BenchmarkPluginRequest &req,                  
                 moveit_msgs::BenchmarkPluginResponse &res) const;
  */
  
  void terminateSolve();

  ModelBasedPlanningContextPtr getPlanningContext(const planning_interface::MotionPlanRequest &req) const;
  ModelBasedPlanningContextPtr getPlanningContext(const std::string &config, const std::string &factory_type = "") const;

  ModelBasedPlanningContextPtr getLastPlanningContext() const
  {
    return context_manager_.getLastPlanningContext();    
  }
  
  const PlanningContextManager& getPlanningContextManager() const
  {
    return context_manager_;
  }
  
  PlanningContextManager& getPlanningContextManager()
  {
    return context_manager_;
  }

  ConstraintsLibrary& getConstraintsLibrary()
  {
    return *constraints_library_;
  }

  const ConstraintsLibrary& getConstraintsLibrary() const
  {
    return *constraints_library_;
  }

  constraint_samplers::ConstraintSamplerManager& getConstraintSamplerManager()
  {
    return *constraint_sampler_manager_;
  }
  
  void useConstraintsApproximations(bool flag)
  {
    use_constraints_approximations_ = flag;
  }
  
  bool isUsingConstraintsApproximations() const
  {
    return use_constraints_approximations_;
  }

  void loadConstraintApproximations(const std::string &path)
  {
    constraints_library_->loadConstraintApproximations(path);
  }

  void saveConstraintApproximations(const std::string &path)
  {
    constraints_library_->saveConstraintApproximations(path);
  }
  
protected:

  void configureConstraints(const ModelBasedPlanningContextPtr &context) const;
  
  /** \brief Configure the OMPL planning context for a new planning request */
  ModelBasedPlanningContextPtr prepareForSolve(const planning_interface::MotionPlanRequest &req,
                                               const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                               moveit_msgs::MoveItErrorCodes *error_code,
                                               unsigned int *attempts, double *timeout) const;
  
  /** \brief The kinematic model for which motion plans are computed */
  robot_model::RobotModelConstPtr kmodel_;
  
  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;
  
  PlanningContextManager context_manager_;
  
  ConstraintsLibraryPtr constraints_library_;
  
  bool use_constraints_approximations_;
  
};

}


#endif
