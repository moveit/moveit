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

#ifndef MOVEIT_OMPL_INTERFACE_CONSTRAINTS_LIBRARY_
#define MOVEIT_OMPL_INTERFACE_CONSTRAINTS_LIBRARY_

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <ompl/base/StateStorage.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/function.hpp>

namespace ompl_interface
{

typedef ompl::base::StateStorageWithMetadata< std::vector<std::size_t> > ConstraintApproximationStateStorage;
typedef boost::function<bool(const ompl::base::State*, const ompl::base::State*)> ConstraintStateStorageOrderFn;
typedef boost::function<bool(const ompl::base::State*)> ConstraintStateStorageDelimiterFn;

class ConstraintApproximation;
typedef boost::shared_ptr<ConstraintApproximation> ConstraintApproximationPtr;
typedef boost::shared_ptr<const ConstraintApproximation> ConstraintApproximationConstPtr;

class ConstraintApproximationFactory;
typedef boost::shared_ptr<ConstraintApproximationFactory> ConstraintApproximationFactoryPtr;
typedef boost::shared_ptr<const ConstraintApproximationFactory> ConstraintApproximationFactoryConstPtr;

class ConstraintApproximation
{
public:
  
  ConstraintApproximation(const robot_model::RobotModelConstPtr &kinematic_model, const std::string &group, const std::string &state_space_parameterization,
                          const moveit_msgs::Constraints &msg, const std::string &filename, const ompl::base::StateStoragePtr &storage, 
                          const ConstraintApproximationFactory *parent_factory = NULL);
  
  virtual ~ConstraintApproximation()
  {
  }
  
  const std::string& getName() const
  {
    return constraint_msg_.name;
  }
  
  virtual ompl::base::StateSamplerAllocator getStateSamplerAllocator(const moveit_msgs::Constraints &msg) const;
  
  const std::vector<int>& getSpaceSignature() const
  {
    return space_signature_;
  }
  
  const std::string& getGroup() const
  {
    return group_;
  }  
  
  const std::string& getStateSpaceParameterization() const
  {
    return state_space_parameterization_;
  }
  
  const moveit_msgs::Constraints& getConstraintsMsg() const
  {
    return constraint_msg_;
  }
  
  const ompl::base::StateStoragePtr& getStateStorage() const
  {
    return state_storage_ptr_;
  }
  
  const std::string& getFilename() const
  {
    return ompldb_filename_;
  }  

  void visualizeDistribution(const std::string &link_name, unsigned int count, visualization_msgs::MarkerArray &arr) const;
  
protected:
  
  robot_model::RobotModelConstPtr          kmodel_;
  std::string                                      group_;
  std::string                                      state_space_parameterization_;

  moveit_msgs::Constraints                         constraint_msg_;
  
  std::vector<int>                                 space_signature_;
  
  std::string                                      ompldb_filename_;
  ompl::base::StateStoragePtr                      state_storage_ptr_;
  ConstraintApproximationStateStorage             *state_storage_;
  
  const ConstraintApproximationFactory            *parent_factory_;
};

class ConstraintApproximationFactory
{
public:
  ConstraintApproximationFactory()
  {
  }
  
  virtual ~ConstraintApproximationFactory()
  {
  }
  
  virtual ConstraintStateStorageOrderFn getOrderFunction() const
  {
    return ConstraintStateStorageOrderFn();    
  }

  virtual ConstraintStateStorageDelimiterFn getAboveDelimiterFunction(const moveit_msgs::Constraints &msg) const
  {
    return ConstraintStateStorageDelimiterFn();
  }
  
  virtual ConstraintStateStorageDelimiterFn getBelowDelimiterFunction(const moveit_msgs::Constraints &msg) const
  {
    return ConstraintStateStorageDelimiterFn();
  }  
  
  virtual ConstraintApproximationPtr allocApproximation(const robot_model::RobotModelConstPtr &kinematic_model,
                                                        const std::string &group, const std::string &state_space_parameterization,
                                                        const moveit_msgs::Constraints &msg, const std::string &filename,
                                                        const ompl::base::StateStoragePtr &storage) const = 0;
};

template<typename C>
class SpecialConstraintApproximationFactory : public ConstraintApproximationFactory
{
public:
  
  virtual ConstraintApproximationPtr allocApproximation(const robot_model::RobotModelConstPtr &kinematic_model,
                                                        const std::string &group, const std::string &state_space_parameterization,
                                                        const moveit_msgs::Constraints &msg, std::string &filename,
                                                        const ompl::base::StateStoragePtr &storage) const
  {
    return ConstraintApproximationPtr(new C(kinematic_model, group, state_space_parameterization, msg, filename, storage, this));
  }
};

struct ConstraintApproximationConstructionResults
{
  ConstraintApproximationPtr approx;
  double                     state_sampling_time;
  double                     state_connection_time;
  double                     sampling_success_rate;
};

class ConstraintsLibrary
{
public:
  
  ConstraintsLibrary(const PlanningContextManager &pcontext) : context_manager_(pcontext)
  {
  }

  void loadConstraintApproximations(const std::string &path);
  
  void saveConstraintApproximations(const std::string &path);
  
  ConstraintApproximationConstructionResults
  addConstraintApproximation(const moveit_msgs::Constraints &constr_sampling, const moveit_msgs::Constraints &constr_hard,
                             const std::string &group, const std::string &state_space_parameterization,
                             const planning_scene::PlanningSceneConstPtr &scene, 
                             unsigned int samples, unsigned int edges_per_sample);
  
  ConstraintApproximationConstructionResults
  addConstraintApproximation(const moveit_msgs::Constraints &constr,
                             const std::string &group, const std::string &state_space_parameterization,
                             const planning_scene::PlanningSceneConstPtr &scene, 
                             unsigned int samples, unsigned int edges_per_sample);
  
  void printConstraintApproximations(std::ostream &out = std::cout) const;
  void clearConstraintApproximations();
  
  void registerConstraintApproximation(const ConstraintApproximationPtr &approx)
  {
    constraint_approximations_[approx->getName()] = approx;
  }
  
  template<typename C>
  void registerConstraintApproximationFactory(const std::string &name)
  {
    constraint_factories_[name] = new SpecialConstraintApproximationFactory<C>(name);
  }
  
  const ConstraintApproximationPtr& getConstraintApproximation(const moveit_msgs::Constraints &msg) const;
  
private:
  
  ompl::base::StateStoragePtr constructConstraintApproximation(const ModelBasedPlanningContextPtr &pcontext,
                                                               const moveit_msgs::Constraints &constr_sampling,
                                                               const moveit_msgs::Constraints &constr_hard,
                                                               const ConstraintStateStorageOrderFn &order,
                                                               unsigned int samples, unsigned int edges_per_sample,
                                                               ConstraintApproximationConstructionResults &result);
  
  const PlanningContextManager &context_manager_;
  std::map<std::string, ConstraintApproximationPtr> constraint_approximations_;
  std::map<std::string, ConstraintApproximationFactoryPtr> constraint_factories_;

};

typedef boost::shared_ptr<ConstraintsLibrary> ConstraintsLibraryPtr;
typedef boost::shared_ptr<const ConstraintsLibrary> ConstraintsLibraryConstPtr;

}

#endif
