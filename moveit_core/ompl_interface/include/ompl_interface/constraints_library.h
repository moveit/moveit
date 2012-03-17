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

#include "ompl_interface/planning_context_manager.h"
#include <kinematic_constraints/kinematic_constraint.h>
#include <ompl/base/StateStorage.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/function.hpp>

namespace ompl_interface
{

typedef ompl::base::StateStorageWithMetadata< std::vector<std::size_t> > ConstraintApproximationStateStorage;
typedef boost::function<bool(const ompl::base::State*, const ompl::base::State*)> ConstraintStateStorageOrderFn;

class ConstraintApproximation
{
public:
  
  ConstraintApproximation(const planning_models::KinematicModelConstPtr &kinematic_model, const std::string &group, const std::string &factory,
                          const moveit_msgs::Constraints &msg, const std::string &filename, const ompl::base::StateStoragePtr &storage);
  
  virtual ~ConstraintApproximation(void)
  {
  }
  
  const std::string& getName(void) const
  {
    return constraint_msg_.name;
  }
  
  virtual ompl::base::StateSamplerAllocator getStateSamplerAllocator(const moveit_msgs::Constraints &msg) const
  {
    return state_storage_->getStateSamplerAllocator();
  }
  
  const std::vector<int> getSpaceSignature(void) const
  {
    return space_signature_;
  }
  
  const std::string& getGroup(void) const
  {
    return group_;
  }  
  
  const std::string& getFactory(void) const
  {
    return factory_;
  }
  
  const moveit_msgs::Constraints& getConstraintsMsg(void) const
  {
    return constraint_msg_;
  }
  
  const ompl::base::StateStoragePtr& getStateStorage(void) const
  {
    return state_storage_ptr_;
  }
  
  const std::string& getFilename(void) const
  {
    return ompldb_filename_;
  }  

  void visualizeDistribution(const std::string &link_name, unsigned int count, visualization_msgs::MarkerArray &arr) const;
  
protected:
  
  planning_models::KinematicModelConstPtr          kmodel_;
  std::string                                      group_;
  std::string                                      factory_;

  moveit_msgs::Constraints                         constraint_msg_;
  
  std::vector<int>                                 space_signature_;
  
  std::string                                      ompldb_filename_;
  ompl::base::StateStoragePtr                      state_storage_ptr_;
  ConstraintApproximationStateStorage             *state_storage_;
};

typedef boost::shared_ptr<ConstraintApproximation> ConstraintApproximationPtr;

class ConstraintApproximationFactory
{
public:
  ConstraintApproximationFactory(void)
  {
  }
  
  virtual ~ConstraintApproximationFactory(void)
  {
  }
  
  ConstraintStateStorageOrderFn getOrderFunction(void) const
  {
    return ConstraintStateStorageOrderFn();    
  }
  
  virtual ConstraintApproximationPtr allocApproximation(const planning_models::KinematicModelConstPtr &kinematic_model,
                                                        const std::string &group, const std::string &factory,
                                                        const moveit_msgs::Constraints &msg, const std::string &filename,
                                                        const ompl::base::StateStoragePtr &storage) const = 0;
};

typedef boost::shared_ptr<ConstraintApproximationFactory> ConstraintApproximationFactoryPtr;

template<typename C>
class SpecialConstraintApproximationFactory : public ConstraintApproximationFactory
{
public:
  
  virtual ConstraintApproximationPtr allocApproximation(const planning_models::KinematicModelConstPtr &kinematic_model,
                                                        const std::string &group, const std::string &factory,
                                                        const moveit_msgs::Constraints &msg, std::string &filename,
                                                        const ompl::base::StateStoragePtr &storage) const
  {
    return ConstraintApproximationPtr(new C(kinematic_model, group, factory, msg, filename, storage));
  }
};

class ConstraintsLibrary
{
public:
  
  ConstraintsLibrary(const PlanningContextManager &pcontext) : context_manager_(pcontext)
  {
  }

  void loadConstraintApproximations(const std::string &path);
  
  void saveConstraintApproximations(const std::string &path);
  
  void addConstraintApproximation(const moveit_msgs::Constraints &constr_sampling, const moveit_msgs::Constraints &constr_hard,
                                  const std::string &group, const std::string &factory,
                                  const pm::KinematicState &kstate, 
                                  unsigned int samples, unsigned int edges_per_sample);
  
  void addConstraintApproximation(const moveit_msgs::Constraints &constr,
                                  const std::string &group, const std::string &factory,
                                  const pm::KinematicState &kstate,
                                  unsigned int samples, unsigned int edges_per_sample);
    
  void printConstraintApproximations(std::ostream &out = std::cout) const;
  void clearConstraintApproximations(void);
  
  void registerConstraintApproximation(const ConstraintApproximationPtr &approx)
  {
    constraint_approximations_[approx->getName()] = approx;
  }
  
  template<typename C>
  void registerConstraintApproximationFactory(const std::string &name)
  {
    constraint_factories_[name] = new SpecialConstraintApproximationFactory<C>(name);
  }
  
  const ConstraintApproximationPtr& getConstraintApproximation(const moveit_msgs::Constraints &msg)
  {
    std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.find(msg.name);
    if (it != constraint_approximations_.end())
      return it->second;
    static ConstraintApproximationPtr empty;
    return empty;
  }
  
private:
  
  ompl::base::StateStoragePtr constructConstraintApproximation(const ModelBasedPlanningContextPtr &pcontext,
                                                               const moveit_msgs::Constraints &constr_sampling,
                                                               const moveit_msgs::Constraints &constr_hard,
                                                               const pm::KinematicState &default_state,
                                                               const ConstraintStateStorageOrderFn &order,
                                                               unsigned int samples, unsigned int edges_per_sample);
  
  const PlanningContextManager &context_manager_;
  std::map<std::string, ConstraintApproximationPtr> constraint_approximations_;
  std::map<std::string, ConstraintApproximationFactoryPtr> constraint_factories_;

};
}

#endif
