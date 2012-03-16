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

/* Author: Ioan Sucan */

#include "ompl_interface/constraints_library.h"
#include <boost/date_time/posix_time/posix_time.hpp>

void ompl_interface::ConstraintsLibrary::loadConstraintApproximations(const std::string &path)
{
  ROS_INFO("Loading constrained space approximations from '%s'", path.c_str());
  
  std::ifstream fin((path + "/manifest").c_str());
  if (!fin.good())
    ROS_DEBUG("Manifest not found in folder '%s'", path.c_str());
  
  while (fin.good() && !fin.eof())
  {
    std::string group, factory, serialization, filename;
    fin >> group;
    if (fin.eof())
      break;
    fin >> factory;
    if (fin.eof())
      break;
    fin >> serialization;    
    if (fin.eof())
      break;
    fin >> filename;
    const ModelBasedPlanningContextPtr &pc = getPlanningContext(group, factory);
    if (pc)
    {
      ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(pc->getOMPLSimpleSetup().getStateSpace());
      cass->load((path + "/" + filename).c_str());
      std::size_t sum = 0;
      for (std::size_t i = 0 ; i < cass->size() ; ++i)
        sum += cass->getMetadata(i).size();
      constraints_approximations_->push_back(ConstraintApproximation(kmodel_, group, factory, serialization, filename, ompl::base::StateStoragePtr(cass)));
      ROS_INFO("Loaded %lu states and %lu connections (%0.1lf per state) from %s", cass->size(), sum, (double)sum / (double)cass->size(), filename.c_str());
    }
  }
}

void ompl_interface::ConstraintsLibrary::saveConstraintApproximations(const std::string &path)
{
  ROS_INFO("Saving %u constrained space approximations to '%s'", (unsigned int)constraints_approximations_->size(), path.c_str());
  
  std::ofstream fout((path + "/manifest").c_str());
  for (std::size_t i = 0 ; i < constraints_approximations_->size() ; ++i)
  {
    fout << constraints_approximations_->at(i).group_ << std::endl;
    fout << constraints_approximations_->at(i).factory_ << std::endl;
    fout << constraints_approximations_->at(i).serialization_ << std::endl;
    fout << constraints_approximations_->at(i).ompldb_filename_ << std::endl;
    if (constraints_approximations_->at(i).state_storage_)
      constraints_approximations_->at(i).state_storage_->store((path + "/" + constraints_approximations_->at(i).ompldb_filename_).c_str());
  }
  fout.close();
}

void ompl_interface::ConstraintsLibrary::clearConstraintApproximations(void)
{
  constraints_approximations_->clear();
}

void ompl_interface::ConstraintsLibrary::printConstraintApproximations(std::ostream &out) const
{
  for (std::size_t i = 0 ; i < constraints_approximations_->size() ; ++i)
  {
    out << constraints_approximations_->at(i).group_ << std::endl;
    out << constraints_approximations_->at(i).factory_ << std::endl;
    out << constraints_approximations_->at(i).ompldb_filename_ << std::endl;
    out << constraints_approximations_->at(i).constraint_msg_ << std::endl;
  }
}

void ompl_interface::ConstraintsLibrary::addConstraintApproximation(const moveit_msgs::Constraints &constr, const std::string &group, const std::string &factory,
                                                                    const pm::KinematicState &kstate, unsigned int samples, unsigned int edges_per_sample)
{
  addConstraintApproximation(constr, constr, group, factory, kstate, samples, edges_per_sample);
}

void ompl_interface::ConstraintsLibrary::addConstraintApproximation(const moveit_msgs::Constraints &constr_sampling, const moveit_msgs::Constraints &constr_hard,
                                                                    const std::string &group, const std::string &factory,
                                                                    const pm::KinematicState &kstate, unsigned int samples, unsigned int edges_per_sample)
{ 
  addConstraintApproximation(constr_sampling, constr_hard, group, factory, kstate, ConstraintStateStorageOrderFn(), samples, edges_per_sample);
  
}

void ompl_interface::ConstraintsLibrary::addConstraintApproximation(const moveit_msgs::Constraints &constr,
                                                                    const std::string &group, const std::string &factory,
                                                                    const pm::KinematicState &kstate, const ConstraintStateStorageOrderFn &order, 
                                                                    unsigned int samples, unsigned int edges_per_sample)
{ 
  addConstraintApproximation(constr, constr, group, factory, kstate, order, samples, edges_per_sample);
}

void ompl_interface::ConstraintsLibrary::addConstraintApproximation(const moveit_msgs::Constraints &constr_sampling, const moveit_msgs::Constraints &constr_hard,
                                                                    const std::string &group, const std::string &factory,
                                                                    const pm::KinematicState &kstate, const ConstraintStateStorageOrderFn &order, 
                                                                    unsigned int samples, unsigned int edges_per_sample)
{  
  const ModelBasedPlanningContextPtr &pc = context_manager_->getPlanningContext(group, factory);
  if (pc)
  {
    ros::WallTime start = ros::WallTime::now();
    ompl::base::StateStoragePtr ss = constructConstraintApproximation(constr_sampling, constr_hard, kstate, order, samples, edges_per_sample);
    ROS_INFO("Spend %lf seconds constructing the database", (ros::WallTime::now() - start).toSec());
    if (ss)
    {
      ConstraintApproximationPtr ca(new ConstraintApproximation(kmodel_, group, factory, constr_hard, group + "_" + 
                                                                boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()) + ".ompldb", ss));
      if (ca->getName())
        named_constraints_approximations_[ca->getName()] = ca;
      else
        anonymous_constraint_approximations_.push_back(ca);
    }
    else
      ROS_ERROR("Unable to construct constraint approximation for group '%s'", group.c_str());
  }
}

ompl::base::StateStoragePtr ompl_interface::ConstraintsLibrary::constructConstraintApproximation(const moveit_msgs::Constraints &constr_sampling,
                                                                                                 const moveit_msgs::Constraints &constr_hard,
                                                                                                 const pm::KinematicState &default_state,
                                                                                                 const ConstraintStateStorageOrderFn &order,
                                                                                                 unsigned int samples, unsigned int edges_per_sample)
{
  // state storage structure
  ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(ompl_simple_setup_.getStateSpace());
  ob::StateStoragePtr sstor(cass);
  
  // construct a sampler for the sampling constraints
  kc::KinematicConstraintSet kset(getKinematicModel(), pm::TransformsConstPtr(new pm::Transforms(getKinematicModel()->getModelFrame())));
  kset.add(constr_hard);
  
  // default state
  clear();
  setStartState(default_state);
  
  int nthreads = 0;
  unsigned int attempts = 0;

  ompl_state_space_->setPlanningVolume(-20.0, 20.0, -20.0, 20.0, -20.0, 20.0);

  // construct the constrained states
#pragma omp parallel
  { 
#pragma omp master
    {
	nthreads = omp_get_num_threads();    
    }
    
    pm::KinematicState kstate(default_state);
    
    const ob::SpaceInformationPtr &si = ompl_simple_setup_.getSpaceInformation();
    kc::ConstraintSamplerPtr cs = kc::ConstraintSampler::constructFromMessage(getJointModelGroup(), constr_sampling, getKinematicModel(),
									      pm::TransformsConstPtr(new pm::Transforms(getKinematicModel()->getModelFrame())),
									      ompl_state_space_->getKinematicsAllocator(),
									      ompl_state_space_->getKinematicsSubgroupAllocators());
    ConstrainedSampler *csmp = cs ? new ConstrainedSampler(this, cs) : NULL;
    ob::StateSamplerPtr ss(csmp ? ob::StateSamplerPtr(csmp) : ompl_simple_setup_.getStateSpace()->allocDefaultStateSampler());
    
    ompl::base::ScopedState<> temp(si);
    int done = -1;
    bool slow_warn = false;
    ompl::time::point start = ompl::time::now();
    while (sstor->size() < samples)
    {
      ++attempts;
#pragma omp master
      {      
	int done_now = 100 * sstor->size() / samples;
	if (done != done_now)
	{
	  done = done_now;
	  ROS_INFO("%d%% complete (kept %0.1lf%% sampled states)", done, 100.0 * (double)sstor->size() / (double)attempts);
	}

	if (!slow_warn && attempts > 10 && attempts > sstor->size() * 100)
	{
	  slow_warn = true;
	  ROS_WARN("Computation of valid state database is very slow...");
	}
      }

      if (attempts > samples && sstor->size() == 0)
      {
	ROS_ERROR("Unable to generate any samples");
	break;
      }
      
      ss->sampleUniform(temp.get());
      ompl_state_space_->copyToKinematicState(kstate, temp.get());
      kstate.getJointStateGroup(getJointModelGroup()->getName())->updateLinkTransforms();
      double distance = 0.0;
      if (kset.decide(kstate, distance))
      {
#pragma omp critical
	{
	  if (sstor->size() < samples)
	  {
	    temp->as<ModelBasedStateSpace::StateType>()->tag = sstor->size();
	    sstor->addState(temp.get());
	  }
	}
      }
    }
#pragma omp master
    {
      ROS_INFO("Generated %u states in %lf seconds", (unsigned int)sstor->size(), ompl::time::seconds(ompl::time::now() - start)); 
      if (csmp)
	ROS_INFO("Constrained sampling rate: %lf", csmp->getConstrainedSamplingRate());
    }
  }
  if (order)
  {
    ROS_INFO("Sorting states...");
    sstor->sort(order);
  }
  
  if (edges_per_sample > 0)
  {
    ROS_INFO("Computing graph connections (max %u edges per sample) ...", edges_per_sample);
    
    ompl::tools::SelfConfig sc(ompl_simple_setup_.getSpaceInformation());
    double range = 0.0;
    sc.configurePlannerRange(range);
    
    // construct connexions
    const ob::StateSpacePtr &space = ompl_simple_setup_.getStateSpace();
    std::vector<planning_models::KinematicState> kstates(nthreads, default_state);
    const std::vector<const ompl::base::State*> &states = sstor->getStates();
    std::vector<ompl::base::ScopedState<> > temps(nthreads, ompl::base::ScopedState<>(space));
    
    ompl::time::point start = ompl::time::now();
    int good = 0;
    int done = -1;
    
#pragma omp parallel for schedule(dynamic) 
    for (std::size_t j = 0 ; j < sstor->size() ; ++j)
    {
      int threadid = omp_get_thread_num();
      pm::KinematicState &kstate = kstates[threadid];
      pm::KinematicState::JointStateGroup *jsg = kstate.getJointStateGroup(getJointModelGroup()->getName());
      ompl::base::State *temp = temps[threadid].get();
      double distance = 0.0;
      int done_now = 100 * j / sstor->size();
      if (done != done_now)
      {
	done = done_now;
	ROS_INFO("%d%% complete", done);
      }
      
      for (std::size_t i = j + 1 ; i < sstor->size() ; ++i)
      {
	double d = space->distance(states[j], states[i]);

        if (d > range * 3.0 || d < range / 100.0)
          continue;
        
        space->interpolate(states[j], states[i], 0.5, temp);
        ompl_state_space_->copyToKinematicState(kstate, temp);
        jsg->updateLinkTransforms();
        if (kset.decide(kstate, distance))
        {
	  space->interpolate(states[j], states[i], 0.25, temp);
	  ompl_state_space_->copyToKinematicState(kstate, temp);
	  jsg->updateLinkTransforms();
	  if (kset.decide(kstate, distance))
	  {
            space->interpolate(states[j], states[i], 0.75, temp);
            ompl_state_space_->copyToKinematicState(kstate, temp);
            jsg->updateLinkTransforms();
            if (kset.decide(kstate, distance))
            {
#pragma omp critical
              {
                cass->getMetadata(i).push_back(j);
                cass->getMetadata(j).push_back(i);
                good++;
              }
              if (cass->getMetadata(j).size() >= edges_per_sample)
                break;
            }
	  }
        }
      }
    }
    ROS_INFO("Computed possible connexions in %lf seconds. Added %d connexions", ompl::time::seconds(ompl::time::now() - start), good);
  }
  
  return sstor;
}
