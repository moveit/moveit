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
#include "ompl_interface/detail/constrained_sampler.h"
#include <ompl/tools/config/SelfConfig.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

namespace ompl_interface
{
template<typename T>
void msgToHex(const T& msg, std::string &hex)
{
  static const char symbol[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  const size_t serial_size_arg = ros::serialization::serializationLength(msg);
  
  boost::shared_array<uint8_t> buffer_arg(new uint8_t[serial_size_arg]);
  ros::serialization::OStream stream_arg(buffer_arg.get(), serial_size_arg);
  ros::serialization::serialize(stream_arg, msg);
  hex.resize(serial_size_arg * 2);
  for (std::size_t i = 0 ; i < serial_size_arg ; ++i)
  {
    hex[i * 2] = symbol[buffer_arg[i]/16];
    hex[i * 2 + 1] = symbol[buffer_arg[i]%16];
  }
}

template<typename T>
void hexToMsg(const std::string &hex, T& msg)
{
  const size_t serial_size_arg = hex.length() / 2;
  boost::shared_array<uint8_t> buffer_arg(new uint8_t[serial_size_arg]);
  for (std::size_t i = 0 ; i < serial_size_arg ; ++i)
  {
    buffer_arg[i] =
      (hex[i * 2] <= '9' ? (hex[i * 2] - '0') : (hex[i * 2] - 'A' + 10)) * 16 +
      (hex[i * 2 + 1] <= '9' ? (hex[i * 2 + 1] - '0') : (hex[i * 2 + 1] - 'A' + 10));
  }
  ros::serialization::IStream stream_arg(buffer_arg.get(), serial_size_arg);
  ros::serialization::deserialize(stream_arg, msg);
}

class ConstraintApproximationStateSampler : public ob::StateSampler
{
public:
  
  ConstraintApproximationStateSampler(const ob::StateSpace *space, const ConstraintApproximationStateStorage *state_storage) : 
    ob::StateSampler(space), state_storage_(state_storage)
  {
    min_index_ = 0;
    max_index_ = state_storage_->size() - 1;
  }
  
  ConstraintApproximationStateSampler(const ob::StateSpace *space, const ConstraintApproximationStateStorage *state_storage,
                                      int mini, int maxi) : 
    ob::StateSampler(space), state_storage_(state_storage)
  {
    min_index_ = mini;
    max_index_ = maxi;
  }
  
  virtual void sampleUniform(ob::State *state)
  { 
    space_->copyState(state, state_storage_->getState(rng_.uniformInt(min_index_, max_index_)));
  }
  
  virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
  {
    int index = -1;
    int tag = near->as<ModelBasedStateSpace::StateType>()->tag;
    if (tag >= 0)
    {
      const std::vector<std::size_t> &md = state_storage_->getMetadata(tag);
      if (!md.empty() && rng_.uniform01() * md.size() > 1.0)
      {
        index = md[rng_.uniformInt(0, md.size() - 1)];
      }
    }
    if (index < 0) 
    {
      index = rng_.uniformInt(min_index_, max_index_);
    }
    
    double dist = space_->distance(near, state_storage_->getState(index));
    if (dist > distance)
    {
      double d = rng_.uniformReal(0.0, distance);
      space_->interpolate(near, state_storage_->getState(index), d / dist, state);
    }
    else
      space_->copyState(state, state_storage_->getState(index));
  }
  
  virtual void sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev)
  {
    sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
  }
  
protected:
  
  /** \brief The states to sample from */
  const ConstraintApproximationStateStorage *state_storage_;  
  unsigned int min_index_;
  unsigned int max_index_;
  
};

}

ompl_interface::ConstraintApproximation::ConstraintApproximation(const planning_models::KinematicModelConstPtr &kinematic_model,
                                                                 const std::string &group, const std::string &factory,
                                                                 const moveit_msgs::Constraints &msg, const std::string &filename,
                                                                 const ompl::base::StateStoragePtr &storage) :
  kmodel_(kinematic_model), group_(group), factory_(factory), constraint_msg_(msg),
  ompldb_filename_(filename), state_storage_ptr_(storage)
{
  state_storage_ = static_cast<ConstraintApproximationStateStorage*>(state_storage_ptr_.get());
  state_storage_->getStateSpace()->computeSignature(space_signature_);
}

void ompl_interface::ConstraintApproximation::visualizeDistribution(const std::string &link_name, unsigned int count, visualization_msgs::MarkerArray &arr) const
{
  planning_models::KinematicState kstate(kmodel_);
  kstate.setToDefaultValues();
  
  ompl::RNG rng;
  std_msgs::ColorRGBA color;
  color.r = 0.0f;
  color.g = 1.0f;
  color.b = 1.0f;
  color.a = 1.0f;
  if (state_storage_->size() < count)
    count = state_storage_->size();
  
  for (std::size_t i = 0 ; i < count ; ++i)
  {
    state_storage_->getStateSpace()->as<ModelBasedStateSpace>()->copyToKinematicState(kstate, state_storage_->getState(rng.uniformInt(0, state_storage_->size() - 1)));
    kstate.getJointStateGroup(group_)->updateLinkTransforms();
    const Eigen::Vector3d &pos = kstate.getLinkState(link_name)->getGlobalLinkTransform().translation();
    
    visualization_msgs::Marker mk;
    mk.header.stamp = ros::Time::now();
    mk.header.frame_id = kmodel_->getModelFrame();
    mk.ns = "stored_constraint_data";
    mk.id = i;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = pos.x();
    mk.pose.position.y = pos.y();
    mk.pose.position.z = pos.z();
    mk.pose.orientation.w = 1.0;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.035;
    mk.color = color;
    mk.lifetime = ros::Duration(30.0);
    arr.markers.push_back(mk);
  }
}

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
    const ModelBasedPlanningContextPtr &pc = context_manager_.getPlanningContext(group, factory);
    if (pc)
    {
      moveit_msgs::Constraints msg;
      hexToMsg(serialization, msg);
      ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(pc->getOMPLSimpleSetup().getStateSpace());
      cass->load((path + "/" + filename).c_str());
      ConstraintApproximationPtr cap;
      if (constraint_factories_.find(msg.name) != constraint_factories_.end())
        cap = constraint_factories_[msg.name]->allocApproximation(context_manager_.getKinematicModel(),
                                                                  group, factory, msg, filename, ompl::base::StateStoragePtr(cass));
      else
        cap.reset(new ConstraintApproximation(context_manager_.getKinematicModel(),
                                              group, factory, msg, filename, ompl::base::StateStoragePtr(cass)));
      if (cap)
      {
        constraint_approximations_[cap->getName()] = cap;
        std::size_t sum = 0;
        for (std::size_t i = 0 ; i < cass->size() ; ++i)
          sum += cass->getMetadata(i).size();
        ROS_INFO("Loaded %lu states and %lu connections (%0.1lf per state) from %s", cass->size(), sum, (double)sum / (double)cass->size(), filename.c_str());
      }
    }
  }
}

void ompl_interface::ConstraintsLibrary::saveConstraintApproximations(const std::string &path)
{
  ROS_INFO("Saving %u constrained space approximations to '%s'", (unsigned int)constraint_approximations_.size(), path.c_str());
  
  std::ofstream fout((path + "/manifest").c_str());
  for (std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.begin() ; it != constraint_approximations_.end() ; ++it)
  {
    fout << it->second->getGroup() << std::endl;
    fout << it->second->getFactory() << std::endl;
    std::string serialization;
    msgToHex(it->second->getConstraintsMsg(), serialization);
    fout << serialization << std::endl;
    fout << it->second->getFilename() << std::endl;
    if (it->second->getStateStorage())
      it->second->getStateStorage()->store((path + "/" + it->second->getFilename()).c_str());
  }
  fout.close();
}

void ompl_interface::ConstraintsLibrary::clearConstraintApproximations(void)
{
  constraint_approximations_.clear();
}

void ompl_interface::ConstraintsLibrary::printConstraintApproximations(std::ostream &out) const
{

  for (std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.begin() ; it != constraint_approximations_.end() ; ++it)
  {
    out << it->second->getGroup() << std::endl;
    out << it->second->getFactory() << std::endl;
    out << it->second->getFilename() << std::endl;
    out << it->second->getConstraintsMsg() << std::endl;
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

  const ModelBasedPlanningContextPtr &pc = context_manager_.getPlanningContext(group, factory);
  if (pc)
  {
    std::map<std::string, ConstraintApproximationFactoryPtr>::const_iterator it = constraint_factories_.find(constr_hard.name);
    ConstraintApproximationFactory *fct = NULL;
    ConstraintStateStorageOrderFn order;
    if (it != constraint_factories_.end())
    {
      fct = it->second.get();
      order = fct->getOrderFunction();
    }
    
    ros::WallTime start = ros::WallTime::now();
    ompl::base::StateStoragePtr ss = constructConstraintApproximation(pc, constr_sampling, constr_hard, kstate, order, samples, edges_per_sample);
    ROS_INFO("Spend %lf seconds constructing the database", (ros::WallTime::now() - start).toSec());
    if (ss)
    {
      ConstraintApproximationPtr ca;
      if (fct)
        ca = fct->allocApproximation(context_manager_.getKinematicModel(), group, factory, constr_hard, group + "_" + 
                                     boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()) + ".ompldb", ss);
      else
        ca.reset(new ConstraintApproximation(context_manager_.getKinematicModel(), group, factory, constr_hard, group + "_" + 
                                             boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()) + ".ompldb", ss));
      if (ca)
        constraint_approximations_[ca->getName()] = ca;
    }
    else
      ROS_ERROR("Unable to construct constraint approximation for group '%s'", group.c_str());
  }
}

ompl::base::StateStoragePtr ompl_interface::ConstraintsLibrary::constructConstraintApproximation(const ModelBasedPlanningContextPtr &pcontext,
                                                                                                 const moveit_msgs::Constraints &constr_sampling,
                                                                                                 const moveit_msgs::Constraints &constr_hard,
                                                                                                 const pm::KinematicState &default_state,
                                                                                                 const ConstraintStateStorageOrderFn &order,
                                                                                                 unsigned int samples, unsigned int edges_per_sample)
{
  // state storage structure
  ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(pcontext->getOMPLStateSpace());
  ob::StateStoragePtr sstor(cass);
  
  // construct a sampler for the sampling constraints
  kc::KinematicConstraintSet kset(pcontext->getKinematicModel(), pm::TransformsConstPtr(new pm::Transforms(pcontext->getKinematicModel()->getModelFrame())));
  kset.add(constr_hard);
  
  // default state
  pcontext->clear();
  pcontext->setStartState(default_state);
  
  int nthreads = 0;
  unsigned int attempts = 0;
  
  double large_val = 1000000;
  pcontext->getOMPLStateSpace()->setPlanningVolume(-large_val, large_val, -large_val, large_val, -large_val, large_val);
  
  // construct the constrained states
#pragma omp parallel
  { 
#pragma omp master
    {
	nthreads = omp_get_num_threads();    
    }
    
    pm::KinematicState kstate(default_state);
    
    kc::ConstraintSamplerPtr cs = kc::ConstraintSampler::constructFromMessage(pcontext->getJointModelGroup(), constr_sampling, pcontext->getKinematicModel(),
									      pm::TransformsConstPtr(new pm::Transforms(pcontext->getKinematicModel()->getModelFrame())),
									      pcontext->getOMPLStateSpace()->getKinematicsAllocator(),
									      pcontext->getOMPLStateSpace()->getKinematicsSubgroupAllocators());
    ConstrainedSampler *csmp = cs ? new ConstrainedSampler(pcontext.get(), cs) : NULL;
    ob::StateSamplerPtr ss(csmp ? ob::StateSamplerPtr(csmp) : pcontext->getOMPLStateSpace()->allocDefaultStateSampler());
    
    ompl::base::ScopedState<> temp(pcontext->getOMPLStateSpace());
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
      pcontext->getOMPLStateSpace()->copyToKinematicState(kstate, temp.get());
      kstate.getJointStateGroup(pcontext->getJointModelGroup()->getName())->updateLinkTransforms();
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
    
    ompl::tools::SelfConfig sc(pcontext->getOMPLSimpleSetup().getSpaceInformation());
    double range = 0.0;
    sc.configurePlannerRange(range);
    
    // construct connexions
    const ob::StateSpacePtr &space = pcontext->getOMPLSimpleSetup().getStateSpace();
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
      pm::KinematicState::JointStateGroup *jsg = kstate.getJointStateGroup(pcontext->getJointModelGroup()->getName());
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
        pcontext->getOMPLStateSpace()->copyToKinematicState(kstate, temp);
        jsg->updateLinkTransforms();
        if (kset.decide(kstate, distance))
        {
	  space->interpolate(states[j], states[i], 0.25, temp);
	  pcontext->getOMPLStateSpace()->copyToKinematicState(kstate, temp);
	  jsg->updateLinkTransforms();
	  if (kset.decide(kstate, distance))
	  {
            space->interpolate(states[j], states[i], 0.75, temp);
            pcontext->getOMPLStateSpace()->copyToKinematicState(kstate, temp);
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
