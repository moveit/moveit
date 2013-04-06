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

#include <moveit/ompl_interface/constraints_library.h>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace ompl_interface
{

namespace
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
}

class ConstraintApproximationStateSampler : public ob::StateSampler
{
public:
  
  ConstraintApproximationStateSampler(const ob::StateSpace *space, const ConstraintApproximationStateStorage *state_storage) : 
    ob::StateSampler(space), state_storage_(state_storage)
  {
    min_index_ = 0;
    max_index_ = state_storage_->size() - 1;
    inv_dim_ = space->getDimension() > 0 ? 1.0 / (double)space->getDimension() : 1.0;
  }
  
  ConstraintApproximationStateSampler(const ob::StateSpace *space, const ConstraintApproximationStateStorage *state_storage,
                                      std::size_t mini, std::size_t maxi) : 
    ob::StateSampler(space), state_storage_(state_storage)
  {
    min_index_ = mini;
    max_index_ = maxi;  
    inv_dim_ = space->getDimension() > 0 ? 1.0 / (double)space->getDimension() : 1.0;
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
      double d = pow(rng_.uniform01(), inv_dim_) * distance;
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
  double inv_dim_;
};

ompl::base::StateSamplerPtr allocConstraintApproximationStateSampler(const ob::StateSpace *space, const std::vector<int> &expected_signature,
                                                                     const ConstraintApproximationStateStorage *state_storage, std::size_t mini, std::size_t maxi)
{
  std::vector<int> sig;
  space->computeSignature(sig);
  if (sig != expected_signature)
    return ompl::base::StateSamplerPtr();
  else
    return ompl::base::StateSamplerPtr(new ConstraintApproximationStateSampler(space, state_storage, mini, maxi));
}

}

ompl_interface::ConstraintApproximation::ConstraintApproximation(const robot_model::RobotModelConstPtr &kinematic_model,
                                                                 const std::string &group, const std::string &state_space_parameterization,
                                                                 const moveit_msgs::Constraints &msg, const std::string &filename,
                                                                 const ompl::base::StateStoragePtr &storage,
                                                                 const ConstraintApproximationFactory *parent_factory) :
  kmodel_(kinematic_model), group_(group), state_space_parameterization_(state_space_parameterization), constraint_msg_(msg),
  ompldb_filename_(filename), state_storage_ptr_(storage), parent_factory_(parent_factory)
{
  state_storage_ = static_cast<ConstraintApproximationStateStorage*>(state_storage_ptr_.get());
  state_storage_->getStateSpace()->computeSignature(space_signature_);
}

ompl::base::StateSamplerAllocator ompl_interface::ConstraintApproximation::getStateSamplerAllocator(const moveit_msgs::Constraints &msg) const
{
  if (state_storage_->size() == 0)
    return ompl::base::StateSamplerAllocator();
  std::size_t mini = 0;
  std::size_t maxi = state_storage_->size() - 1;
  if (parent_factory_)
  {
    ConstraintStateStorageDelimiterFn above = parent_factory_->getAboveDelimiterFunction(msg);
    ConstraintStateStorageDelimiterFn below = parent_factory_->getBelowDelimiterFunction(msg);

    // figure out mini & maxi
    if (above)
    {
      std::size_t rangeStart = mini;
      std::size_t rangeEnd = maxi;
      
      while (rangeStart < rangeEnd)
      {
        std::size_t mid = (rangeStart + rangeEnd) / 2;
        if (above(state_storage_->getState(mid)))
          rangeStart = mid + 1;
        else
          rangeEnd = mid;
      }
      if (rangeEnd < maxi)
        mini = rangeEnd + 1;
      else
        mini = maxi;
    }
    
    if (below)
    {	
      std::size_t rangeStart = mini;
      std::size_t rangeEnd = maxi;
      
      while (rangeStart < rangeEnd)
      {
        std::size_t mid = (rangeStart + rangeEnd) / 2;
        if (below(state_storage_->getState(mid)))
        {
          if (mid == 0)
            rangeEnd = 0;
          else
            rangeEnd = mid - 1;
        }
        else
          rangeStart = mid;
      }
      if (rangeStart > 0)
        maxi = rangeStart - 1;
      else
        maxi = 0;
    }
    
    if (mini > maxi)
    {
      logDebug("Empty sampling range found for constraint: %u, %u", (unsigned int)mini, (unsigned int)maxi);
      return ompl::base::StateSamplerAllocator();
    }
    
    if ((above && above(state_storage_->getState(mini))) || (below && below(state_storage_->getState(maxi))))
    {
      logDebug("Empty sampling range found for constraint: %u, %u", (unsigned int)mini, (unsigned int)maxi);
      return ompl::base::StateSamplerAllocator();
    }
    
    logDebug("Stored states sampling range is [%u, %u]", (unsigned int)mini, (unsigned int)maxi);
  }
  return boost::bind(&allocConstraintApproximationStateSampler, _1, space_signature_, state_storage_, mini, maxi);
}

void ompl_interface::ConstraintApproximation::visualizeDistribution(const std::string &link_name, unsigned int count, visualization_msgs::MarkerArray &arr) const
{
  robot_state::RobotState kstate(kmodel_);
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
    state_storage_->getStateSpace()->as<ModelBasedStateSpace>()->copyToRobotState(kstate, state_storage_->getState(rng.uniformInt(0, state_storage_->size() - 1)));
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
  constraint_approximations_.clear();
  std::ifstream fin((path + "/manifest").c_str());
  if (!fin.good())
  {
    logDebug("Manifest not found in folder '%s'. Not loading constraint approximations.", path.c_str());
    return;
  }
  
  logInform("Loading constrained space approximations from '%s'", path.c_str());
  
  while (fin.good() && !fin.eof())
  {
    std::string group, state_space_parameterization, serialization, filename;
    fin >> group;
    if (fin.eof())
      break;
    fin >> state_space_parameterization;
    if (fin.eof())
      break;
    fin >> serialization;    
    if (fin.eof())
      break;
    fin >> filename;
    const ModelBasedPlanningContextPtr &pc = context_manager_.getPlanningContext(group, state_space_parameterization);
    if (pc)
    {
      moveit_msgs::Constraints msg;
      hexToMsg(serialization, msg);
      ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(pc->getOMPLSimpleSetup().getStateSpace());
      cass->load((path + "/" + filename).c_str());
      ConstraintApproximationPtr cap;
      if (constraint_factories_.find(msg.name) != constraint_factories_.end())
        cap = constraint_factories_[msg.name]->allocApproximation(context_manager_.getRobotModel(),
                                                                  group, state_space_parameterization, msg, filename, ompl::base::StateStoragePtr(cass));
      else
        cap.reset(new ConstraintApproximation(context_manager_.getRobotModel(),
                                              group, state_space_parameterization, msg, filename, ompl::base::StateStoragePtr(cass)));
      if (cap)
      {
        if (constraint_approximations_.find(cap->getName()) != constraint_approximations_.end())
          logWarn("Overwriting constraint approximation named '%s'", cap->getName().c_str());
        
        constraint_approximations_[cap->getName()] = cap;
        std::size_t sum = 0;
        for (std::size_t i = 0 ; i < cass->size() ; ++i)
          sum += cass->getMetadata(i).size();
        logInform("Loaded %lu states and %lu connections (%0.1lf per state) from %s", cass->size(), sum, (double)sum / (double)cass->size(), filename.c_str());
      }
    }
  }
}

void ompl_interface::ConstraintsLibrary::saveConstraintApproximations(const std::string &path)
{
  logInform("Saving %u constrained space approximations to '%s'", (unsigned int)constraint_approximations_.size(), path.c_str());
  try
  {
    boost::filesystem::create_directories(path);
  }
  catch(...)
  {
  }
  
  std::ofstream fout((path + "/manifest").c_str());
  if (fout.good())
    for (std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.begin() ; it != constraint_approximations_.end() ; ++it)
    {
      fout << it->second->getGroup() << std::endl;
      fout << it->second->getStateSpaceParameterization() << std::endl;
      std::string serialization;
      msgToHex(it->second->getConstraintsMsg(), serialization);
      fout << serialization << std::endl;
      fout << it->second->getFilename() << std::endl;
      if (it->second->getStateStorage())
        it->second->getStateStorage()->store((path + "/" + it->second->getFilename()).c_str());
    }
  else
    logError("Unable to save constraint approximation to '%s'", path.c_str());
  fout.close();
}

void ompl_interface::ConstraintsLibrary::clearConstraintApproximations()
{
  constraint_approximations_.clear();
}

void ompl_interface::ConstraintsLibrary::printConstraintApproximations(std::ostream &out) const
{
  for (std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.begin() ; it != constraint_approximations_.end() ; ++it)
  {
    out << it->second->getGroup() << std::endl;
    out << it->second->getStateSpaceParameterization() << std::endl;
    out << it->second->getFilename() << std::endl;
    out << it->second->getConstraintsMsg() << std::endl;
  }
}

const ompl_interface::ConstraintApproximationPtr& ompl_interface::ConstraintsLibrary::getConstraintApproximation(const moveit_msgs::Constraints &msg) const
{
  std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.find(msg.name);
  if (it != constraint_approximations_.end())
    return it->second;

  static ConstraintApproximationPtr empty;
  return empty;
}

ompl_interface::ConstraintApproximationConstructionResults
ompl_interface::ConstraintsLibrary::addConstraintApproximation(const moveit_msgs::Constraints &constr, const std::string &group, const std::string &state_space_parameterization,
                                                               const planning_scene::PlanningSceneConstPtr &scene, unsigned int samples, unsigned int edges_per_sample)
{
  return addConstraintApproximation(constr, constr, group, state_space_parameterization, scene, samples, edges_per_sample);
}

ompl_interface::ConstraintApproximationConstructionResults
ompl_interface::ConstraintsLibrary::addConstraintApproximation(const moveit_msgs::Constraints &constr_sampling, const moveit_msgs::Constraints &constr_hard,
                                                               const std::string &group, const std::string &state_space_parameterization,
                                                               const planning_scene::PlanningSceneConstPtr &scene, unsigned int samples, unsigned int edges_per_sample)
{ 
  ConstraintApproximationConstructionResults res;
  ModelBasedPlanningContextPtr pc = context_manager_.getPlanningContext(group, state_space_parameterization);
  if (pc)
  {                                             
    pc->clear();
    pc->setPlanningScene(scene);
    pc->setCompleteInitialState(scene->getCurrentState());

    std::map<std::string, ConstraintApproximationFactoryPtr>::const_iterator it = constraint_factories_.find(constr_hard.name);
    ConstraintApproximationFactory *fct = NULL;
    ConstraintStateStorageOrderFn order;
    if (it != constraint_factories_.end())
    {
      fct = it->second.get();
      order = fct->getOrderFunction();
    }
    
    ros::WallTime start = ros::WallTime::now();
    ompl::base::StateStoragePtr ss = constructConstraintApproximation(pc, constr_sampling, constr_hard, order, samples, edges_per_sample, res);
    logInform("Spent %lf seconds constructing the database", (ros::WallTime::now() - start).toSec());
    if (ss)
    {
      ConstraintApproximationPtr ca;
      if (fct)
        ca = fct->allocApproximation(context_manager_.getRobotModel(), group, state_space_parameterization, constr_hard, group + "_" + 
                                     boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()) + ".ompldb", ss);
      else
        ca.reset(new ConstraintApproximation(context_manager_.getRobotModel(), group, state_space_parameterization, constr_hard, group + "_" + 
                                             boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()) + ".ompldb", ss));
      if (ca)
      {
        if (constraint_approximations_.find(ca->getName()) != constraint_approximations_.end())
          logWarn("Overwriting constraint approximation named '%s'", ca->getName().c_str());
        constraint_approximations_[ca->getName()] = ca;
        res.approx = ca;
      }
    }
    else
      logError("Unable to construct constraint approximation for group '%s'", group.c_str());
  }
  return res;
}

ompl::base::StateStoragePtr ompl_interface::ConstraintsLibrary::constructConstraintApproximation(const ModelBasedPlanningContextPtr &pcontext,
                                                                                                 const moveit_msgs::Constraints &constr_sampling,
                                                                                                 const moveit_msgs::Constraints &constr_hard,
                                                                                                 const ConstraintStateStorageOrderFn &order,
                                                                                                 unsigned int samples, unsigned int edges_per_sample,
                                                                                                 ConstraintApproximationConstructionResults &result)
{
  // state storage structure
  ConstraintApproximationStateStorage *cass = new ConstraintApproximationStateStorage(pcontext->getOMPLStateSpace());
  ob::StateStoragePtr sstor(cass);
  
  // construct a sampler for the sampling constraints
  kinematic_constraints::KinematicConstraintSet kset(pcontext->getRobotModel(), robot_state::TransformsConstPtr(new robot_state::Transforms(pcontext->getRobotModel()->getModelFrame())));
  kset.add(constr_hard);

  const robot_state::RobotState &default_state = pcontext->getCompleteInitialRobotState();
  
  int nthreads = 0;
  unsigned int attempts = 0;
  
  double bounds_val = std::numeric_limits<double>::max() / 2.0 - 1.0;
  pcontext->getOMPLStateSpace()->setPlanningVolume(-bounds_val, bounds_val, -bounds_val, bounds_val, -bounds_val, bounds_val);
  pcontext->getOMPLStateSpace()->setup();
  
  // construct the constrained states
#pragma omp parallel
  { 
#pragma omp master
    {
	nthreads = omp_get_num_threads();    
    }
    
    robot_state::RobotState kstate(default_state);
    const constraint_samplers::ConstraintSamplerManagerPtr &csmng = pcontext->getConstraintSamplerManager();
    ConstrainedSampler *csmp = NULL;
    if (csmng)
    {
      constraint_samplers::ConstraintSamplerPtr cs = csmng->selectSampler(pcontext->getPlanningScene(), pcontext->getJointModelGroup()->getName(), constr_sampling);
      if (cs)
        csmp = new ConstrainedSampler(pcontext.get(), cs);
    }
    
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
	  logInform("%d%% complete (kept %0.1lf%% sampled states)", done, 100.0 * (double)sstor->size() / (double)attempts);
	}

	if (!slow_warn && attempts > 10 && attempts > sstor->size() * 100)
	{
	  slow_warn = true;
	  logWarn("Computation of valid state database is very slow...");
	}
      }

      if (attempts > samples && sstor->size() == 0)
      {
	logError("Unable to generate any samples");
	break;
      }
      
      ss->sampleUniform(temp.get());
      pcontext->getOMPLStateSpace()->copyToRobotState(kstate, temp.get());
      if (kset.decide(kstate).satisfied)
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
      result.state_sampling_time = ompl::time::seconds(ompl::time::now() - start);
      logInform("Generated %u states in %lf seconds", (unsigned int)sstor->size(), result.state_sampling_time); 
      if (csmp)
      {
        result.sampling_success_rate = csmp->getConstrainedSamplingRate();
        logInform("Constrained sampling rate: %lf", result.sampling_success_rate);
      }
    }
  }
  if (order)
  {
    logInform("Sorting states...");
    sstor->sort(order);
  }
  
  if (edges_per_sample > 0)
  {
    logInform("Computing graph connections (max %u edges per sample) ...", edges_per_sample);
    
    ompl::tools::SelfConfig sc(pcontext->getOMPLSimpleSetup().getSpaceInformation());
    double range = 0.0;
    sc.configurePlannerRange(range);
    
    // construct connexions
    const ob::StateSpacePtr &space = pcontext->getOMPLSimpleSetup().getStateSpace();
    std::vector<robot_state::RobotState> kstates(nthreads, default_state);
    const std::vector<const ompl::base::State*> &states = sstor->getStates();
    std::vector<ompl::base::ScopedState<> > temps(nthreads, ompl::base::ScopedState<>(space));
    
    ompl::time::point start = ompl::time::now();
    int good = 0;
    int done = -1;
    
#pragma omp parallel for schedule(dynamic) 
    for (std::size_t j = 0 ; j < sstor->size() ; ++j)
    {
      int threadid = omp_get_thread_num();
      robot_state::RobotState &kstate = kstates[threadid];
      robot_state::JointStateGroup *jsg = kstate.getJointStateGroup(pcontext->getJointModelGroup()->getName());
      ompl::base::State *temp = temps[threadid].get();
      int done_now = 100 * j / sstor->size();
      if (done != done_now)
      {
	done = done_now;
        logInform("%d%% complete", done);
      }
      
      for (std::size_t i = j + 1 ; i < sstor->size() ; ++i)
      {
	double d = space->distance(states[j], states[i]);

        if (d > range * 3.0 || d < range / 100.0)
          continue;
        
        space->interpolate(states[j], states[i], 0.5, temp);
        pcontext->getOMPLStateSpace()->copyToRobotState(kstate, temp);
        if (kset.decide(kstate).satisfied)
        {
	  space->interpolate(states[j], states[i], 0.25, temp);
	  pcontext->getOMPLStateSpace()->copyToRobotState(kstate, temp);
	  if (kset.decide(kstate).satisfied)
	  {
            space->interpolate(states[j], states[i], 0.75, temp);
            pcontext->getOMPLStateSpace()->copyToRobotState(kstate, temp);
            if (kset.decide(kstate).satisfied)
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
    result.state_connection_time = ompl::time::seconds(ompl::time::now() - start);
    logInform("Computed possible connexions in %lf seconds. Added %d connexions", result.state_connection_time, good);
  }
  
  return sstor;
}
