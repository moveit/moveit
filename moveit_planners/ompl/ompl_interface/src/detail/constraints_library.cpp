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

#include <moveit/ompl_interface/detail/constraints_library.h>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <moveit/profiler/profiler.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace ompl_interface
{
namespace
{
template <typename T>
void msgToHex(const T& msg, std::string& hex)
{
  static const char symbol[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  const size_t serial_size_arg = ros::serialization::serializationLength(msg);

  boost::shared_array<uint8_t> buffer_arg(new uint8_t[serial_size_arg]);
  ros::serialization::OStream stream_arg(buffer_arg.get(), serial_size_arg);
  ros::serialization::serialize(stream_arg, msg);
  hex.resize(serial_size_arg * 2);
  for (std::size_t i = 0; i < serial_size_arg; ++i)
  {
    hex[i * 2] = symbol[buffer_arg[i] / 16];
    hex[i * 2 + 1] = symbol[buffer_arg[i] % 16];
  }
}

template <typename T>
void hexToMsg(const std::string& hex, T& msg)
{
  const size_t serial_size_arg = hex.length() / 2;
  boost::shared_array<uint8_t> buffer_arg(new uint8_t[serial_size_arg]);
  for (std::size_t i = 0; i < serial_size_arg; ++i)
  {
    buffer_arg[i] = (hex[i * 2] <= '9' ? (hex[i * 2] - '0') : (hex[i * 2] - 'A' + 10)) * 16 +
                    (hex[i * 2 + 1] <= '9' ? (hex[i * 2 + 1] - '0') : (hex[i * 2 + 1] - 'A' + 10));
  }
  ros::serialization::IStream stream_arg(buffer_arg.get(), serial_size_arg);
  ros::serialization::deserialize(stream_arg, msg);
}
}

void ConstraintsLibrary::loadConstraintApproximations(const std::string& path)
{
  constraint_approximations_.clear();
  std::ifstream fin((path + "/manifest").c_str());
  if (!fin.good())
  {
    ROS_WARN_NAMED("constraints_library", "Manifest not found in folder '%s'. Not loading constraint approximations.",
                   path.c_str());
    return;
  }

  ROS_INFO_NAMED("constraints_library", "Loading constrained space approximations from '%s'...", path.c_str());

  while (fin.good() && !fin.eof())
  {
    std::string group, state_space_parameterization, serialization, filename;
    bool explicit_motions;
    unsigned int milestones;
    fin >> group;
    if (fin.eof())
      break;
    fin >> state_space_parameterization;
    if (fin.eof())
      break;
    fin >> explicit_motions;
    if (fin.eof())
      break;
    fin >> milestones;
    if (fin.eof())
      break;
    fin >> serialization;
    if (fin.eof())
      break;
    fin >> filename;

    if (context_->getGroupName() != group &&
        context_->getOMPLStateSpace()->getParameterizationType() != state_space_parameterization)
    {
      ROS_INFO_NAMED("constraints_library",
                     "Ignoring constraint approximation of type '%s' for group '%s' from '%s'...",
                     state_space_parameterization.c_str(), group.c_str(), filename.c_str());
      continue;
    }

    ROS_INFO_NAMED("constraints_library", "Loading constraint approximation of type '%s' for group '%s' from '%s'...",
                   state_space_parameterization.c_str(), group.c_str(), filename.c_str());

    moveit_msgs::Constraints msg;
    hexToMsg(serialization, msg);
    ConstraintApproximationStateStorage* cass =
        new ConstraintApproximationStateStorage(context_->getOMPLSimpleSetup()->getStateSpace());
    cass->load((path + "/" + filename).c_str());
    ConstraintApproximationPtr cap(new ConstraintApproximation(group, state_space_parameterization, explicit_motions,
                                                               msg, filename, ompl::base::StateStoragePtr(cass),
                                                               milestones));
    if (constraint_approximations_.find(cap->getName()) != constraint_approximations_.end())
      ROS_WARN_NAMED("constraints_library", "Overwriting constraint approximation named '%s'", cap->getName().c_str());
    constraint_approximations_[cap->getName()] = cap;
    std::size_t sum = 0;
    for (std::size_t i = 0; i < cass->size(); ++i)
      sum += cass->getMetadata(i).first.size();
    ROS_INFO_NAMED("constraints_library", "Loaded %lu states (%lu milestones) and %lu connections (%0.1lf per state) "
                                          "for constraint named '%s'%s",
                   cass->size(), cap->getMilestoneCount(), sum, (double)sum / (double)cap->getMilestoneCount(),
                   msg.name.c_str(), explicit_motions ? ". Explicit motions included." : "");
  }
  ROS_INFO_NAMED("constraints_library", "Done loading constrained space approximations.");
}

void ConstraintsLibrary::saveConstraintApproximations(const std::string& path)
{
  ROS_INFO_NAMED("constraints_library", "Saving %u constrained space approximations to '%s'",
                 (unsigned int)constraint_approximations_.size(), path.c_str());
  try
  {
    boost::filesystem::create_directories(path);
  }
  catch (...)
  {
  }

  std::ofstream fout((path + "/manifest").c_str());
  if (fout.good())
    for (std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.begin();
         it != constraint_approximations_.end(); ++it)
    {
      fout << it->second->getGroup() << std::endl;
      fout << it->second->getStateSpaceParameterization() << std::endl;
      fout << it->second->hasExplicitMotions() << std::endl;
      fout << it->second->getMilestoneCount() << std::endl;
      std::string serialization;
      msgToHex(it->second->getConstraintsMsg(), serialization);
      fout << serialization << std::endl;
      fout << it->second->getFilename() << std::endl;
      if (it->second->getStateStorage())
        it->second->getStateStorage()->store((path + "/" + it->second->getFilename()).c_str());
    }
  else
    ROS_ERROR_NAMED("constraints_library", "Unable to save constraint approximation to '%s'", path.c_str());
  fout.close();
}

void ConstraintsLibrary::clearConstraintApproximations()
{
  constraint_approximations_.clear();
}

void ConstraintsLibrary::printConstraintApproximations(std::ostream& out) const
{
  for (std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.begin();
       it != constraint_approximations_.end(); ++it)
  {
    out << it->second->getGroup() << std::endl;
    out << it->second->getStateSpaceParameterization() << std::endl;
    out << it->second->hasExplicitMotions() << std::endl;
    out << it->second->getMilestoneCount() << std::endl;
    out << it->second->getFilename() << std::endl;
    out << it->second->getConstraintsMsg() << std::endl;
  }
}

const ConstraintApproximationPtr&
ConstraintsLibrary::getConstraintApproximation(const moveit_msgs::Constraints& msg) const
{
  std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.find(msg.name);
  if (it != constraint_approximations_.end())
    return it->second;

  static ConstraintApproximationPtr empty;
  return empty;
}

ConstraintApproximationConstructionResults ConstraintsLibrary::addConstraintApproximation(
    const moveit_msgs::Constraints& constr, const std::string& group,
    const planning_scene::PlanningSceneConstPtr& scene, const ConstraintApproximationConstructionOptions& options)
{
  return addConstraintApproximation(constr, constr, group, scene, options);
}

ConstraintApproximationConstructionResults ConstraintsLibrary::addConstraintApproximation(
    const moveit_msgs::Constraints& constr_sampling, const moveit_msgs::Constraints& constr_hard,
    const std::string& group, const planning_scene::PlanningSceneConstPtr& scene,
    const ConstraintApproximationConstructionOptions& options)
{
  ConstraintApproximationConstructionResults res;

  if (context_->getGroupName() != group &&
      context_->getOMPLStateSpace()->getParameterizationType() != options.state_space_parameterization)
  {
    ROS_INFO_NAMED("constraints_library", "Ignoring constraint approximation of type '%s' for group '%s'...",
                   options.state_space_parameterization.c_str(), group.c_str());
    return res;
  }

  context_->clear();
  context_->setPlanningScene(scene);
  context_->setCompleteInitialState(scene->getCurrentState());

  ros::WallTime start = ros::WallTime::now();
  ompl::base::StateStoragePtr ss =
      constructConstraintApproximation(context_, constr_sampling, constr_hard, options, res);
  ROS_INFO_NAMED("constraints_library", "Spent %lf seconds constructing the database",
                 (ros::WallTime::now() - start).toSec());
  if (ss)
  {
    ConstraintApproximationPtr ca(new ConstraintApproximation(
        group, options.state_space_parameterization, options.explicit_motions, constr_hard,
        group + "_" + boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()) +
            ".ompldb",
        ss, res.milestones));
    if (constraint_approximations_.find(ca->getName()) != constraint_approximations_.end())
      ROS_WARN_NAMED("constraints_library", "Overwriting constraint approximation named '%s'", ca->getName().c_str());
    constraint_approximations_[ca->getName()] = ca;
    res.approx = ca;
  }
  else
    ROS_ERROR_NAMED("constraints_library", "Unable to construct constraint approximation for group '%s'",
                    group.c_str());
  return res;
}

ompl::base::StateStoragePtr ConstraintsLibrary::constructConstraintApproximation(
    ModelBasedPlanningContext* pcontext, const moveit_msgs::Constraints& constr_sampling,
    const moveit_msgs::Constraints& constr_hard, const ConstraintApproximationConstructionOptions& options,
    ConstraintApproximationConstructionResults& result)
{
  // state storage structure
  ConstraintApproximationStateStorage* cass = new ConstraintApproximationStateStorage(pcontext->getOMPLStateSpace());
  ompl::base::StateStoragePtr sstor(cass);

  // construct a sampler for the sampling constraints
  kinematic_constraints::KinematicConstraintSet kset(pcontext->getRobotModel());
  robot_state::Transforms no_transforms(pcontext->getRobotModel()->getModelFrame());
  kset.add(constr_hard, no_transforms);

  const robot_state::RobotState& default_state = pcontext->getCompleteInitialRobotState();

  unsigned int attempts = 0;

  double bounds_val = std::numeric_limits<double>::max() / 2.0 - 1.0;
  pcontext->getOMPLStateSpace()->setPlanningVolume(-bounds_val, bounds_val, -bounds_val, bounds_val, -bounds_val,
                                                   bounds_val);
  pcontext->getOMPLStateSpace()->setup();

  // construct the constrained states

  robot_state::RobotState kstate(default_state);
  const constraint_samplers::ConstraintSamplerManagerPtr& csmng = pcontext->getConstraintSamplerManager();
  ConstrainedSampler* csmp = NULL;
  if (csmng)
  {
    constraint_samplers::ConstraintSamplerPtr cs =
        csmng->selectSampler(pcontext->getPlanningScene(), pcontext->getJointModelGroup()->getName(), constr_sampling);
    if (cs)
      csmp = new ConstrainedSampler(pcontext, cs);
  }

  ompl::base::StateSamplerPtr ss(csmp ? ompl::base::StateSamplerPtr(csmp) :
                                        pcontext->getOMPLStateSpace()->allocDefaultStateSampler());

  ompl::base::ScopedState<> temp(pcontext->getOMPLStateSpace());
  int done = -1;
  bool slow_warn = false;
  ompl::time::point start = ompl::time::now();
  while (sstor->size() < options.samples)
  {
    ++attempts;
    int done_now = 100 * sstor->size() / options.samples;
    if (done != done_now)
    {
      done = done_now;
      ROS_INFO_NAMED("constraints_library", "%d%% complete (kept %0.1lf%% sampled states)", done,
                     100.0 * (double)sstor->size() / (double)attempts);
    }

    if (!slow_warn && attempts > 10 && attempts > sstor->size() * 100)
    {
      slow_warn = true;
      ROS_WARN_NAMED("constraints_library", "Computation of valid state database is very slow...");
    }

    if (attempts > options.samples && sstor->size() == 0)
    {
      ROS_ERROR_NAMED("constraints_library", "Unable to generate any samples");
      break;
    }

    ss->sampleUniform(temp.get());
    pcontext->getOMPLStateSpace()->copyToRobotState(kstate, temp.get());
    if (kset.decide(kstate).satisfied)
    {
      if (sstor->size() < options.samples)
      {
        temp->as<ModelBasedStateSpace::StateType>()->tag = sstor->size();
        sstor->addState(temp.get());
      }
    }
  }

  result.state_sampling_time = ompl::time::seconds(ompl::time::now() - start);
  ROS_INFO_NAMED("constraints_library", "Generated %u states in %lf seconds", (unsigned int)sstor->size(),
                 result.state_sampling_time);
  if (csmp)
  {
    result.sampling_success_rate = csmp->getConstrainedSamplingRate();
    ROS_INFO_NAMED("constraints_library", "Constrained sampling rate: %lf", result.sampling_success_rate);
  }

  result.milestones = sstor->size();
  if (options.edges_per_sample > 0)
  {
    ROS_INFO_NAMED("constraints_library", "Computing graph connections (max %u edges per sample) ...",
                   options.edges_per_sample);

    // construct connexions
    const ompl::base::StateSpacePtr& space = pcontext->getOMPLSimpleSetup()->getStateSpace();
    unsigned int milestones = sstor->size();
    std::vector<ompl::base::State*> int_states(options.max_explicit_points, NULL);
    pcontext->getOMPLSimpleSetup()->getSpaceInformation()->allocStates(int_states);

    ompl::time::point start = ompl::time::now();
    int good = 0;
    int done = -1;

    for (std::size_t j = 0; j < milestones; ++j)
    {
      int done_now = 100 * j / milestones;
      if (done != done_now)
      {
        done = done_now;
        ROS_INFO_NAMED("constraints_library", "%d%% complete", done);
      }
      if (cass->getMetadata(j).first.size() >= options.edges_per_sample)
        continue;

      const ompl::base::State* sj = sstor->getState(j);

      for (std::size_t i = j + 1; i < milestones; ++i)
      {
        if (cass->getMetadata(i).first.size() >= options.edges_per_sample)
          continue;
        double d = space->distance(sstor->getState(i), sj);
        if (d >= options.max_edge_length)
          continue;
        unsigned int isteps =
            std::min<unsigned int>(options.max_explicit_points, d / options.explicit_points_resolution);
        double step = 1.0 / (double)isteps;
        double remain = 1.0;
        bool ok = true;
        space->interpolate(sstor->getState(i), sj, step, int_states[0]);
        for (unsigned int k = 1; k < isteps; ++k)
        {
          double this_step = step / (1.0 - (k - 1) * step);
          space->interpolate(int_states[k - 1], sj, this_step, int_states[k]);
          pcontext->getOMPLStateSpace()->copyToRobotState(kstate, int_states[k]);
          if (!kset.decide(kstate).satisfied)
          {
            ok = false;
            break;
          }
        }

        if (ok)
        {
          cass->getMetadata(i).first.push_back(j);
          cass->getMetadata(j).first.push_back(i);

          if (options.explicit_motions)
          {
            cass->getMetadata(i).second[j].first = sstor->size();
            for (unsigned int k = 0; k < isteps; ++k)
            {
              int_states[k]->as<ModelBasedStateSpace::StateType>()->tag = -1;
              sstor->addState(int_states[k]);
            }
            cass->getMetadata(i).second[j].second = sstor->size();
            cass->getMetadata(j).second[i] = cass->getMetadata(i).second[j];
          }

          good++;
          if (cass->getMetadata(j).first.size() >= options.edges_per_sample)
            break;
        }
      }
    }

    result.state_connection_time = ompl::time::seconds(ompl::time::now() - start);
    ROS_INFO_NAMED("constraints_library", "Computed possible connexions in %lf seconds. Added %d connexions",
                   result.state_connection_time, good);
    pcontext->getOMPLSimpleSetup()->getSpaceInformation()->freeStates(int_states);

    return sstor;
  }
}
}  // ompl_interface
