/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#include <limits>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>

#include "moveit/ompl_interface/detail/state_validity_checker.h"
#include "moveit/ompl_interface/mrx_custom/rrt/InformedBiTRRT.h"
#include "moveit/ompl_interface/mrx_custom/detail/initial_heuristic_path_helper.h"

ompl::geometric::InformedBiTRRT::InformedBiTRRT(const base::SpaceInformationPtr& si)
  : base::Planner(si, "InformedBiTRRT"), joint_pose_space_(nullptr)
{
  specs_.approximateSolutions = false;
  specs_.directed = true;

  Planner::declareParam<double>("range", this, &InformedBiTRRT::setRange, &InformedBiTRRT::getRange, "0.:1.:10000.");

  // BiTRRT Specific Variables
  setTempChangeFactor(0.1);  // how much to increase the temp each time

  Planner::declareParam<double>("temp_change_factor", this, &InformedBiTRRT::setTempChangeFactor,
                                &InformedBiTRRT::getTempChangeFactor, "0.:.1:1.");
  Planner::declareParam<double>("init_temperature", this, &InformedBiTRRT::setInitTemperature,
                                &InformedBiTRRT::getInitTemperature);
  Planner::declareParam<double>("frontier_threshold", this, &InformedBiTRRT::setFrontierThreshold,
                                &InformedBiTRRT::getFrontierThreshold);
  Planner::declareParam<double>("frontier_node_ratio", this, &InformedBiTRRT::setFrontierNodeRatio,
                                &InformedBiTRRT::getFrontierNodeRatio);
  Planner::declareParam<double>("cost_threshold", this, &InformedBiTRRT::setCostThreshold,
                                &InformedBiTRRT::getCostThreshold);

  // C-forest specific variables
  Planner::declareParam<bool>("cforest_add_path", this, &InformedBiTRRT::setCForestAddPath,
                              &InformedBiTRRT::getCForestAddPath, "0,1");

  Planner::declareParam<std::string>("cforest_optimal_path_rule", this, &InformedBiTRRT::setCForestOptimalPathRule,
                                     &InformedBiTRRT::getCForestOptimalPathRule);
  Planner::declareParam<double>("initial_diameter_multiplier", this, &InformedBiTRRT::setInitialDiameterMultiplier,
                                &InformedBiTRRT::getInitialDiameterMultiplier);

  Planner::declareParam<bool>("add_initial_heuristic_path", this, &InformedBiTRRT::setAddInitialHeuristicPath,
                              &InformedBiTRRT::getAddInitialHeuristicPath, "0,1");
  Planner::declareParam<std::string>("heuristic_path_axis", this, &InformedBiTRRT::setHeuristicPathAxis,
                                     &InformedBiTRRT::getHeuristicPathAxis);
  Planner::declareParam<double>("heuristic_path_eef_step", this, &InformedBiTRRT::setHeuristicPathEEFStep,
                                &InformedBiTRRT::getHeuristicPathEEFStep);
  Planner::declareParam<double>("heuristic_path_max_length", this, &InformedBiTRRT::setHeuristicPathMaxLength,
                                &InformedBiTRRT::getHeuristicPathMaxLength);

  if (add_initial_heuristic_path_)
  {
    OMPL_INFORM("AddInitialHeuristicPath params: \n"
                "1) heuristic_path_axis = %s \n"
                "2) heuristic_path_eef_step = %.3lf \n"
                "3) heuristic_path_max_length = %.3lf.",
                heuristic_path_axis_.c_str(), heuristic_path_eef_step_, heuristic_path_max_length_);
  }
}

ompl::geometric::InformedBiTRRT::~InformedBiTRRT()
{
  freeMemory();
}

void ompl::geometric::InformedBiTRRT::freeMemory()
{
  std::vector<Motion*> motions;

  if (tStart_)
  {
    tStart_->list(motions);
    for (auto& motion : motions)
    {
      if (motion->state != nullptr)
        joint_pose_space_->freeState(motion->state);
      delete motion;
    }
  }

  if (tGoal_)
  {
    tGoal_->list(motions);
    for (auto& motion : motions)
    {
      if (motion->state != nullptr)
        joint_pose_space_->freeState(motion->state);
      delete motion;
    }
  }
}

void ompl::geometric::InformedBiTRRT::clear()
{
  Planner::clear();
  freeMemory();
  if (tStart_)
    tStart_->clear();
  if (tGoal_)
    tGoal_->clear();
  connectionPoint_ = std::make_pair<Motion*, Motion*>(nullptr, nullptr);

  // TRRT specific variables
  temp_ = initTemperature_;
  nonfrontierCount_ = 1;
  frontierCount_ = 1;  // init to 1 to prevent division by zero error
  if (opt_)
    bestCost_ = worstCost_ = opt_->identityCost();
}

void ompl::geometric::InformedBiTRRT::setup()
{
  Planner::setup();
  tools::SelfConfig sc(si_, getName());

  auto space_ptr = std::dynamic_pointer_cast<ompl_interface::JointPoseModelStateSpace>(si_->getStateSpace());

  if (space_ptr != nullptr)
  {
    joint_pose_space_ = space_ptr;
  }
  else
  {
    OMPL_ERROR("InformedBiTRRT only uses JointPoseModelStateSpace. But it's %s.",
               si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->getParameterizationType().c_str());
    throw std::runtime_error("Invalid state space type.");
  }

  // Configuring the range of the planner
  if (maxDistance_ < std::numeric_limits<double>::epsilon())
  {
    sc.configurePlannerRange(maxDistance_);
    maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
  }
  OMPL_INFORM("maxDistance_ : %lf", maxDistance_);

  // Configuring nearest neighbors structures for the planning trees
  if (!tStart_)
    tStart_.reset(new NearestNeighborsGNATNoThreadSafety<Motion*>());
  if (!tGoal_)
    tGoal_.reset(new NearestNeighborsGNATNoThreadSafety<Motion*>());
  tStart_->setDistanceFunction(
      [this](const Motion* a, const Motion* b) { return this->joint_pose_space_->distanceJoint(a->state, b->state); });
  tGoal_->setDistanceFunction(
      [this](const Motion* a, const Motion* b) { return this->joint_pose_space_->distanceJoint(a->state, b->state); });

  // Setup the optimization objective, if it isn't specified
  if (!pdef_ || !pdef_->hasOptimizationObjective())
  {
    OMPL_INFORM("%s: No optimization objective specified.  Defaulting to mechanical work minimization.",
                getName().c_str());
    opt_ = std::make_shared<ompl::base::MechanicalWorkOptimizationObjective>(si_);
  }
  else
    opt_ = pdef_->getOptimizationObjective();

  // Set the threshold that decides if a new node is a frontier node or non-frontier node
  if (frontierThreshold_ < std::numeric_limits<double>::epsilon())
  {
    frontierThreshold_ = joint_pose_space_->getMaximumExtent() * 0.01;
    OMPL_INFORM("%s: Frontier threshold detected to be %lf", getName().c_str(), frontierThreshold_);
  }

  // initialize TRRT specific variables
  temp_ = initTemperature_;
  nonfrontierCount_ = 1;
  frontierCount_ = 1;  // init to 1 to prevent division by zero error
  bestCost_ = worstCost_ = opt_->identityCost();
  connectionRange_ = 10.0 * si_->getStateSpace()->getLongestValidSegmentLength();
}

ompl::geometric::InformedBiTRRT::Motion* ompl::geometric::InformedBiTRRT::addMotion(const base::State* state,
                                                                                    TreeData& tree, Motion* parent)
{
  auto* motion = new Motion(joint_pose_space_);
  joint_pose_space_->copyState(motion->state, state);
  motion->cost = opt_->stateCost(motion->state);
  motion->parent = parent;
  motion->root = parent != nullptr ? parent->root : nullptr;

  if (opt_->isCostBetterThan(motion->cost, bestCost_))  // motion->cost is better than the existing best
    bestCost_ = motion->cost;
  if (opt_->isCostBetterThan(worstCost_, motion->cost))  // motion->cost is worse than the existing worst
    worstCost_ = motion->cost;

  // Add start motion to the tree
  tree->add(motion);
  return motion;
}

bool ompl::geometric::InformedBiTRRT::transitionTest(const base::Cost& motionCost)
{
  // Disallow any cost that is not better than the cost threshold
  if (!opt_->isCostBetterThan(motionCost, costThreshold_))
    return false;

  // Always accept if the cost is near or below zero
  if (motionCost.value() < 1e-4)
    return true;

  double dCost = motionCost.value();
  double transitionProbability = exp(-dCost / temp_);
  if (transitionProbability > 0.5)
  {
    double costRange = worstCost_.value() - bestCost_.value();
    if (fabs(costRange) > 1e-4)  // Do not divide by zero
      // Successful transition test.  Decrease the temperature slightly
      temp_ /= exp(dCost / (0.1 * costRange));

    return true;
  }

  // The transition failed.  Increase the temperature (slightly)
  temp_ *= tempChangeFactor_;
  return false;
}

bool ompl::geometric::InformedBiTRRT::minExpansionControl(double dist)
{
  if (dist > frontierThreshold_)  // Exploration
  {
    ++frontierCount_;
    return true;
  }
  // Refinement
  // Check the current ratio first before accepting it
  if ((double)nonfrontierCount_ / (double)frontierCount_ > frontierNodeRatio_)
    return false;

  ++nonfrontierCount_;
  return true;
}

ompl::geometric::InformedBiTRRT::GrowResult
ompl::geometric::InformedBiTRRT::extendTree(Motion* nearest, TreeData& tree, Motion* toMotion, Motion*& result)
{
  bool reach = true;

  // Compute the state to extend toward
  bool treeIsStart = (tree == tStart_);
  double d = (treeIsStart ? joint_pose_space_->distanceJoint(nearest->state, toMotion->state) :
                            joint_pose_space_->distanceJoint(toMotion->state, nearest->state));
  // Truncate the random state to be no more than maxDistance_ from nearest neighbor
  if (d > maxDistance_)
  {
    if (tree == tStart_)
      joint_pose_space_->interpolate(nearest->state, toMotion->state, maxDistance_ / d, toMotion->state);
    else
      joint_pose_space_->interpolate(toMotion->state, nearest->state, 1.0 - maxDistance_ / d, toMotion->state);
    d = maxDistance_;
    reach = false;
  }

  // Validating the motion
  // If we are in the goal tree, we validate the motion in reverse
  // si_->checkMotion assumes that the first argument is valid, so we must check this explicitly
  // If the motion is valid, check the probabilistic transition test and the
  // expansion control to ensure high quality nodes are added.
  bool validMotion =
      (tree == tStart_ ? si_->checkMotion(nearest->state, toMotion->state) :
                         si_->isValid(toMotion->state) && si_->checkMotion(toMotion->state, nearest->state)) &&
      transitionTest(tree == tStart_ ? opt_->motionCost(nearest->state, toMotion->state) :
                                       opt_->motionCost(toMotion->state, nearest->state)) &&
      minExpansionControl(d);

  if (validMotion)
  {
    result = addMotion(toMotion->state, tree, nearest);
    return reach ? SUCCESS : ADVANCED;
  }

  return FAILED;
}

ompl::geometric::InformedBiTRRT::GrowResult ompl::geometric::InformedBiTRRT::extendTree(Motion* toMotion,
                                                                                        TreeData& tree, Motion*& result)
{
  // Nearest neighbor
  Motion* nearest = tree->nearest(toMotion);
  return extendTree(nearest, tree, toMotion, result);
}

bool ompl::geometric::InformedBiTRRT::connectTrees(Motion* nmotion, TreeData& tree, Motion* xmotion)
{
  // Get the nearest state to nmotion in tree (nmotion is NOT in tree)
  Motion* nearest = tree->nearest(nmotion);
  bool treeIsStart = tree == tStart_;
  double dist = (treeIsStart ? joint_pose_space_->distanceJoint(nearest->state, nmotion->state) :
                               joint_pose_space_->distanceJoint(nmotion->state, nearest->state));

  // Do not attempt a connection if the trees are far apart
  if (dist > connectionRange_)
    return false;

  // Copy the resulting state into our scratch space
  joint_pose_space_->copyState(xmotion->state, nmotion->state);

  // Do not try to connect states directly.  Must chop up the
  // extension into segments, just in case one piece fails
  // the transition test
  GrowResult result;
  Motion* next = nullptr;
  do
  {
    // Extend tree from nearest toward xmotion
    // Store the result into next
    // This function MAY trash xmotion
    result = extendTree(nearest, tree, xmotion, next);

    if (result == ADVANCED)
    {
      nearest = next;

      // xmotion may get trashed during extension, so we reload it here
      joint_pose_space_->copyState(xmotion->state,
                                   nmotion->state);  // xmotion may get trashed during extension, so we reload it here
    }
  } while (result == ADVANCED);

  // Successful connection
  if (result == SUCCESS)
  {
    Motion* startMotion = treeIsStart ? next : nmotion;
    Motion* goalMotion = treeIsStart ? nmotion : next;

    // Make sure start-goal pair is valid
    if (pdef_->getGoal()->isStartGoalPairValid(startMotion->root, goalMotion->root))
    {
      // Since we have connected, nmotion->state and next->state have the same value
      // We need to check one of their parents to avoid a duplicate state in the solution path
      // One of these must be true, since we do not ever attempt to connect start and goal directly.
      if (startMotion->parent != nullptr)
        startMotion = startMotion->parent;
      else
        goalMotion = goalMotion->parent;

      connectionPoint_ = std::make_pair(startMotion, goalMotion);
      return true;
    }
  }

  return false;
}

ompl::base::PlannerStatus ompl::geometric::InformedBiTRRT::solve(const base::PlannerTerminationCondition& ptc)
{
  num_solutions_ = 0;

  // Basic error checking
  checkValidity();

  // Goal information
  base::Goal* goal = pdef_->getGoal().get();
  auto* gsr = dynamic_cast<base::GoalSampleableRegion*>(goal);

  if (gsr == nullptr)
  {
    OMPL_ERROR("%s: Goal object does not derive from GoalSampleableRegion", getName().c_str());
    return base::PlannerStatus::INVALID_GOAL;
  }

  // Loop through the (valid) input states and add them to the start tree
  while (const base::State* state = pis_.nextStart())
  {
    auto* motion = new Motion(joint_pose_space_);
    joint_pose_space_->copyState(motion->state, state);
    motion->cost = opt_->stateCost(motion->state);
    motion->root = motion->state;  // this state is the root of a tree

    if (tStart_->size() == 0)  // do not overwrite best/worst from a prior call to solve
      worstCost_ = bestCost_ = motion->cost;

    // Add start motion to the tree
    tStart_->add(motion);

    if (add_initial_heuristic_path_)
      addInitialHeuristicPath(motion, tStart_);
  }

  if (tStart_->size() == 0)
  {
    OMPL_ERROR("%s: Start tree has no valid states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  // Do the same for the goal tree, if it is empty, but only once
  if (tGoal_->size() == 0)
  {
    const base::State* state = pis_.nextGoal(ptc);
    if (state != nullptr)
    {
      Motion* motion = addMotion(state, tGoal_);
      motion->root = motion->state;  // this state is the root of a tree

      if (add_initial_heuristic_path_)
        addInitialHeuristicPath(motion, tGoal_);
    }
  }

  if (tGoal_->size() == 0)
  {
    OMPL_ERROR("%s: Goal tree has no valid states!", getName().c_str());
    return base::PlannerStatus::INVALID_GOAL;
  }

  OMPL_INFORM("%s: Planning started with %d states already in datastructure", getName().c_str(),
              (int)(tStart_->size() + tGoal_->size()));

  sampler_ = initSampler();

  auto* rmotion = new Motion(joint_pose_space_);
  base::State* rstate = rmotion->state;

  auto* xmotion = new Motion(joint_pose_space_);
  base::State* xstate = xmotion->state;

  TreeData tree = tStart_;
  TreeData otherTree = tGoal_;

  bool solved = false;
  // Planning loop
  while (!ptc)
  {
    // Check if there are more goal states
    if (pis_.getSampledGoalsCount() < tGoal_->size() / 2)
    {
      if (const base::State* state = pis_.nextGoal())
      {
        Motion* motion = addMotion(state, tGoal_);
        motion->root = motion->state;  // this state is the root of a tree

        if (add_initial_heuristic_path_)
          addInitialHeuristicPath(motion, tGoal_);
      }
    }

    // Sample a state uniformly at random
    sampler_->sampleUniform(rstate);

    Motion* result;                                   // the motion that gets added in extendTree
    if (extendTree(rmotion, tree, result) != FAILED)  // we added something new to the tree
    {
      // Try to connect the other tree to the node we just added
      if (connectTrees(result, otherTree, xmotion))
      {
        // The trees have been connected.  Construct the solution path
        Motion* solution = connectionPoint_.first;
        std::vector<Motion*> mpath1;
        while (solution != nullptr)
        {
          mpath1.push_back(solution);
          solution = solution->parent;
        }

        solution = connectionPoint_.second;
        std::vector<Motion*> mpath2;
        while (solution != nullptr)
        {
          mpath2.push_back(solution);
          solution = solution->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        path->getStates().reserve(mpath1.size() + mpath2.size());
        for (int i = mpath1.size() - 1; i >= 0; --i)
          path->append(mpath1[i]->state);
        for (auto& i : mpath2)
          path->append(i->state);

        const std::size_t numStates = path->getStateCount();

        PathSimplifier psk(si_, pdef_->getGoal(), pdef_->getOptimizationObjective());

        if (psk.simplify(*path, ptc))
        {
          ompl::base::PlannerSolution sol(path);
          sol.setPlannerName(getName());
          sol.setOptimized(opt_, path->cost(opt_), true);

          pdef_->addSolutionPath(sol);
          solved = true;
          OMPL_INFORM("Path simplification changed from %d to %d states", numStates, path->getStateCount());
          break;
        }
      }
    }

    std::swap(tree, otherTree);

    // C-Forest compatibility
    checkSolutionUpdate();
  }

  joint_pose_space_->freeState(rstate);
  joint_pose_space_->freeState(xstate);
  delete rmotion;
  delete xmotion;

  OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
              tStart_->size(), tGoal_->size());
  return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

ompl_interface::EllipsoidalSamplerPtr ompl::geometric::InformedBiTRRT::initSampler()
{
  std::vector<Motion*> starts, goals;
  tStart_->list(starts);
  tGoal_->list(goals);

  const unsigned int dim = joint_pose_space_->getNumPositions();
  std::vector<double> focus1(dim), focus2(dim);
  joint_pose_space_->copyPositionsToReals(focus1, starts[0]->state);
  joint_pose_space_->copyPositionsToReals(focus2, goals[0]->state);

  auto sampler = std::make_shared<ompl_interface::EllipsoidalSampler>(dim, focus1, focus2, joint_pose_space_);

  Eigen::Map<Eigen::VectorXd> start(focus1.data(), focus1.size());
  Eigen::Map<Eigen::VectorXd> goal(focus2.data(), focus2.size());

  std::stringstream ss;
  ss << "Sampler" << std::endl
     << "start :" << std::endl
     << start << std::endl
     << "goal :" << std::endl
     << goal << std::endl
     << "minTraverse : " << sampler->getMinTransverseDiameter();
  OMPL_INFORM("%s", ss.str().c_str());

  if (initial_diameter_multiplier_ > 0)
  {
    const double diameter = sampler->getMinTransverseDiameter() * initial_diameter_multiplier_;
    sampler->setTraverseDiameter(diameter);
    OMPL_INFORM("Set traverse diameter=%lf. Initial diameter multiplier=%lf.", diameter, initial_diameter_multiplier_);
  }

  return sampler;
}

double ompl::geometric::InformedBiTRRT::getDiameter(const ompl::base::PathPtr& path) const
{
  const auto pg = path->as<ompl::geometric::PathGeometric>();

  double diameter = std::numeric_limits<double>::min();
  ompl::base::State* max_state;

  for (const auto state : pg->getStates())
  {
    const double pl = sampler_->getPathLength(state);

    if (pl > diameter)
    {
      diameter = pl;
      max_state = state;
    }
  }

  std::stringstream ss;
  ss << "Max Diameter : " << diameter << std::endl << " State :" << std::endl;
  joint_pose_space_->printPositions(max_state, ss);

  OMPL_INFORM("%s", ss.str().c_str());

  return diameter;
}

void ompl::geometric::InformedBiTRRT::pruneImpl(const double diameter, TreeData& tree)
{
  constexpr double remove_flag = std::numeric_limits<double>::min();

  std::vector<Motion*> motions, prev_motions, curr_motions;

  if (tree)
    tree->list(motions);

  for (auto motion : motions)
  {
    if (sampler_->getPathLength(motion->state) > diameter)
    {
      if (motion->root != motion->state)
        motion->remove_flag = true;
    }
  }
  const auto func = [](const Motion* motion) { return !motion->remove_flag; };
  std::copy_if(motions.begin(), motions.end(), std::back_inserter(prev_motions), func);

  size_t prev_cnt, curr_cnt;
  size_t iter = 1;
  do
  {
    prev_cnt = prev_motions.size();

    for (auto motion : prev_motions)
    {
      if (motion->root != motion->state)
      {
        if (motion->parent != nullptr && motion->parent->remove_flag)
          motion->remove_flag = true;
      }
    }

    curr_motions.clear();
    std::copy_if(prev_motions.begin(), prev_motions.end(), std::back_inserter(curr_motions), func);

    curr_cnt = curr_motions.size();
    prev_motions = curr_motions;
    OMPL_INFORM("Iter %zu. prev_cnt=%zu. curr_cnt=%zu.", iter++, prev_cnt, curr_cnt);
  } while (prev_cnt != curr_cnt);

  tree->clear();
  tree->add(prev_motions);

  size_t num_pruned = 0;
  for (auto motion : motions)
  {
    if (motion->remove_flag)
    {
      if (motion->state != nullptr)
        joint_pose_space_->freeState(motion->state);
      delete motion;
      num_pruned++;
    }
  }

  OMPL_INFORM("Before prunning, num_states=%zu. After prunning, num_states=%zu. num_pruned=%zu", motions.size(),
              prev_motions.size(), num_pruned);
}

void ompl::geometric::InformedBiTRRT::prune(const double diameter)
{
  OMPL_INFORM("Start pruning tStart_");
  pruneImpl(diameter, tStart_);
  OMPL_INFORM("Start pruning tGoal_");
  pruneImpl(diameter, tGoal_);
}

void ompl::geometric::InformedBiTRRT::addPathImpl(const ompl::base::PathPtr& path, TreeData& tree)
{
  const bool treeIsStart = tree == tStart_;

  // Add states closer to one of both.
  const auto comp_func =
      treeIsStart ?
          [](const ompl_interface::EllipsoidalSamplerPtr& sampler, const std::vector<double>& curr_point) {
            return sampler->distanceFromStartPoint(curr_point) < sampler->distanceFromGoalPoint(curr_point);
          } :
          [](const ompl_interface::EllipsoidalSamplerPtr& sampler, const std::vector<double>& curr_point) {
            return sampler->distanceFromStartPoint(curr_point) > sampler->distanceFromGoalPoint(curr_point);
          };

  const auto& states = path->as<ompl::geometric::PathGeometric>()->getStates();
  std::vector<double> curr_point(joint_pose_space_->getNumPositions());
  auto* rmotion = new Motion(joint_pose_space_);
  Motion* result;

  size_t cnt = 0;

  for (auto it = states.begin(); it != states.end(); it++)
  {
    joint_pose_space_->copyPositionsToReals(curr_point, *it);

    if (comp_func(sampler_, curr_point))
    {
      joint_pose_space_->copyState(rmotion->state, *it);
      Motion* nearest = tree->nearest(rmotion);

      if (joint_pose_space_->distanceJoint(nearest->state, rmotion->state) > 1e-4)
      {
        extendTree(nearest, tree, rmotion, result);
        cnt++;
      }
    }
  }

  OMPL_INFORM("Add path states [%zu/%zu]", cnt, states.size());
}

void ompl::geometric::InformedBiTRRT::addPath(const ompl::base::PathPtr& path)
{
  OMPL_INFORM("Start addPath tStart_");
  addPathImpl(path, tStart_);
  OMPL_INFORM("Start addPath tGoal_");
  addPathImpl(path, tGoal_);
}

void ompl::geometric::InformedBiTRRT::checkSolutionUpdate()
{
  const auto solutions = pdef_->getSolutions();

  if (solutions.size() > 0 && solutions.size() != num_solutions_)
  {
    num_solutions_ = solutions.size();

    std::vector<double> diameters;
    std::transform(solutions.begin(), solutions.end(), std::back_inserter(diameters),
                   [this](const ompl::base::PlannerSolution& sol) { return this->getDiameter(sol.path_); });

    size_t min_idx;

    if (cforest_opt_rule_ == OptimalPathRule::DIAMETER)
    {
      const auto min_it = std::min_element(diameters.begin(), diameters.end());
      min_idx = std::distance(diameters.begin(), min_it);
    }
    else if (cforest_opt_rule_ == OptimalPathRule::COST)
    {
      const auto min_it =
          std::min_element(solutions.begin(), solutions.end(),
                           [](const ompl::base::PlannerSolution& a, const ompl::base::PlannerSolution& b) {
                             return a.cost_.value() < b.cost_.value();
                           });
      min_idx = std::distance(solutions.begin(), min_it);
    }
    else
    {
      throw std::runtime_error("NotImplementError");
    }

    const double diameter = diameters[min_idx];

    sampler_->setTraverseDiameter(diameter);

    // Add path from the solution
    if (cforest_add_path_)
      addPath(solutions[min_idx].path_);

    // Prune vertices in both trees by ellipsoid diameter.
    prune(diameter);

    OMPL_INFORM("Solution updated. solution=[%zu/%zu]. Diameter=%lf. Cost=%lf.", min_idx, num_solutions_, diameter,
                solutions[min_idx].cost_.value());
  }
}

void ompl::geometric::InformedBiTRRT::addInitialHeuristicPath(Motion* start_motion, TreeData& tree)
{
  auto checker = std::dynamic_pointer_cast<ompl_interface::StateValidityChecker>(si_->getStateValidityChecker());

  if (checker == nullptr)
    throw std::runtime_error("StateValidityChecker should be ompl_interface::StateValidityChecker.");

  ompl_interface::InitialHeuristicPathHelper helper(checker, heuristic_path_axis_, heuristic_path_eef_step_,
                                                    heuristic_path_max_length_);

  if (!helper.isLeafGroup())
  {
    OMPL_ERROR("%s is not a leaf group. Do not add initial heuristic path to the tree.", helper.getGroupName().c_str());
    return;
  }

  const auto path = helper.getCartesianPath(start_motion->state);

  auto* tmp_state = joint_pose_space_->allocState();
  Motion* parent = start_motion;

  // path[0] is same with start_motion->state.
  for (size_t i = 1; i < path.size(); i++)
  {
    joint_pose_space_->copyToOMPLState(tmp_state, *path[i]);
    parent = addMotion(tmp_state, tree, parent);
  }

  joint_pose_space_->freeState(tmp_state);

  OMPL_DEBUG("addInitialHeuristicPath adds %zu states to %s.", path.size() - 1, tree == tStart_ ? "tStart" : "tGoal");
}

void ompl::geometric::InformedBiTRRT::getPlannerData(base::PlannerData& data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion*> motions;
  if (tStart_)
    tStart_->list(motions);
  for (auto& motion : motions)
  {
    if (motion->parent == nullptr)
      data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
    else
    {
      data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
    }
  }

  motions.clear();
  if (tGoal_)
    tGoal_->list(motions);
  for (auto& motion : motions)
  {
    if (motion->parent == nullptr)
      data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
    else
    {
      // The edges in the goal tree are reversed to be consistent with start tree
      data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
    }
  }

  // Add the edge connecting the two trees
  if ((connectionPoint_.first != nullptr) && (connectionPoint_.second != nullptr))
    data.addEdge(data.vertexIndex(connectionPoint_.first->state), data.vertexIndex(connectionPoint_.second->state));
}
