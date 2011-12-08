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

#include "ompl_interface/planning_group.h"
#include "ompl_interface/detail/threadsafe_state_storage.h"
#include "ompl_interface/detail/state_validity_checker.h"
#include "ompl_interface/detail/constrained_sampler.h"
#include "ompl_interface/detail/constrained_goal_sampler.h"
#include "ompl_interface/detail/constrained_goal_region.h"
#include "ompl_interface/detail/projection_evaluators.h"

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <planning_models/conversions.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <sstream>

ompl::base::PlannerPtr ompl_interface::PlanningGroup::plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
                                                                       const std::map<std::string, std::string> &config) const
{
    ompl::base::Planner *p = NULL;
    if (planner == "geometric::RRT")
        p = new ompl::geometric::RRT(si);
    else if (planner == "geometric::RRTConnect")
        p = new ompl::geometric::RRTConnect(si);
    else if (planner == "geometric::pRRT")
        p = new ompl::geometric::pRRT(si);
    else if (planner == "geometric::LazyRRT")
        p = new ompl::geometric::LazyRRT(si);
    else if (planner == "geometric::EST")
        p = new ompl::geometric::EST(si);
    else if (planner == "geometric::SBL")
        p = new ompl::geometric::SBL(si);
    else if (planner == "geometric::pSBL")
        p = new ompl::geometric::pSBL(si);
    else if (planner == "geometric::KPIECE")
        p = new ompl::geometric::KPIECE1(si);
    else if (planner == "geometric::BKPIECE")
        p = new ompl::geometric::BKPIECE1(si);
    else if (planner == "geometric::LBKPIECE")
        p = new ompl::geometric::LBKPIECE1(si);
    else if (planner == "geometric::RRTStar")
        p = new ompl::geometric::RRTstar(si);
    else if (planner == "geometric::PRM")
        p = new ompl::geometric::PRM(si);
    else
        ROS_WARN("%s: Unknown planner type: %s", name_.c_str(), planner.c_str());
    if (p)
        p->params().setParams(config, true);
    return ompl::base::PlannerPtr(p);
}

ompl_interface::PlanningGroup::PlanningGroup(const std::string &name, const planning_models::KinematicModel::JointModelGroup *jmg,
                                             const std::map<std::string, std::string> &config, const planning_scene::PlanningSceneConstPtr &scene) :
    name_(name), jmg_(jmg), planning_scene_(scene), km_state_space_(jmg), ssetup_(km_state_space_.getOMPLSpace()),
    pplan_(ssetup_.getProblemDefinition()), start_state_(scene->getKinematicModel()), last_plan_time_(0.0),
    max_goal_samples_(10), max_sampling_attempts_(10000), max_planning_threads_(4)
{   
    max_solution_segment_length_ = ssetup_.getStateSpace()->getMaximumExtent() / 1000.0;
    ssetup_.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new StateValidityChecker(this)));
    ssetup_.getStateSpace()->setStateSamplerAllocator(boost::bind(&PlanningGroup::allocPathConstrainedSampler, this, _1));
    useConfig(config);
    path_kset_.reset(new kinematic_constraints::KinematicConstraintSet(planning_scene_->getKinematicModel(), planning_scene_->getTransforms()));
    goal_kset_.reset(new kinematic_constraints::KinematicConstraintSet(planning_scene_->getKinematicModel(), planning_scene_->getTransforms()));
}

ompl_interface::PlanningGroup::~PlanningGroup(void)
{
}

void ompl_interface::PlanningGroup::useConfig(const std::map<std::string, std::string> &config)
{
    if (config.empty())
        return;
    std::map<std::string, std::string> cfg = config;

    // set the projection evaluator
    std::map<std::string, std::string>::iterator it = cfg.find("projection_evaluator");
    if (it != cfg.end())
    {
        setProjectionEvaluator(boost::trim_copy(it->second));
        cfg.erase(it);
    }

    if (cfg.empty())
        return;

    it = cfg.find("type");
    if (it == cfg.end())
        ROS_WARN("%s: Attribute 'type' not specified in planner configuration", name_.c_str());
    else
    {
        // remove the 'type' parameter; the rest are parameters for the planner itself
        std::string type = it->second;
        cfg.erase(it);
        ssetup_.setPlannerAllocator(boost::bind(&PlanningGroup::plannerAllocator, this, _1, type, cfg));
        ROS_INFO("Planner configuration '%s' will use planner '%s'. Additional configuration parameters will be set when the planner is constructed.",
                 name_.c_str(), type.c_str());
    }

    ssetup_.getSpaceInformation()->setup();
    ssetup_.getSpaceInformation()->params().setParams(cfg, true);
}

void ompl_interface::PlanningGroup::setProjectionEvaluator(const std::string &peval)
{
    if (peval.find_first_of("link(") == 0 && peval[peval.length() - 1] == ')')
    {
        std::string link_name = peval.substr(5, peval.length() - 6);
        if (planning_scene_->getKinematicModel()->hasLinkModel(link_name))
            ssetup_.getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new ProjectionEvaluatorLinkPose(this, link_name)));
        else
            ROS_ERROR("Attempted to set projection evaluator with respect to position of link '%s', but that link is not known to the kinematic model.", link_name.c_str());
    }
    else
        if (peval.find_first_of("joints(") == 0 && peval[peval.length() - 1] == ')')
        {
            std::string joints = peval.substr(7, peval.length() - 8);
            boost::replace_all(joints, ",", " ");
            std::vector<std::pair<std::string, unsigned int> > j;
            std::stringstream ss(joints);
            while (ss.good() && !ss.eof())
            {
                std::string v; ss >> v >> std::ws;
                if (planning_scene_->getKinematicModel()->hasJointModel(v))
                {
                    unsigned int vc = planning_scene_->getKinematicModel()->getJointModel(v)->getVariableCount();
                    if (vc > 0)
                        j.push_back(std::make_pair(v, vc));
                    else
                        ROS_WARN("%s: Ignoring joint '%s' in projection since it has 0 DOF", name_.c_str(), v.c_str());
                }
                else
                    ROS_ERROR("%s: Attempted to set projection evaluator with respect to value of joint '%s', but that joint is not known to the kinematic model.",
                              name_.c_str(), v.c_str());
            }
            if (j.empty())
                ROS_ERROR("%s: No valid joints specified for joint projection", name_.c_str());
            else
                ssetup_.getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new ProjectionEvaluatorJointValue(this, j)));
        }
        else
            ROS_ERROR("Unable to allocate projection evaluator based on description: '%s'", peval.c_str());
}

ompl::base::StateSamplerPtr ompl_interface::PlanningGroup::allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const
{
    if (km_state_space_.getOMPLSpace().get() != ss)
        ROS_FATAL("%s: Attempted to allocate a state sampler for an unknown state space", name_.c_str());
    ROS_DEBUG("%s: Allocating a new state sampler (attempts to use path constraints)", name_.c_str());
    const kinematic_constraints::ConstraintSamplerPtr &cs = getConstraintsSampler(path_constraints_);
    if (cs)
        return ompl::base::StateSamplerPtr(new ConstrainedSampler(this, cs));
    else
        return ss->allocDefaultStateSampler();
}

kinematic_constraints::ConstraintSamplerPtr ompl_interface::PlanningGroup::getConstraintsSampler(const moveit_msgs::Constraints &constr) const
{
    // based on the goal constraints, decide on a goal representation
    kinematic_constraints::ConstraintSamplerPtr sampler;

    // if we have position and/or orientation constraints on links that we can perform IK for,
    // we will use a sampleable goal region that employs IK to sample goals
    for (std::size_t p = 0 ; p < constr.position_constraints.size() ; ++p)
        if (ik_allocator_)
            for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
                if (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name)
                {
                    boost::scoped_ptr<kinematic_constraints::PositionConstraint> pc
                        (new kinematic_constraints::PositionConstraint(planning_scene_->getKinematicModel(), planning_scene_->getTransforms()));
                    boost::scoped_ptr<kinematic_constraints::OrientationConstraint> oc
                        (new kinematic_constraints::OrientationConstraint(planning_scene_->getKinematicModel(), planning_scene_->getTransforms()));
                    if (pc->use(constr.position_constraints[p]) && oc->use(constr.orientation_constraints[o]))
                    {
                        sampler.reset(new kinematic_constraints::IKConstraintSampler(ik_allocator_, jmg_, *pc, *oc));
                        ROS_DEBUG("%s: Allocated an IK-based sampler satisfying position and orientation constraints", name_.c_str());
                    }
                }

    if (!sampler)
        for (std::size_t p = 0 ; p < constr.position_constraints.size() ; ++p)
            if (ik_allocator_)
            {
                boost::scoped_ptr<kinematic_constraints::PositionConstraint> pc
                    (new kinematic_constraints::PositionConstraint(planning_scene_->getKinematicModel(), planning_scene_->getTransforms()));
                if (pc->use(constr.position_constraints[p]))
                {
                    sampler.reset(new kinematic_constraints::IKConstraintSampler(ik_allocator_, jmg_, *pc));
                    ROS_DEBUG("%s: Allocated an IK-based sampler satisfying position constraints", name_.c_str());
                }
            }

    if (!sampler)
        for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
            if (ik_allocator_)
            {
                boost::scoped_ptr<kinematic_constraints::OrientationConstraint> oc
                    (new kinematic_constraints::OrientationConstraint(planning_scene_->getKinematicModel(), planning_scene_->getTransforms()));
                if (oc->use(constr.orientation_constraints[o]))
                {
                    sampler.reset(new kinematic_constraints::IKConstraintSampler(ik_allocator_, jmg_, *oc));
                    ROS_DEBUG("%s: Allocated an IK-based sampler satisfying orientation constraints", name_.c_str());
                }
            }

    // if we cannot perform IK but there are joint constraints that we can use to construct goal samples,
    // we again use a sampleable goal region
    if (!sampler && !constr.joint_constraints.empty())
    {
        std::vector<kinematic_constraints::JointConstraint> jc;
        for (std::size_t i = 0 ; i < constr.joint_constraints.size() ; ++i)
        {
            kinematic_constraints::JointConstraint j(planning_scene_->getKinematicModel(), planning_scene_->getTransforms());
            if (j.use(constr.joint_constraints[i]))
                jc.push_back(j);
        }
        if (!jc.empty())
        {
            sampler.reset(new kinematic_constraints::JointConstraintSampler(jmg_, jc));
            ROS_DEBUG("%s: Allocated a sampler satisfying joint constraints", name_.c_str());
        }
    }

    if (!sampler)
        ROS_DEBUG("%s: No constraints sampler allocated", name_.c_str());

    return sampler;
}

void ompl_interface::PlanningGroup::setPlanningVolume(const moveit_msgs::WorkspaceParameters &wparams)
{
    ROS_DEBUG("%s: Setting planning volume (affects SE2 & SE3 joints only) to x = [%f, %f], y = [%f, %f], z = [%f, %f]", name_.c_str(),
              wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y, wparams.max_corner.y, wparams.min_corner.z, wparams.max_corner.z);

    km_state_space_.setPlanningVolume(wparams.min_corner.x, wparams.max_corner.x,
                                      wparams.min_corner.y, wparams.max_corner.y,
                                      wparams.min_corner.z, wparams.max_corner.z);
}

bool ompl_interface::PlanningGroup::setupPlanningContext(const planning_models::KinematicState &start_state,
                                                         const moveit_msgs::Constraints &goal_constraints,
                                                         const moveit_msgs::Constraints &path_constraints,
                                                         moveit_msgs::MoveItErrorCodes *error)
{
    // ******************* check if the input is correct

    // first we need to identify what kind of planning we will perform
    if (goal_constraints.joint_constraints.empty() &&
        goal_constraints.position_constraints.empty() &&
        goal_constraints.orientation_constraints.empty())
    {
        ROS_WARN("%s: No goal constraints specified. There is no problem to solve.", name_.c_str());
        if (error)
            error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
    }

    // ******************* set up the starting state for the plannig context
    // set the starting state
    start_state_ = start_state;

    // notify the state validity checker about the new starting state
    static_cast<StateValidityChecker*>(ssetup_.getStateValidityChecker().get())->updatePlanningContext();

    // convert the input state to the corresponding OMPL state
    ompl::base::ScopedState<> ompl_start_state(km_state_space_.getOMPLSpace());
    km_state_space_.copyToOMPLState(ompl_start_state.get(), start_state_);
    ssetup_.setStartState(ompl_start_state);

    // ******************* set the path constraints to use
    path_constraints_ = path_constraints;
    path_kset_->clear();
    path_kset_->add(path_constraints);

    // ******************* set up the goal representation, based on goal constraints

    // first, we add path constraints to the goal ones
    goal_constraints_ = kinematic_constraints::mergeConstraints(goal_constraints, path_constraints);
    goal_kset_->clear();
    goal_kset_->add(goal_constraints_);
    const kinematic_constraints::ConstraintSamplerPtr &gsampler = getConstraintsSampler(goal_constraints_);
    if (gsampler)
    {
        ROS_DEBUG("%s: Using an OMPL GoalSampleableRegion with constraints", name_.c_str());
        // we can sample from the constraint specification
        ssetup_.setGoal(ompl::base::GoalPtr(new ConstrainedGoalSampler(this, goal_kset_, gsampler)));
    }
    else
    {
        ROS_DEBUG("%s: Using an OMPL GoalRegion with constraints'", name_.c_str());
        // the constraints are such that we cannot easily sample the goal region,
        // so we use a simpler goal region specification
        ssetup_.setGoal(ompl::base::GoalPtr(new ConstrainedGoalRegion(this, goal_kset_)));
    }

    ROS_DEBUG("%s: New planning context is set.", name_.c_str());

    return true;
}

bool ompl_interface::PlanningGroup::solve(double timeout, unsigned int count)
{
    ssetup_.getGoal()->clearSolutionPaths();
    bool gls = ssetup_.getGoal()->hasType(ompl::base::GOAL_LAZY_SAMPLES);
    // just in case sampling is not started
    if (gls)
        static_cast<ompl::base::GoalLazySamples*>(ssetup_.getGoal().get())->startSampling();

    // try to fix invalid input states, if any
    double d = ssetup_.getStateSpace()->getMaximumExtent() / 1000.0;
    if (!ssetup_.getProblemDefinition()->fixInvalidInputStates(d, d, 1000))
        ssetup_.getProblemDefinition()->fixInvalidInputStates(d * 10.0, d * 10.0, 1000);
    bool result = false;
    if (count <= 1)
    {
        ROS_DEBUG("%s: Solving the planning problem once...", name_.c_str());
        result = ssetup_.solve(timeout);
        last_plan_time_ = ssetup_.getLastPlanComputationTime();
    }
    else
    {
        ROS_DEBUG("%s: Solving the planning problem %u times...", name_.c_str(), count);
        pplan_.clearHybridizationPaths();
        if (count <= max_planning_threads_)
        {
            pplan_.clearPlanners();
            if (ssetup_.getPlannerAllocator())
                for (unsigned int i = 0 ; i < count ; ++i)
                    pplan_.addPlannerAllocator(ssetup_.getPlannerAllocator());
            else
                for (unsigned int i = 0 ; i < count ; ++i)
                    pplan_.addPlanner(ompl::geometric::getDefaultPlanner(ssetup_.getGoal()));
            ompl::time::point start = ompl::time::now();
            result = pplan_.solve(timeout, 1, count, true);
            last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
        }
        else
        {
            ompl::time::point start = ompl::time::now();
            int n = count / max_planning_threads_;
            result = true;
            for (int i = 0 ; i < n ; ++i)
            {
                pplan_.clearPlanners();
                if (ssetup_.getPlannerAllocator())
                    for (unsigned int i = 0 ; i < max_planning_threads_ ; ++i)
                        pplan_.addPlannerAllocator(ssetup_.getPlannerAllocator());
                else
                    for (unsigned int i = 0 ; i < max_planning_threads_ ; ++i)
                        pplan_.addPlanner(ompl::geometric::getDefaultPlanner(ssetup_.getGoal()));
                bool r = pplan_.solve(timeout, 1, max_planning_threads_, true);
                result = result && r;
            }
            n = count % max_planning_threads_;
            if (n)
            {
                pplan_.clearPlanners();
                if (ssetup_.getPlannerAllocator())
                    for (int i = 0 ; i < n ; ++i)
                        pplan_.addPlannerAllocator(ssetup_.getPlannerAllocator());
                else
                    for (int i = 0 ; i < n ; ++i)
                        pplan_.addPlanner(ompl::geometric::getDefaultPlanner(ssetup_.getGoal()));
                bool r = pplan_.solve(timeout, 1, n, true);
                result = result && r;
            }
            last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
        }
    }

    if (gls)
        // just in case we need to stop sampling
        static_cast<ompl::base::GoalLazySamples*>(ssetup_.getGoal().get())->stopSampling();

    return result;
}

void ompl_interface::PlanningGroup::simplifySolution(double timeout)
{
    ssetup_.simplifySolution(timeout);
}

void ompl_interface::PlanningGroup::interpolateSolution(void)
{
    if (ssetup_.haveSolutionPath())
    {
	ompl::geometric::PathGeometric &pg = ssetup_.getSolutionPath();
	pg.interpolate((std::size_t)floor(0.5 + pg.length() / max_solution_segment_length_));
    }
}

bool ompl_interface::PlanningGroup::getSolutionPath(moveit_msgs::RobotTrajectory &traj) const
{
    if (!ssetup_.haveSolutionPath())
        return false;

    const ompl::geometric::PathGeometric &pg = ssetup_.getSolutionPath();
    planning_models::KinematicState ks = start_state_;
    const std::vector<planning_models::KinematicModel::JointModel*> &jnt = planning_scene_->getKinematicModel()->getJointModels();
    std::vector<const planning_models::KinematicModel::JointModel*> onedof;
    std::vector<const planning_models::KinematicModel::JointModel*> mdof;
    traj.joint_trajectory.header.frame_id = planning_scene_->getKinematicModel()->getModelFrame();
    for (std::size_t i = 0 ; i < jnt.size() ; ++i)
        if (jnt[i]->getVariableCount() == 1)
        {
            traj.joint_trajectory.joint_names.push_back(jnt[i]->getName());
            onedof.push_back(jnt[i]);
        }
            else
        {
            traj.multi_dof_joint_trajectory.joint_names.push_back(jnt[i]->getName());
            traj.multi_dof_joint_trajectory.frame_ids.push_back(planning_scene_->getKinematicModel()->getModelFrame());
            traj.multi_dof_joint_trajectory.child_frame_ids.push_back(jnt[i]->getChildLinkModel()->getName());
            mdof.push_back(jnt[i]);
        }
    if (!onedof.empty())
        traj.joint_trajectory.points.resize(pg.states.size());
    if (!mdof.empty())
        traj.multi_dof_joint_trajectory.points.resize(pg.states.size());
    for (std::size_t i = 0 ; i < pg.states.size() ; ++i)
    {
        km_state_space_.copyToKinematicState(ks, pg.states[i]);
        if (!onedof.empty())
        {
            traj.joint_trajectory.points[i].positions.resize(onedof.size());
            for (std::size_t j = 0 ; j < onedof.size() ; ++j)
                traj.joint_trajectory.points[i].positions[j] = ks.getJointState(onedof[j]->getName())->getVariableValues()[0];
            traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
        }
        if (!mdof.empty())
        {
            traj.multi_dof_joint_trajectory.points[i].poses.resize(mdof.size());
            for (std::size_t j = 0 ; j < mdof.size() ; ++j)
            {
                planning_models::msgFromPose(ks.getJointState(mdof[j]->getName())->getVariableTransform(),
                                             traj.multi_dof_joint_trajectory.points[i].poses[j]);
            }
            traj.multi_dof_joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
        }
    }
    return true;
}

void ompl_interface::PlanningGroup::fillResponse(moveit_msgs::GetMotionPlan::Response &res) const
{
    ROS_DEBUG("%s: Returning successful solution with %lu states", name_.c_str(), ssetup_.getSolutionPath().states.size());
    planning_models::kinematicStateToRobotState(start_state_, res.robot_state);
    res.planning_time = ros::Duration(last_plan_time_);
    getSolutionPath(res.trajectory);
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
}
