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
        ROS_WARN("Unknown planner type: %s", planner.c_str());
    if (p)
        p->setParams(config);
    return ompl::base::PlannerPtr(p);
}

ompl_interface::PlanningGroup::PlanningGroup(const std::string &name, const planning_models::KinematicModel::JointModelGroup *jmg,
                                             const std::map<std::string, std::string> &config,
                                             const planning_scene::PlanningScenePtr &scene, ompl::StateSpaceCollection &ssc) :
    name_(name), jmg_(jmg), planning_scene_(scene), km_state_space_(ssc, jmg), planning_context_(km_state_space_),
    max_goal_samples_(10), max_sampling_attempts_(10000)
{
    planning_context_.start_state_.reset(new planning_models::KinematicState(planning_scene_->getKinematicModel()));
    planning_context_.ssetup_.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new StateValidityChecker(this)));
    planning_context_.ssetup_.getStateSpace()->setStateSamplerAllocator(boost::bind(&PlanningGroup::allocPathConstrainedSampler, this, _1));
    if (!config.empty())
    {
        std::map<std::string, std::string>::const_iterator it = config.find("type");
        if (it == config.end())
            ROS_WARN("Attribute 'type' not specified for planning group '%s'", name_.c_str());
        else
        {
            std::map<std::string, std::string> rest = config; rest.erase("type");
            planning_context_.ssetup_.setPlannerAllocator(boost::bind(&PlanningGroup::plannerAllocator, this, _1, name, rest));
        }
    }
}

ompl_interface::PlanningGroup::~PlanningGroup(void)
{
}

namespace ompl_interface
{

    // we could do this without defining a new class, but this way we have a nice container for the constraint sampler as well
    class ConstrainedGoalSampler : public ompl::base::GoalLazySamples
    {
    public:
        ConstrainedGoalSampler(const PlanningGroup *pg, const kinematic_constraints::KinematicConstraintSetPtr &ks, const kinematic_constraints::ConstraintSamplerPtr &cs) :
            ompl::base::GoalLazySamples(pg->getPlanningContext().ssetup_.getSpaceInformation(), boost::bind(&ConstrainedGoalSampler::sampleC, this, _1, _2), false),
            pg_(pg), ks_(ks), cs_(cs), tss_(*pg->getPlanningContext().start_state_)
        {
            startSampling();
        }

    private:

        bool sampleC(const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
        {
            unsigned int ma = pg_->getMaximumSamplingAttempts();

            // terminate after too many attempts
            if (gls->samplingAttemptsCount() >= ma)
                return false;
            // terminate after a maximum number of samples
            if (gls->getStateCount() >= pg_->getMaximumGoalSamples())
                return false;
            // terminate the sampling thread when a solution has been found
            if (gls->isAchieved())
                return false;

            planning_models::KinematicState *s = tss_.getStateStorage();
            std::vector<double> values;
            for (unsigned int a = 0 ; a < ma && gls->isSampling() ; ++a)
                if (cs_->sample(values, ma, pg_->getPlanningContext().start_state_.get()))
                {
                    s->getJointStateGroup(pg_->getJointModelGroup()->getName())->setStateValues(values);
                    if (ks_->decide(*s).first)
                    {
                        pg_->getKMStateSpace().copyToOMPLState(newGoal, values);
                        return true;
                    }
                }
            return false;
        }

        const PlanningGroup                             *pg_;
        kinematic_constraints::KinematicConstraintSetPtr ks_;
        kinematic_constraints::ConstraintSamplerPtr      cs_;
        TSStateStorage                                   tss_;
    };

    class ConstrainedGoalRegion : public ompl::base::GoalRegion
    {
    public:
        ConstrainedGoalRegion(const PlanningGroup *pg, const kinematic_constraints::KinematicConstraintSetPtr &ks) :
            ompl::base::GoalRegion(pg->getPlanningContext().ssetup_.getSpaceInformation()), pg_(pg), ks_(ks),
            tss_(*pg->getPlanningContext().start_state_)
        {
        }

        virtual ~ConstrainedGoalRegion(void)
        {
        }

        virtual double distanceGoal(const ompl::base::State *st) const
        {
            planning_models::KinematicState *s = tss_.getStateStorage();
            pg_->getKMStateSpace().copyToKinematicState(*s, st);
            return ks_->decide(*s).second;
        }

        virtual bool isSatisfied(const ompl::base::State *st, double *distance) const
        {
            planning_models::KinematicState *s = tss_.getStateStorage();
            pg_->getKMStateSpace().copyToKinematicState(*s, st);
            const std::pair<bool, double> &r = ks_->decide(*s);
            if (distance)
                *distance = r.second;
            return r.first;
        }

    protected:

        const PlanningGroup                             *pg_;
        kinematic_constraints::KinematicConstraintSetPtr ks_;
        TSStateStorage                                   tss_;
    };

    class ConstrainedSampler : public ompl::base::StateSampler
    {
    public:
        ConstrainedSampler(const PlanningGroup *pg, const kinematic_constraints::ConstraintSamplerPtr &cs) :
            ompl::base::StateSampler(pg->getKMStateSpace().getOMPLSpace().get()),
            pg_(pg), default_(space_->allocDefaultStateSampler()), cs_(cs)
        {
        }

        bool sampleC(ompl::base::State *state)
        {
            std::vector<double> values;
            if (cs_->sample(values, pg_->getMaximumSamplingAttempts(), pg_->getPlanningContext().start_state_.get()))
            {
                pg_->getKMStateSpace().copyToOMPLState(state, values);
                return true;
            }
            return false;
        }

        virtual void sampleUniform(ompl::base::State *state)
        {
            if (!sampleC(state))
                default_->sampleUniform(state);
        }

        virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
        {
            if (!sampleC(state))
                default_->sampleUniformNear(state, near, distance);
        }

        virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
        {
            if (!sampleC(state))
                default_->sampleGaussian(state, mean, stdDev);
        }

    private:

        const PlanningGroup                        *pg_;
        ompl::base::StateSamplerPtr                 default_;
        kinematic_constraints::ConstraintSamplerPtr cs_;
    };
}

ompl::base::StateSamplerPtr ompl_interface::PlanningGroup::allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const
{
    if (km_state_space_.getOMPLSpace().get() != ss)
        ROS_FATAL("Attempted to allocate a state sampler for an unknown state space");
    const kinematic_constraints::ConstraintSamplerPtr &cs = getConstraintsSampler(planning_context_.path_constraints_);
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
        if (jmg_->supportsIK(constr.position_constraints[p].link_name))
            for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
                if (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name)
                {
                    boost::scoped_ptr<kinematic_constraints::PositionConstraint> pc
                        (new kinematic_constraints::PositionConstraint(*planning_scene_->getKinematicModel(), *planning_scene_->getTransforms()));
                    boost::scoped_ptr<kinematic_constraints::OrientationConstraint> oc
                        (new kinematic_constraints::OrientationConstraint(*planning_scene_->getKinematicModel(), *planning_scene_->getTransforms()));
                    if (pc->use(constr.position_constraints[p]) && oc->use(constr.orientation_constraints[o]))
                        sampler.reset(new kinematic_constraints::IKConstraintSampler(ik_allocator_, jmg_, *pc, *oc));
                }

    if (!sampler)
        for (std::size_t p = 0 ; p < constr.position_constraints.size() ; ++p)
            if (jmg_->supportsIK(constr.position_constraints[p].link_name))
            {
                boost::scoped_ptr<kinematic_constraints::PositionConstraint> pc
                    (new kinematic_constraints::PositionConstraint(*planning_scene_->getKinematicModel(), *planning_scene_->getTransforms()));
                if (pc->use(constr.position_constraints[p]))
                    sampler.reset(new kinematic_constraints::IKConstraintSampler(ik_allocator_, jmg_, *pc));
            }

    if (!sampler)
        for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
            if (jmg_->supportsIK(constr.orientation_constraints[o].link_name))
            {
                boost::scoped_ptr<kinematic_constraints::OrientationConstraint> oc
                    (new kinematic_constraints::OrientationConstraint(*planning_scene_->getKinematicModel(), *planning_scene_->getTransforms()));
                if (oc->use(constr.orientation_constraints[o]))
                    sampler.reset(new kinematic_constraints::IKConstraintSampler(ik_allocator_, jmg_, *oc));
            }

    // if we cannot perform IK but there are joint constraints that we can use to construct goal samples,
    // we again use a sampleable goal region
    if (!sampler && !constr.joint_constraints.empty())
    {
        std::vector<kinematic_constraints::JointConstraint> jc;
        for (std::size_t i = 0 ; i < constr.joint_constraints.size() ; ++i)
        {
            kinematic_constraints::JointConstraint j(*planning_scene_->getKinematicModel(), *planning_scene_->getTransforms());
            if (j.use(constr.joint_constraints[i]))
                jc.push_back(j);
        }
        if (!jc.empty())
            sampler.reset(new kinematic_constraints::JointConstraintSampler(jmg_, jc));
    }
    return sampler;
}

bool ompl_interface::PlanningGroup::setupPlanningContext(const planning_models::KinematicState &start_state,
                                                         const moveit_msgs::Constraints &goal_constraints,
                                                         const moveit_msgs::Constraints &path_constraints)
{
    // ******************* check if the input is correct

    // first we need to identify what kind of planning we will perform
    if (goal_constraints.joint_constraints.empty() &&
        goal_constraints.position_constraints.empty() &&
        goal_constraints.orientation_constraints.empty())
    {
        ROS_WARN("No goal constraints specified");
        return false;
    }

    // ******************* set up the starting state for the plannig context
    // set the starting state
    *planning_context_.start_state_ = start_state;

    // notify the state validity checker about the new starting state
    static_cast<StateValidityChecker*>(planning_context_.ssetup_.getStateValidityChecker().get())->updatePlanningContext();

    // convert the input state to the corresponding OMPL state
    ompl::base::ScopedState<> ompl_start_state(km_state_space_.getOMPLSpace());
    km_state_space_.copyToOMPLState(ompl_start_state.get(), *planning_context_.start_state_);
    planning_context_.ssetup_.setStartState(ompl_start_state);

    // ******************* set the path constraints to use
    planning_context_.path_constraints_ = path_constraints;

    // ******************* set up the goal representation, based on goal constraints

    // first, we add path constraints to the goal ones
    planning_context_.goal_constraints_ = kinematic_constraints::mergeConstraints(goal_constraints, path_constraints);

    // construct a constraint evaluator for the goal region
    kinematic_constraints::KinematicConstraintSetPtr ks
        (new kinematic_constraints::KinematicConstraintSet(planning_scene_->getKinematicModel(), planning_scene_->getTransforms()));
    ks->add(planning_context_.goal_constraints_);

    const kinematic_constraints::ConstraintSamplerPtr &gsampler = getConstraintsSampler(planning_context_.goal_constraints_);
    if (gsampler)
        // we can sample from the constraint specification
        planning_context_.ssetup_.setGoal(ompl::base::GoalPtr(new ConstrainedGoalSampler(this, ks, gsampler)));
    else
        // the constraints are such that we cannot easily sample the goal region,
        // so we use a simpler goal region specification
        planning_context_.ssetup_.setGoal(ompl::base::GoalPtr(new ConstrainedGoalRegion(this, ks)));

    return true;
}

bool ompl_interface::PlanningGroup::solve(double timeout, unsigned int count)
{
    return planning_context_.ssetup_.solve(timeout);
}

bool ompl_interface::PlanningGroup::getSolutionPath(moveit_msgs::RobotTrajectory &traj) const
{
    if (!planning_context_.ssetup_.haveSolutionPath())
        return false;

    const ompl::geometric::PathGeometric &pg = planning_context_.ssetup_.getSolutionPath();
    planning_models::KinematicState ks = *planning_context_.start_state_;
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
                const btTransform &t = ks.getJointState(mdof[j]->getName())->getVariableTransform();
                const btQuaternion &q = t.getRotation();
                geometry_msgs::Pose &p = traj.multi_dof_joint_trajectory.points[i].poses[j];
                p.position.x = t.getOrigin().getX(); p.position.y = t.getOrigin().getY(); p.position.z = t.getOrigin().getZ();
                p.orientation.x = q.getX(); p.orientation.y = q.getY(); p.orientation.z = q.getZ(); p.orientation.w = q.getW();
            }
            traj.multi_dof_joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
        }
    }
    return true;
}

void ompl_interface::PlanningGroup::fillResponse(moveit_msgs::GetMotionPlan::Response &res) const
{
    planning_models::kinematicStateToRobotState(*planning_context_.start_state_, res.robot_state);
    res.planning_time = ros::Duration(planning_context_.ssetup_.getLastPlanComputationTime());
    getSolutionPath(res.trajectory);
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
}
