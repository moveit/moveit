/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

// ROS msgs
#include <moveit/kinematics_planner/kinematics_planner.h>
#include <moveit/kinematics_planner/kinematics_solver.h>
#include <moveit/kinematic_model/kinematic_model.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

static const double DISCRETIZATION_TRANSLATION = 0.001;
static const double DISCRETIZATION_ROTATION = 0.001;

namespace kinematics_planner
{

bool KinematicsPlanner::initialize(const std::vector<std::string> &group_names, const kinematic_model::KinematicModelConstPtr &model)
{
    /* should get these from ROS params */
    discretization_translation_ = DISCRETIZATION_TRANSLATION;
    discretization_rotation_ = DISCRETIZATION_ROTATION;

    /* assumes that the KinematicsModel does not change once this object is created */
    KinematicsSolverPtr solver = boost::make_shared<KinematicsSolver>();
    solver->initialize(model);
    kinematics_solver_ = solver;

    return true;
}

bool KinematicsPlanner::checkRequest(const std::map<std::string,geometry_msgs::PoseStamped> &request) const
{
    /* FIXME needs to be filled in -jbinney */

    return true;
}


bool KinematicsPlanner::solve(const std::map<std::string,geometry_msgs::PoseStamped> &start_request,
                              const std::map<std::string,geometry_msgs::PoseStamped> &goal_request,
                              const planning_scene::PlanningSceneConstPtr& planning_scene,
                              const moveit_msgs::Constraints &path_constraints,
                              double timeout,
                              moveit_msgs::RobotTrajectory &robot_trajectory,
                              moveit_msgs::MoveItErrorCodes &error_code) const
{
    kinematic_constraints::KinematicConstraintSet kinematic_constraints_set(planning_scene->getKinematicModel(),planning_scene->getTransforms());
    kinematic_constraints_set.add(path_constraints);
    return solve(start_request,goal_request,planning_scene,kinematic_constraints_set,timeout,robot_trajectory,error_code);
}

bool KinematicsPlanner::solve(const std::map<std::string,geometry_msgs::PoseStamped> &start_request,
                              const std::map<std::string,geometry_msgs::PoseStamped> &goal_request,
                              const planning_scene::PlanningSceneConstPtr& planning_scene,
                              const kinematic_constraints::KinematicConstraintSet& kinematic_constraint_set,
                              double timeout,
                              moveit_msgs::RobotTrajectory &robot_trajectory,
                              moveit_msgs::MoveItErrorCodes &error_code) const
{
    if(!kinematics_solver_)
    {
        // ROS_ERROR("KinematicsPlanner called without being initialized");
        return false;
    }

    ros::WallTime start_time = ros::WallTime::now();
    if(!checkRequest(start_request) || !checkRequest(goal_request))
    {
        error_code.val = error_code.INVALID_GROUP_NAME;
        return false;
    }

    /* make sure that we have goal positions for all groups which have start positions */
    robot_state::RobotState kinematic_state = planning_scene->getCurrentState();
    std::vector<std::string> group_names;
    std::map<std::string, robot_state::JointStateGroup*> joint_state_groups;
    std::map<std::string,geometry_msgs::PoseStamped>::const_iterator group_iter;
    for(group_iter = start_request.begin(); group_iter != start_request.end(); ++group_iter)
    {
        std::string group_name = group_iter->first;
        if(goal_request.find(group_name) == goal_request.end())
        {
            error_code.val = error_code.INVALID_GROUP_NAME;
            return false;
        }
        group_names.push_back(group_name);
        joint_state_groups[group_name] = kinematic_state.getJointStateGroup(group_name);
    }

    /* transform all of the start and goal poses into the root frame of the kinematic model */
    std::string model_frame = kinematic_state.getKinematicModel()->getModelFrame();
    std::map<std::string, std::string> kinematics_base_frames;
    for(group_iter = start_request.begin(); group_iter != start_request.end(); ++group_iter)
    {
        std::string group_name = group_iter->first;
        kinematics_base_frames[group_name] = model_frame;
    }
    std::map<std::string, geometry_msgs::PoseStamped> start = transformPoses(planning_scene, kinematic_state, start_request, kinematics_base_frames);
    std::map<std::string, geometry_msgs::PoseStamped> goal = transformPoses(planning_scene, kinematic_state, goal_request, kinematics_base_frames);

    /* interpolate between start and goal cartesian poses */
    std::map<std::string, std::vector<geometry_msgs::Pose> > interpolated_poses = getInterpolatedPosesMap(start, goal);
    unsigned int num_poses = interpolated_poses[group_names[0]].size();

    /* pre-allocate space for solution joint values */
    kinematics_planner::SolutionTrajectoryMap solutions;
    for(group_iter = start_request.begin(); group_iter != start_request.end(); ++group_iter)
    {
        std::string group_name = group_iter->first;
        solutions[group_name].resize(num_poses);
        for(unsigned int j=0; j < num_poses; j++)
            solutions[group_name][j].resize(joint_state_groups[group_name]->getVariableCount());
    }

    const std::map<std::string, unsigned int> joint_indices_map = planning_scene->getKinematicModel()->getJointVariablesIndexMap();

    /* build the request that we'll use for IK calls */
    moveit_msgs::GetConstraintAwarePositionIK::Request request;
    moveit_msgs::GetConstraintAwarePositionIK::Response response;
    request.ik_request.pose_stamped.header.frame_id = kinematic_state.getKinematicModel()->getModelFrame();
    request.ik_request.robot_state.joint_state.name = planning_scene->getKinematicModel()->getJointModelNames();
    request.timeout = ros::Duration(1.0);
    planning_scene->getCurrentState().getStateValues(request.ik_request.robot_state.joint_state.position);
    request.constraints = kinematic_constraint_set.getAllConstraints();

    ros::WallDuration elapsed_time = ros::WallTime::now() - start_time;

    while(elapsed_time <= ros::WallDuration(timeout))
    {
        bool success = true;
        for(group_iter = start_request.begin(); group_iter != start_request.end(); ++group_iter)
            joint_state_groups[group_iter->first]->setToRandomValues();

        for(unsigned int pose_i = 0; pose_i < num_poses; ++pose_i)
        {
            for(group_iter = start_request.begin(); group_iter != start_request.end(); ++group_iter)
            {
                std::string group_name = group_iter->first;
                request.ik_request.group_name = group_name;

                request.ik_request.pose_stamped.pose = interpolated_poses[group_name][pose_i];

                std::vector<double> joint_state_values;
                joint_state_groups[group_name]->getVariableValues(joint_state_values);

                /* solve IK for this pose */
                if(!kinematics_solver_->getIK(planning_scene, request, response))
                {
                    success = false;
                    // ROS_INFO("Pose %d: IK Failed with error code %d", pose_i, response.error_code.val);
                    error_code.val = error_code.PLANNING_FAILED;
                    break;
                }
                if(response.error_code.val != response.error_code.SUCCESS)
                {
                    success = false;
                    error_code.val = error_code.PLANNING_FAILED;
                    break;
                }

                /* use the kinematic solution from this IK step as the seed for the next IK step. */
                const std::vector<std::string> group_joint_names = joint_state_groups[group_name]->getJointNames();
                for(unsigned int response_joint_i = 0; response_joint_i < response.solution.joint_state.name.size(); response_joint_i++)
                {
                    const std::string joint_name = response.solution.joint_state.name[response_joint_i];
                    const unsigned int joint_i = joint_indices_map.find(joint_name)->second;

                    const double joint_val = response.solution.joint_state.position[response_joint_i];
                    request.ik_request.robot_state.joint_state.position[joint_i] = joint_val;

                    /* fill in solution */
                    solutions[group_name][pose_i][response_joint_i] = joint_val;
                }
            }

            if(!success)
                break;

            if(planning_scene->isStateColliding(kinematic_state))
            {
                error_code.val = error_code.GOAL_IN_COLLISION;
                success = false;
                break;
            }
            if(!planning_scene->isStateConstrained(kinematic_state,kinematic_constraint_set))
            {
                error_code.val = error_code.GOAL_IN_COLLISION;
                success = false;
                break;
            }
        }

        if(success)
        {

            /* populate the vector of joint names */
            for(unsigned int group_i = 0; group_i < group_names.size(); group_i++)
            {
                const std::string group_name = group_names[group_i];
                const std::vector<std::string> group_joint_names = joint_state_groups[group_name]->getJointNames();
                for(unsigned int group_joint_i = 0; group_joint_i < group_joint_names.size(); group_joint_i++)
                {
                    const std::string joint_name = group_joint_names[group_joint_i];
                    robot_trajectory.joint_trajectory.joint_names.push_back(joint_name);
                }
            }

            /* fill in the joint values */
            robot_trajectory.joint_trajectory.points.resize(num_poses);
            // ROS_INFO("Interpolated IK trajectory has %d points", num_poses);
            for(unsigned int i = 0; i < robot_trajectory.joint_trajectory.points.size(); ++i)
            {
                // ROS_INFO("Filling in point %d", i);
                for(unsigned int group_i = 0; group_i < group_names.size(); group_i++)
                {
                    const std::string group_name = group_names[group_i];
                    const std::vector<double>& group_solutions = (solutions.find(group_name)->second)[i];
                    robot_trajectory.joint_trajectory.points[i].positions.insert(robot_trajectory.joint_trajectory.points[i].positions.end(), group_solutions.begin(), group_solutions.end());
                }
            }
            return true;
        }
        elapsed_time = ros::WallTime::now()-start_time;
    }

    return false;
}

std::map<std::string, std::vector<geometry_msgs::Pose> > KinematicsPlanner::getInterpolatedPosesMap(const std::map<std::string,geometry_msgs::PoseStamped> &start,
                                                                                                    const std::map<std::string,geometry_msgs::PoseStamped> &goal) const
{
    /* calculate the number of interpolated poses we need */
    unsigned int num_segments = 0;
    std::map<std::string,geometry_msgs::PoseStamped>::const_iterator group_iter;
    for(group_iter = start.begin(); group_iter != start.end(); ++group_iter)
    {
        std::string group_name = group_iter->first;
        unsigned int group_segments = getNumSegments(start.find(group_name)->second.pose, goal.find(group_name)->second.pose);
        if(group_segments > num_segments)
            num_segments = group_segments;
    }

    /* at least one segment in the trajectory */
    if(num_segments < 1)
        num_segments = 1;

    /* interpolate poses for each group */
    std::map<std::string,std::vector<geometry_msgs::Pose> > result;
    for(group_iter = start.begin(); group_iter != start.end(); ++group_iter)
    {
        std::string group_name = group_iter->first;

        result[group_name] = getInterpolatedPoses(start.find(group_name)->second.pose,
                                                  goal.find(group_name)->second.pose,
                                                  num_segments);
    }

    return result;
}

unsigned int KinematicsPlanner::getNumSegments(const geometry_msgs::Pose &start,
                                               const geometry_msgs::Pose &goal) const
{
    kinematic_model::FloatingJointModel floating_joint("end_effector");
    std::vector<double> start_values = getFloatingJointValues(start);
    std::vector<double> goal_values = getFloatingJointValues(goal);

    double translation_distance = floating_joint.distanceTranslation(start_values,goal_values);
    double rotation_angle = fabs(floating_joint.distanceRotation(start_values,goal_values));

    unsigned int num_segments = std::ceil(translation_distance/discretization_translation_);
    unsigned int num_rotation_segments = std::ceil(rotation_angle/discretization_rotation_);
    if(num_rotation_segments > num_segments)
        num_segments = num_rotation_segments;
    return num_segments;
}

std::vector<geometry_msgs::Pose> KinematicsPlanner::getInterpolatedPoses(const geometry_msgs::Pose &start,
                                                                         const geometry_msgs::Pose &goal,
                                                                         const unsigned int num_segments) const
{
    kinematic_model::FloatingJointModel floating_joint("end_effector");
    std::vector<double> start_values = getFloatingJointValues(start);
    std::vector<double> goal_values = getFloatingJointValues(goal);

    std::vector<geometry_msgs::Pose> interp_poses;
    interp_poses.resize(num_segments+1);
    unsigned int pose_i;
    for(pose_i = 0; pose_i < num_segments; ++pose_i)
    {
        double t = ((double) pose_i)/num_segments;
        std::vector<double> state(7);
        floating_joint.interpolate(start_values, goal_values, t, state);
        geometry_msgs::Pose pose = getPose(state);
        interp_poses[pose_i] = pose;
    }
    interp_poses[pose_i] = goal;
    return interp_poses;
}

}
