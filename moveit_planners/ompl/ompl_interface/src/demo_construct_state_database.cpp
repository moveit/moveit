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

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/profiler/profiler.h>

#include <boost/math/constants/constants.hpp>

static const std::string ROBOT_DESCRIPTION = "robot_description";

moveit_msgs::Constraints getConstraints()
{
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = "torso_lift_link";
  ocm.orientation.x = 0;
  ocm.orientation.y = 0;
  ocm.orientation.z = 0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = boost::math::constants::pi<double>();
  ocm.weight = 1.0;
  moveit_msgs::Constraints cmsg;
  cmsg.orientation_constraints.resize(1, ocm);
  cmsg.name = ocm.link_name + ":upright";
  return cmsg;
}

void computeDB(const robot_model::RobotModelPtr &robot_model, unsigned int ns, unsigned int ne)
{
  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene(robot_model));
  ompl_interface::OMPLInterface ompl_interface(robot_model);
  moveit_msgs::Constraints c = getConstraints();
  ompl_interface::ConstraintApproximationConstructionOptions opt;
  opt.state_space_parameterization = "PoseModel";
  opt.samples = ns;
  opt.edges_per_sample = ne;
  opt.explicit_motions = true;
  opt.max_edge_length = 0.2;
  opt.explicit_points_resolution = 0.05;
  opt.max_explicit_points = 10;

  ompl_interface.getConstraintsLibrary().addConstraintApproximation(c, "right_arm", ps, opt);
  ompl_interface.getConstraintsLibrary().saveConstraintApproximations("~/constraints_approximation_database");
  ROS_INFO("Done");
}

/*
bool constructConstraintApproximation(moveit_msgs::ConstructConstraintApproximation::Request &req,
moveit_msgs::ConstructConstraintApproximation::Response &res)
{
  planning_scene::PlanningScenePtr diff_scene = psm_.getPlanningScene()->diff();
  robot_state::robotStateMsgToRobotState(*psm_.getPlanningScene()->getTransforms(), req.start_state,
diff_scene->getCurrentStateNonConst());
  ompl_interface::ConstraintApproximationConstructionResults ca_res =
    ompl_interface_.getConstraintsLibrary().addConstraintApproximation(req.constraint, req.group,
req.state_space_parameterization,
                                                                       diff_scene, req.samples, req.edges_per_sample);
  if (ca_res.approx)
  {
    res.sampling_success_rate = ca_res.sampling_success_rate;
    res.state_sampling_time = ca_res.state_sampling_time;
    res.state_connection_time = ca_res.state_connection_time;
    res.filename = ca_res.approx->getFilename();
    return ompl_interface_.saveConstraintApproximations();
  }
  else
    return false;
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "construct_ompl_state_database", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  unsigned int nstates = 1000;
  unsigned int nedges = 0;

  if (argc > 1)
    try
    {
      nstates = boost::lexical_cast<unsigned int>(argv[1]);
    }
    catch (...)
    {
    }

  if (argc > 2)
    try
    {
      nedges = boost::lexical_cast<unsigned int>(argv[2]);
    }
    catch (...)
    {
    }

  robot_model_loader::RobotModelLoader rml(ROBOT_DESCRIPTION);
  computeDB(rml.getModel(), nstates, nedges);

  ros::shutdown();
  return 0;
}
