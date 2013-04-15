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

/* Author: Ioan Sucan, Mario Prats */

#include <moveit/benchmarks/benchmark_execution.h>
#include <moveit/benchmarks/benchmarks_utils.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <boost/regex.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/progress.hpp>


#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>

namespace moveit_benchmarks
{

namespace
{

// update the constrained link for Position and Orientation constraints, if that link is empty
void checkConstrainedLink(moveit_msgs::Constraints &c, const std::string &link_name)
{
  for (std::size_t i = 0 ; i < c.position_constraints.size() ; ++i)
    if (c.position_constraints[i].link_name.empty())
      c.position_constraints[i].link_name = link_name;
  for (std::size_t i = 0 ; i < c.orientation_constraints.size() ; ++i)
    if (c.orientation_constraints[i].link_name.empty())
      c.orientation_constraints[i].link_name = link_name;
}

void checkHeader(moveit_msgs::Constraints &c, const std::string &header_frame)
{
  for (std::size_t i = 0 ; i < c.position_constraints.size() ; ++i)
    if (c.position_constraints[i].header.frame_id.empty())
    {
      c.position_constraints[i].header.frame_id = header_frame;
      c.position_constraints[i].header.stamp = ros::Time::now();
    }
  for (std::size_t i = 0 ; i < c.orientation_constraints.size() ; ++i)
    if (c.orientation_constraints[i].header.frame_id.empty())
    {
      c.orientation_constraints[i].header.frame_id = header_frame;
      c.orientation_constraints[i].header.stamp = ros::Time::now();
    }
}

}
}

moveit_benchmarks::BenchmarkExecution::BenchmarkExecution(const planning_scene::PlanningScenePtr &scene, const std::string &host, std::size_t port) :
  planning_scene_(scene),
  pss_(host, port), 
  psws_(host, port),
  cs_(host, port),
  rs_(host, port)
{    
  // load the planning plugins
  try
  {
    planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::Planner>("moveit_core", "planning_interface::Planner"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  
  const std::vector<std::string> &classes = planner_plugin_loader_->getDeclaredClasses();
  for (std::size_t i = 0 ; i < classes.size() ; ++i)
  {
    ROS_INFO("Attempting to load and configure %s", classes[i].c_str());
    try
    {
      boost::shared_ptr<planning_interface::Planner> p = planner_plugin_loader_->createInstance(classes[i]);
      p->initialize(planning_scene_->getRobotModel());
      planner_interfaces_[classes[i]] = p;
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while loading planner '" << classes[i] << "': " << ex.what());
    }
  }
  
  if (planner_interfaces_.empty())
    ROS_ERROR("No planning plugins have been loaded. Nothing to do for the benchmarking service.");
  else
  {
    std::stringstream ss;
    for (std::map<std::string, boost::shared_ptr<planning_interface::Planner> >::const_iterator it = planner_interfaces_.begin() ; 
         it != planner_interfaces_.end(); ++it)
      ss << it->first << " ";
    ROS_INFO("Available planner instances: %s", ss.str().c_str());
  }
}

void moveit_benchmarks::BenchmarkExecution::runAllBenchmarks(BenchmarkType type)
{
  moveit_warehouse::PlanningSceneWithMetadata pswm;
  moveit_warehouse::PlanningSceneWorldWithMetadata pswwm;
  bool world_only = false;

  // read the environment geometry 
  if (!pss_.hasPlanningScene(opt_.scene))
  {
    if (psws_.hasPlanningSceneWorld(opt_.scene))
    { 
      bool ok = false;
      try
      {
        ok = psws_.getPlanningSceneWorld(pswwm, opt_.scene);
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      if (!ok)
        return;
      world_only = true;
    }
    else
    {  
      std::stringstream ss;
      std::vector<std::string> names;
      pss_.getPlanningSceneNames(names);
      for (std::size_t i = 0 ; i < names.size() ; ++i)
        ss << names[i] << " ";
      ROS_ERROR("Scene '%s' not found in warehouse. Available names: ", opt_.scene.c_str(), ss.str().c_str());
      return;
    }
  }
  else
  {
    bool ok = false;
    try
    {
      ok = pss_.getPlanningScene(pswm, opt_.scene);
    }
    catch (std::runtime_error &ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    
    if (!ok)
    {
      ROS_ERROR("Scene '%s' not found in warehouse", opt_.scene.c_str());
      return;
    }
  }

  // fill in the environment geometry in the benchmark request
  BenchmarkRequest req;
  req.benchmark_type = type;
  
  if (world_only)
  {
    req.scene.world = static_cast<const moveit_msgs::PlanningSceneWorld&>(*pswwm);
    req.scene.robot_model_name = "NO ROBOT INFORMATION. ONLY WORLD GEOMETRY"; // so that run_benchmark sees a different robot name
  }
  else
    req.scene = static_cast<const moveit_msgs::PlanningScene&>(*pswm);

  req.scene.name = opt_.scene;
  std::vector<std::string> planning_queries_names;
  try
  {
    pss_.getPlanningQueriesNames(opt_.query_regex, planning_queries_names, opt_.scene);
  }
  catch (std::runtime_error &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  
  if (planning_queries_names.empty())
    ROS_INFO("Scene '%s' has no associated queries", opt_.scene.c_str());
  req.plugins = opt_.plugins;
  
  unsigned int n_call = 0;
  
  // see if we have any start states specified; if we do, we run the benchmark for 
  // each start state; if not, we run it for one start state only: the current one saved in the loaded scene
  std::vector<std::string> start_states;
  if (!opt_.start_regex.empty())
  {     
    boost::regex start_regex(opt_.start_regex);
    std::vector<std::string> state_names;
    rs_.getKnownRobotStates(state_names);
    for (std::size_t i = 0 ; i < state_names.size() ; ++i)
    {
      boost::cmatch match;
      if (boost::regex_match(state_names[i].c_str(), match, start_regex))
        start_states.push_back(state_names[i]);
    }
    if (start_states.empty())
    {
      ROS_WARN("No stored states matched the provided regex: '%s'", opt_.start_regex.c_str());
      return;
    }
    else
      ROS_INFO("Running benchmark using %u start states.", (unsigned int)start_states.size());
  }
  else
    ROS_INFO("No specified start state. Running benchmark once with the default start state.");
  
  bool have_more_start_states = true;
  boost::scoped_ptr<moveit_msgs::RobotState> start_state_to_use;
  while (have_more_start_states)
  { 
    start_state_to_use.reset();
    
    // if no set of start states provided, run once for the current one
    if (opt_.start_regex.empty())
      have_more_start_states = false;
    else
    {
      // otherwise, extract the start states one by one
      std::string state_name = start_states.back();
      start_states.pop_back();
      have_more_start_states = !start_states.empty();
      moveit_warehouse::RobotStateWithMetadata robot_state;
      bool got_robot_state = false;
      try
      {
        got_robot_state = rs_.getRobotState(robot_state, state_name);
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      
      if (got_robot_state)
      {
        start_state_to_use.reset(new moveit_msgs::RobotState(*robot_state));
        ROS_INFO("Loaded start state '%s'", state_name.c_str());
      }
      else
        continue;
    }
    
    // run benchmarks for specified queries
    // ************************************

    if (!opt_.query_regex.empty())
    {
      for (std::size_t i = 0 ; i < planning_queries_names.size() ; ++i)
      {
        moveit_warehouse::MotionPlanRequestWithMetadata planning_query;
        pss_.getPlanningQuery(planning_query, opt_.scene, planning_queries_names[i]);

        // read request from db
        req.motion_plan_request = static_cast<const moveit_msgs::MotionPlanRequest&>(*planning_query);

        // update request given .cfg options
        if (start_state_to_use)
          req.motion_plan_request.start_state = *start_state_to_use;
        req.filename = opt_.output + "." + boost::lexical_cast<std::string>(++n_call) + ".log";
        if (!opt_.group_override.empty())
          req.motion_plan_request.group_name = opt_.group_override;

        if (opt_.timeout > 0.0)
          req.motion_plan_request.allowed_planning_time = opt_.timeout;

        if (!opt_.default_constrained_link.empty())
        {
          checkConstrainedLink(req.motion_plan_request.path_constraints, opt_.default_constrained_link);
          for (std::size_t j = 0 ; j < req.motion_plan_request.goal_constraints.size() ; ++j)
            checkConstrainedLink(req.motion_plan_request.goal_constraints[j], opt_.default_constrained_link);
        }
        if (!opt_.planning_frame.empty())
        {
          checkHeader(req.motion_plan_request.path_constraints, opt_.planning_frame);
          for (std::size_t j = 0 ; j < req.motion_plan_request.goal_constraints.size() ; ++j)
            checkHeader(req.motion_plan_request.goal_constraints[j], opt_.planning_frame);
        }

        ROS_INFO("Benckmarking query '%s' (%d of %d)", planning_queries_names[i].c_str(), (int)i+1, (int)planning_queries_names.size());
        runBenchmark(req);
      }
    }
    
    // if we have any goals specified as constraints, run benchmarks for those as well   
    // *******************************************************************************

    if (!opt_.goal_regex.empty())
    {
      std::vector<std::string> cnames;
      cs_.getKnownConstraints(opt_.goal_regex, cnames);
      for (std::size_t i = 0 ; i < cnames.size() ; ++i)
      {
        moveit_warehouse::ConstraintsWithMetadata constr;
        bool got_constraints = false;
        try
        {
          got_constraints = cs_.getConstraints(constr, cnames[i]);
        }
        catch (std::runtime_error &ex)
        {
          ROS_ERROR("%s", ex.what());
        }

        if (got_constraints)
        {
          // construct a planning request from the constraints message
          req.motion_plan_request = moveit_msgs::MotionPlanRequest();
          req.motion_plan_request.goal_constraints.resize(1);
          if (start_state_to_use)
            req.motion_plan_request.start_state = *start_state_to_use;
          req.motion_plan_request.goal_constraints[0] = *constr;
          
          // Apply the goal offset for constraints that appear to specify IK poses
          if (constr->position_constraints.size() == 1 && constr->orientation_constraints.size() == 1 && kinematic_constraints::countIndividualConstraints(*constr) == 2 &&
              constr->position_constraints[0].constraint_region.primitive_poses.size() == 1 && constr->position_constraints[0].constraint_region.mesh_poses.empty())
          {
            geometry_msgs::Pose wMc_msg;
            wMc_msg.position = constr->position_constraints[0].constraint_region.primitive_poses[0].position;
            wMc_msg.orientation = constr->orientation_constraints[0].orientation;
            Eigen::Affine3d wMc;
            tf::poseMsgToEigen(wMc_msg, wMc);
            
            Eigen::Affine3d offset_tf(Eigen::AngleAxis<double>(opt_.offsets[3], Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxis<double>(opt_.offsets[4], Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxis<double>(opt_.offsets[5], Eigen::Vector3d::UnitZ()));
            offset_tf.translation() = Eigen::Vector3d(opt_.offsets[0], opt_.offsets[1], opt_.offsets[2]);
            
            Eigen::Affine3d wMnc = wMc * offset_tf;
            geometry_msgs::Pose wMnc_msg;
            tf::poseEigenToMsg(wMnc, wMnc_msg);
            
            req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position = wMnc_msg.position;
            req.motion_plan_request.goal_constraints[0].orientation_constraints[0].orientation = wMnc_msg.orientation;
          }
          
          if (!opt_.group_override.empty())
            req.motion_plan_request.group_name = opt_.group_override;
          if (opt_.timeout > 0.0)
            req.motion_plan_request.allowed_planning_time = opt_.timeout;
          if (!opt_.default_constrained_link.empty())
            checkConstrainedLink(req.motion_plan_request.goal_constraints[0], opt_.default_constrained_link);
          if (!opt_.planning_frame.empty())
            checkHeader(req.motion_plan_request.goal_constraints[0], opt_.planning_frame);
          req.filename = opt_.output + "." + boost::lexical_cast<std::string>(++n_call) + ".log";

          ROS_INFO("Benckmarking goal '%s' (%d of %d)", cnames[i].c_str(), (int)i+1, (int)cnames.size());
          runBenchmark(req);
        }
      }
    }
    
    // if we have any goals specified as trajectory constraints, run benchmarks for those as well   
    // ******************************************************************************************

    if (!opt_.trajectory_regex.empty())
    {
      std::vector<std::string> cnames;
      tcs_.getKnownTrajectoryConstraints(opt_.trajectory_regex, cnames);
      for (std::size_t i = 0 ; i < cnames.size() ; ++i)
      {
        moveit_warehouse::TrajectoryConstraintsWithMetadata constr;
        bool got_constraints = false;
        try
        {
          got_constraints = tcs_.getTrajectoryConstraints(constr, cnames[i]);
        }
        catch (std::runtime_error &ex)
        {
          ROS_ERROR("%s", ex.what());
        }

        if (got_constraints)
        {
          // construct a planning request from the trajectory constraints message
          req.motion_plan_request = moveit_msgs::MotionPlanRequest();
          if (start_state_to_use)
            req.motion_plan_request.start_state = *start_state_to_use;
          req.motion_plan_request.trajectory_constraints = *constr;
          
          Eigen::Affine3d offset_tf(Eigen::AngleAxis<double>(opt_.offsets[3], Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxis<double>(opt_.offsets[4], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxis<double>(opt_.offsets[5], Eigen::Vector3d::UnitZ()));
          offset_tf.translation() = Eigen::Vector3d(opt_.offsets[0], opt_.offsets[1], opt_.offsets[2]);
          
          // Apply waypoint offsets, check fields
          for (std::size_t tc = 0; tc < constr->constraints.size(); ++tc)
          { 
            // Apply the goal offset for constraints that appear to specify IK poses
            if (constr->constraints[tc].position_constraints.size() == 1 && constr->constraints[tc].orientation_constraints.size() == 1 && kinematic_constraints::countIndividualConstraints(constr->constraints[tc]) == 2 &&
                constr->constraints[tc].position_constraints[0].constraint_region.primitive_poses.size() == 1 && constr->constraints[tc].position_constraints[0].constraint_region.mesh_poses.empty())
            {
              geometry_msgs::Pose wMc_msg;
              wMc_msg.position = req.motion_plan_request.trajectory_constraints.constraints[tc].position_constraints[0].constraint_region.primitive_poses[0].position;
              wMc_msg.orientation = req.motion_plan_request.trajectory_constraints.constraints[tc].orientation_constraints[0].orientation;
              Eigen::Affine3d wMc;
              tf::poseMsgToEigen(wMc_msg, wMc);
              
              Eigen::Affine3d wMnc = wMc * offset_tf;   
              geometry_msgs::Pose wMnc_msg;
              tf::poseEigenToMsg(wMnc, wMnc_msg);
              
              req.motion_plan_request.trajectory_constraints.constraints[tc].position_constraints[0].constraint_region.primitive_poses[0].position = wMnc_msg.position;
              req.motion_plan_request.trajectory_constraints.constraints[tc].orientation_constraints[0].orientation = wMnc_msg.orientation;
            }
            if (!opt_.default_constrained_link.empty())
              checkConstrainedLink(req.motion_plan_request.trajectory_constraints.constraints[tc], opt_.default_constrained_link);
            if (!opt_.planning_frame.empty())
              checkHeader(req.motion_plan_request.trajectory_constraints.constraints[tc], opt_.planning_frame);
          }

          if (!opt_.group_override.empty())
            req.motion_plan_request.group_name = opt_.group_override;
          if (opt_.timeout > 0.0)
            req.motion_plan_request.allowed_planning_time = opt_.timeout;
          req.filename = opt_.output + ".trajectory." + boost::lexical_cast<std::string>(i+1) + ".log";

          ROS_INFO("Benckmarking trajectory '%s' (%d of %d)", cnames[i].c_str(), (int)i+1, (int)cnames.size());
          runBenchmark(req);
        }
      }
    }
  }
}

bool moveit_benchmarks::BenchmarkExecution::readOptions(const std::string &filename)
{
  ROS_INFO("Loading '%s'...", filename.c_str());
  
  std::ifstream cfg(filename.c_str());
  if (!cfg.good())
  {
    ROS_ERROR_STREAM("Unable to open file '" << filename << "'");
    return false;
  }

  try
  {
    boost::program_options::options_description desc;
    desc.add_options()
      ("scene.name", boost::program_options::value<std::string>(), "Scene name")
      ("scene.runs", boost::program_options::value<std::string>()->default_value("1"), "Number of runs")
      ("scene.timeout", boost::program_options::value<std::string>()->default_value(""), "Timeout for planning (s)")
      ("scene.start", boost::program_options::value<std::string>()->default_value(""), "Regex for the start states to use")
      ("scene.query", boost::program_options::value<std::string>()->default_value(".*"), "Regex for the queries to execute")
      ("scene.goal", boost::program_options::value<std::string>()->default_value(""), "Regex for the names of constraints to use as goals")
      ("scene.trajectory", boost::program_options::value<std::string>()->default_value(""), "Regex for the names of constraints to use as trajectories")
      ("scene.group", boost::program_options::value<std::string>()->default_value(""), "Override the group to plan for")
      ("scene.planning_frame", boost::program_options::value<std::string>()->default_value(""), "Override the planning frame to use")
      ("scene.default_constrained_link", boost::program_options::value<std::string>()->default_value(""),
       "Specify the default link to consider as constrained when one is not specified in a moveit_msgs::Constraints message")
      ("scene.goal_offset_x", boost::program_options::value<std::string>()->default_value("0.0"), "Goal offset in x")
      ("scene.goal_offset_y", boost::program_options::value<std::string>()->default_value("0.0"), "Goal offset in y")
      ("scene.goal_offset_z", boost::program_options::value<std::string>()->default_value("0.0"), "Goal offset in z")
      ("scene.goal_offset_roll", boost::program_options::value<std::string>()->default_value("0.0"), "Goal offset in roll")
      ("scene.goal_offset_pitch", boost::program_options::value<std::string>()->default_value("0.0"), "Goal offset in pitch")
      ("scene.goal_offset_yaw", boost::program_options::value<std::string>()->default_value("0.0"), "Goal offset in yaw")
      ("scene.output", boost::program_options::value<std::string>(), "Location of benchmark log file");
    
    boost::program_options::variables_map vm;
    boost::program_options::parsed_options po = boost::program_options::parse_config_file(cfg, desc, true);
    
    cfg.close();
    boost::program_options::store(po, vm);
    
    std::map<std::string, std::string> declared_options;
    for (boost::program_options::variables_map::iterator it = vm.begin() ; it != vm.end() ; ++it)
      declared_options[it->first] = boost::any_cast<std::string>(vm[it->first].value());
    opt_.scene = declared_options["scene.name"];
    opt_.output = declared_options["scene.output"];
    opt_.start_regex = declared_options["scene.start"];
    opt_.query_regex = declared_options["scene.query"];
    opt_.goal_regex = declared_options["scene.goal"];
    opt_.trajectory_regex = declared_options["scene.trajectory"];
    opt_.group_override = declared_options["scene.group"];
    opt_.default_constrained_link = declared_options["scene.default_constrained_link"];
    opt_.planning_frame = declared_options["scene.planning_frame"];
    try
    {
      memset(opt_.offsets, 0, 6*sizeof(double));
      if (!declared_options["scene.goal_offset_x"].empty())
        opt_.offsets[0] = boost::lexical_cast<double>(declared_options["scene.goal_offset_x"]);
      if (!declared_options["scene.goal_offset_y"].empty())
        opt_.offsets[1] = boost::lexical_cast<double>(declared_options["scene.goal_offset_y"]);
      if (!declared_options["scene.goal_offset_z"].empty())
        opt_.offsets[2] = boost::lexical_cast<double>(declared_options["scene.goal_offset_z"]);
      if (!declared_options["scene.goal_offset_roll"].empty())
        opt_.offsets[3] = boost::lexical_cast<double>(declared_options["scene.goal_offset_roll"]);
      if (!declared_options["scene.goal_offset_pitch"].empty())
        opt_.offsets[4] = boost::lexical_cast<double>(declared_options["scene.goal_offset_pitch"]);
      if (!declared_options["scene.goal_offset_yaw"].empty())
        opt_.offsets[5] = boost::lexical_cast<double>(declared_options["scene.goal_offset_yaw"]);
    }
    catch(boost::bad_lexical_cast &ex)
    {
      ROS_WARN("%s", ex.what());
    }

    if (opt_.output.empty())
      opt_.output = filename;
    opt_.plugins.clear();
    std::size_t default_run_count = 1;
    if (!declared_options["scene.runs"].empty())
    {
      try
      {
        default_run_count = boost::lexical_cast<std::size_t>(declared_options["scene.runs"]);
      }
      catch(boost::bad_lexical_cast &ex)
      {
        ROS_WARN("%s", ex.what());
      }
    }
    opt_.default_run_count = default_run_count;
    opt_.timeout = 0.0;
    if (!declared_options["scene.timeout"].empty())
    {
      try
      {
        opt_.timeout = boost::lexical_cast<double>(declared_options["scene.timeout"]);
      }
      catch(boost::bad_lexical_cast &ex)
      {
        ROS_WARN("%s", ex.what());
      }
    }
    
    std::vector<std::string> unr = boost::program_options::collect_unrecognized(po.options, boost::program_options::exclude_positional);
    boost::scoped_ptr<PlanningPluginOptions> bpo;
    for (std::size_t i = 0 ; i < unr.size() / 2 ; ++i)
    {
      std::string key = boost::to_lower_copy(unr[i * 2]);
      std::string val = unr[i * 2 + 1];
      if (key.substr(0, 7) != "plugin.")
      {
        ROS_WARN("Unknown option: '%s' = '%s'", key.c_str(), val.c_str());
        continue;
      }
      std::string k = key.substr(7);
      if (k == "name")
      {
        if (bpo)
          opt_.plugins.push_back(*bpo);
        bpo.reset(new PlanningPluginOptions());
        bpo->name = val;
        bpo->runs = default_run_count;
      }
      else
        if (k == "runs")
        {
          if (bpo)
          {
            try
            {
              bpo->runs = boost::lexical_cast<std::size_t>(val);
            }
            catch(boost::bad_lexical_cast &ex)
            {   
              ROS_WARN("%s", ex.what());
            }
          }
          else
            ROS_WARN("Ignoring option '%s' = '%s'. Please include plugin name first.", key.c_str(), val.c_str());
        }
        else
          if (k == "planners")
          {
            if (bpo)
            {   
              boost::char_separator<char> sep(" ");
              boost::tokenizer<boost::char_separator<char> > tok(val, sep);
              for (boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin() ; beg != tok.end(); ++beg)
                bpo->planners.push_back(*beg);
            }
            else
              ROS_WARN("Ignoring option '%s' = '%s'. Please include plugin name first.", key.c_str(), val.c_str());
          }
    }
    if (bpo)
      opt_.plugins.push_back(*bpo);
  }
  
  catch(...)
  {
    ROS_ERROR_STREAM("Unable to parse '" << filename << "'");
    return false;
  }
  
  return true;
}

void moveit_benchmarks::BenchmarkExecution::printOptions(std::ostream &out)
{
  out << "Benchmark for scene '" << opt_.scene << "' to be saved at location '" << opt_.output << "'" << std::endl;
  if (!opt_.query_regex.empty())
    out << "Planning requests associated to the scene that match '" << opt_.query_regex << "' will be evaluated" << std::endl;
  if (!opt_.goal_regex.empty())
    out << "Planning requests constructed from goal constraints that match '" << opt_.goal_regex << "' will be evaluated" << std::endl;
  if (!opt_.trajectory_regex.empty())
      out << "Planning requests constructed from trajectory constraints that match '" << opt_.trajectory_regex << "' will be evaluated" << std::endl;
  out << "Plugins:" << std::endl;
  for (std::size_t i = 0 ; i < opt_.plugins.size() ; ++i)
  {
    out << "   * name: " << opt_.plugins[i].name << " (to be run " << opt_.plugins[i].runs << " times for each planner)" << std::endl;
    out << "   * planners:";
    for (std::size_t j = 0 ; j < opt_.plugins[i].planners.size() ; ++j)
      out << ' ' << opt_.plugins[i].planners[j];
    out << std::endl;
  }
}

void moveit_benchmarks::BenchmarkExecution::runBenchmark(BenchmarkRequest &req)
{
  if (req.benchmark_type & BENCHMARK_PLANNERS)
    runPlanningBenchmark(req);
  if (req.benchmark_type & BENCHMARK_GOAL_EXISTANCE)
    runGoalExistenceBenchmark(req);
}

void moveit_benchmarks::BenchmarkExecution::collectMetrics(std::map<std::string, std::string> &rundata,
                                                          const planning_interface::MotionPlanDetailedResponse &mp_res,
                                                          bool solved, double total_time)
{
  rundata["total_time REAL"] = boost::lexical_cast<std::string>(total_time);
  rundata["solved BOOLEAN"] = boost::lexical_cast<std::string>(solved);
  double L = 0.0;
  double clearance = 0.0;
  double smoothness = 0.0;		
  bool correct = true;
  if (solved)
  {
    double process_time = total_time;
    for (std::size_t j = 0 ; j < mp_res.trajectory_.size() ; ++j)
    {
      correct = true;
      L = 0.0;
      clearance = 0.0;
      smoothness = 0.0;
      const robot_trajectory::RobotTrajectory &p = *mp_res.trajectory_[j];
      
      // compute path length
      for (std::size_t k = 1 ; k < p.getWayPointCount() ; ++k)
        L += p.getWayPoint(k-1).distance(p.getWayPoint(k));
      
      // compute correctness and clearance
      collision_detection::CollisionRequest req;
      for (std::size_t k = 0 ; k < p.getWayPointCount() ; ++k)
      {
        collision_detection::CollisionResult res;
        planning_scene_->checkCollisionUnpadded(req, res, p.getWayPoint(k));
        if (res.collision)
          correct = false;
        if (!p.getWayPoint(k).satisfiesBounds())
          correct = false;
        double d = planning_scene_->distanceToCollisionUnpadded(p.getWayPoint(k));
        if (d > 0.0) // in case of collision, distance is negative
          clearance += d;
      }
      clearance /= (double)p.getWayPointCount();
      
      // compute smoothness
      if (p.getWayPointCount() > 2)
      {
        double a = p.getWayPoint(0).distance(p.getWayPoint(1));
        for (std::size_t k = 2 ; k < p.getWayPointCount() ; ++k)
        {
          // view the path as a sequence of segments, and look at the triangles it forms:
          //          s1
          //          /\          s4
          //      a  /  \ b       |
          //        /    \        |
          //       /......\_______|
          //     s0    c   s2     s3
          //
          // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
          double b = p.getWayPoint(k-1).distance(p.getWayPoint(k));
          double cdist = p.getWayPoint(k-2).distance(p.getWayPoint(k));
          double acosValue = (a*a + b*b - cdist*cdist) / (2.0*a*b);
          if (acosValue > -1.0 && acosValue < 1.0)
          {
            // the smoothness is actually the outside angle of the one we compute
            double angle = (boost::math::constants::pi<double>() - acos(acosValue));
            
            // and we normalize by the length of the segments
            double u = 2.0 * angle; /// (a + b);
            smoothness += u * u;
          }
          a = b;
        }
        smoothness /= (double)p.getWayPointCount();
      }
      rundata["path_" + mp_res.description_[j] + "_correct BOOLEAN"] = boost::lexical_cast<std::string>(correct);
      rundata["path_" + mp_res.description_[j] + "_length REAL"] = boost::lexical_cast<std::string>(L);
      rundata["path_" + mp_res.description_[j] + "_clearance REAL"] = boost::lexical_cast<std::string>(clearance);
      rundata["path_" + mp_res.description_[j] + "_smoothness REAL"] = boost::lexical_cast<std::string>(smoothness);
      rundata["path_" + mp_res.description_[j] + "_time REAL"] = boost::lexical_cast<std::string>(mp_res.processing_time_[j]);     
      process_time -= mp_res.processing_time_[j];
    }
    if (process_time <= 0.0)
      process_time = 0.0;
    rundata["process_time REAL"] = boost::lexical_cast<std::string>(process_time);
    
  }
}

namespace
{
bool isIKSolutionCollisionFree(const planning_scene::PlanningScene *scene, 
                               robot_state::JointStateGroup *group,
                               const std::vector<double> &ik_solution,
                               bool *reachable) 
{
  group->setVariableValues(ik_solution);
  *reachable = true;
  if (scene->isStateColliding(*group->getRobotState(), group->getName(), false))
    return false;
  else
    return true;
}
}

void moveit_benchmarks::BenchmarkExecution::runPlanningBenchmark(BenchmarkRequest &req)
{
  // figure out which planners to test
  if (!req.plugins.empty())
    for (std::size_t i = 0 ; i < req.plugins.size() ; ++i)
      if (planner_interfaces_.find(req.plugins[i].name) == planner_interfaces_.end())
        ROS_ERROR("Planning interface '%s' was not found", req.plugins[i].name.c_str());
  
  std::vector<planning_interface::Planner*> planner_interfaces_to_benchmark;
  std::vector<std::vector<std::string> > planner_ids_to_benchmark_per_planner_interface;
  std::vector<std::size_t> average_count_per_planner_interface;
  planning_interface::MotionPlanRequest mp_req = req.motion_plan_request;
  
  for (std::map<std::string, boost::shared_ptr<planning_interface::Planner> >::const_iterator it = planner_interfaces_.begin() ; 
       it != planner_interfaces_.end(); ++it)
  {
    int found = -1;
    if (!req.plugins.empty())
    {
      for (std::size_t i = 0 ; i < req.plugins.size() ; ++i)
      {
        if (req.plugins[i].name == it->first)
        {
          found = i;
          break;
        }
      }
      if (found < 0)
        continue;
    }
    if (it->second->canServiceRequest(mp_req))
    {
      planner_interfaces_to_benchmark.push_back(it->second.get());
      planner_ids_to_benchmark_per_planner_interface.resize(planner_ids_to_benchmark_per_planner_interface.size() + 1);
      average_count_per_planner_interface.resize(average_count_per_planner_interface.size() + 1, 1);
      std::vector<std::string> known;
      it->second->getPlanningAlgorithms(known);
      
      if (found < 0 || req.plugins[found].planners.empty())
        planner_ids_to_benchmark_per_planner_interface.back() = known;
      else
      {
        average_count_per_planner_interface.back() = std::max<std::size_t>(1, req.plugins[found].runs);
        for (std::size_t k = 0 ; k < req.plugins[found].planners.size() ; ++k)
        {
          bool fnd = false;
          for (std::size_t q = 0 ; q < known.size() ; ++q)
            if (known[q] == req.plugins[found].planners[k] ||
                mp_req.group_name + "[" + req.plugins[found].planners[k] + "]" == known[q])
            {
              fnd = true;
              break;
            }
          if (fnd)
            planner_ids_to_benchmark_per_planner_interface.back().push_back(req.plugins[found].planners[k]);
          else
          {
            ROS_ERROR("The planner id '%s' is not known to the planning interface '%s'", req.plugins[found].planners[k].c_str(), it->first.c_str());
            // To help user debug, list all available planners:
            ROS_ERROR_STREAM("Known algorithms in " << it->first.c_str() << ":");
            for (std::size_t i = 0; i < known.size(); ++i)
            {
              ROS_ERROR_STREAM(" - " << known[i]);
            }
          }
        }          
      }
      
      if (planner_ids_to_benchmark_per_planner_interface.back().empty())
        ROS_ERROR("Planning interface '%s' has no planners defined", it->first.c_str());
    }
    else
      ROS_WARN_STREAM("Planning interface '" << it->second->getDescription() << "' is not able to solve the specified benchmark problem.");
  }
  
  if (planner_interfaces_to_benchmark.empty())
  {
    ROS_ERROR("There are no planning interfaces to benchmark");
    return;
  }
  
  // output information about tested planners
  ROS_INFO("Benchmarking planning interfaces:");
  std::stringstream sst;
  for (std::size_t i = 0 ; i < planner_interfaces_to_benchmark.size() ; ++i)
  {
    if (planner_ids_to_benchmark_per_planner_interface[i].empty())
      continue;
    sst << "  * " << planner_interfaces_to_benchmark[i]->getDescription() << " executed " << average_count_per_planner_interface[i] << " times" << std::endl;
    for (std::size_t k = 0 ; k < planner_ids_to_benchmark_per_planner_interface[i].size() ; ++k)
      sst << "    - " << planner_ids_to_benchmark_per_planner_interface[i][k] << std::endl;
    sst << std::endl;
  }
  ROS_INFO("\n%s", sst.str().c_str());
  
  // configure planning context
  
  if (req.scene.robot_model_name != planning_scene_->getRobotModel()->getName())
  {
    // if we have a different robot, use the world geometry only
    
    // clear all geometry from the scene
    planning_scene_->getWorldNonConst()->clearObjects();
    planning_scene_->getCurrentStateNonConst().clearAttachedBodies();
    planning_scene_->getCurrentStateNonConst().setToDefaultValues();
    
    planning_scene_->processPlanningSceneWorldMsg(req.scene.world);
  }
  else
    planning_scene_->usePlanningSceneMsg(req.scene);
  
  std::size_t total_n_planners = 0;
  std::size_t total_n_runs = 0;
  for (std::size_t i = 0 ; i < planner_ids_to_benchmark_per_planner_interface.size() ; ++i)
  {
    total_n_planners += planner_ids_to_benchmark_per_planner_interface[i].size();
    total_n_runs += planner_ids_to_benchmark_per_planner_interface[i].size() * average_count_per_planner_interface[i];
  }
  
  // benchmark all the planners
  ros::WallTime startTime = ros::WallTime::now();
  boost::progress_display progress(total_n_runs, std::cout);
  typedef std::vector<std::map<std::string, std::string> > RunData;
  std::vector<RunData> data;
  std::vector<bool> first(planner_interfaces_to_benchmark.size(), true);
  for (std::size_t i = 0 ; i < planner_interfaces_to_benchmark.size() ; ++i)
  {
    for (std::size_t j = 0 ; j < planner_ids_to_benchmark_per_planner_interface[i].size() ; ++j)
    {
      mp_req.planner_id = planner_ids_to_benchmark_per_planner_interface[i][j];
      RunData runs(average_count_per_planner_interface[i]);
      for (unsigned int c = 0 ; c < average_count_per_planner_interface[i] ; ++c)
      {
        ++progress; 
        ROS_DEBUG("Calling %s:%s", planner_interfaces_to_benchmark[i]->getDescription().c_str(), mp_req.planner_id.c_str());
        planning_interface::MotionPlanDetailedResponse mp_res;
        ros::WallTime start = ros::WallTime::now();
        bool solved = planner_interfaces_to_benchmark[i]->solve(planning_scene_, mp_req, mp_res);
        double total_time = (ros::WallTime::now() - start).toSec();
        
        // collect data   
        start = ros::WallTime::now();
        collectMetrics(runs[c], mp_res, solved, total_time);
        double metrics_time = (ros::WallTime::now() - start).toSec();
        ROS_DEBUG("Spent %lf seconds collecting metrics", metrics_time);
        
        // record the first solution in the response
        if (solved && first[i])
        {
          first[i] = false;
        }
      }
      data.push_back(runs);
    }
  }
  
  double duration = (ros::WallTime::now() - startTime).toSec();
  std::string host = moveit_benchmarks::getHostname();
  std::string filename = req.filename.empty() ? ("moveit_benchmarks_" + host + "_" + boost::posix_time::to_iso_extended_string(startTime.toBoost()) + ".log") : req.filename;
  std::ofstream out(filename.c_str());
  out << "Experiment " << (planning_scene_->getName().empty() ? "NO_NAME" : planning_scene_->getName()) << std::endl;
  out << "Running on " << (host.empty() ? "UNKNOWN" : host) << std::endl;
  out << "Starting at " << boost::posix_time::to_iso_extended_string(startTime.toBoost()) << std::endl;
  out << "<<<|" << std::endl << "ROS" << std::endl << req.motion_plan_request << std::endl << "|>>>" << std::endl;
  out << req.motion_plan_request.allowed_planning_time << " seconds per run" << std::endl;
  out << duration << " seconds spent to collect the data" << std::endl;
  out << total_n_planners << " planners" << std::endl;
  std::size_t ri = 0;
  for (std::size_t q = 0 ; q < planner_interfaces_to_benchmark.size() ; ++q)
    for (std::size_t p = 0 ; p < planner_ids_to_benchmark_per_planner_interface[q].size() ; ++p, ++ri)
    {
      out << planner_interfaces_to_benchmark[q]->getDescription() + "_" + planner_ids_to_benchmark_per_planner_interface[q][p] << std::endl;
      // in general, we could have properties specific for a planner;
      // right now, we do not include such properties
      out << "0 common properties" << std::endl;
      
      // construct the list of all possible properties for all runs
      std::set<std::string> propSeen;
      for (std::size_t j = 0 ; j < data[ri].size() ; ++j)
        for (std::map<std::string, std::string>::const_iterator mit = data[ri][j].begin() ; mit != data[ri][j].end() ; ++mit)
          propSeen.insert(mit->first);
      std::vector<std::string> properties;
      for (std::set<std::string>::iterator it = propSeen.begin() ; it != propSeen.end() ; ++it)
        properties.push_back(*it);
      out << properties.size() << " properties for each run" << std::endl;
      for (unsigned int j = 0 ; j < properties.size() ; ++j)
        out << properties[j] << std::endl;
      out << data[ri].size() << " runs" << std::endl;
      for (std::size_t j = 0 ; j < data[ri].size() ; ++j)
      {
        for (unsigned int k = 0 ; k < properties.size() ; ++k)
        {
          std::map<std::string, std::string>::const_iterator it = data[ri][j].find(properties[k]);
          if (it != data[ri][j].end())
            out << it->second;
          out << "; ";
        }
        out << std::endl;
      }
      out << '.' << std::endl;
    }
  out.close();
  ROS_INFO("Results saved to '%s'", filename.c_str());
}

void moveit_benchmarks::BenchmarkExecution::runGoalExistenceBenchmark(BenchmarkRequest &req)
{
  // configure planning context
  if (req.scene.robot_model_name != planning_scene_->getRobotModel()->getName())
  {
    // if we have a different robot, use the world geometry only
    // clear all geometry from the scene
    planning_scene_->getWorldNonConst()->clearObjects();
    planning_scene_->getCurrentStateNonConst().clearAttachedBodies();
    planning_scene_->getCurrentStateNonConst().setToDefaultValues();
    
    planning_scene_->processPlanningSceneWorldMsg(req.scene.world);
    planning_scene_->setName(req.scene.name);
  }
  else
    planning_scene_->usePlanningSceneMsg(req.scene);
  
  // \todo the code below needs to be replaced with using constraint samplers;
  
  if (req.motion_plan_request.goal_constraints.size() == 0 &&
      req.motion_plan_request.goal_constraints[0].position_constraints.size() == 0 &&
      req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.size() == 0 &&
      req.motion_plan_request.goal_constraints[0].orientation_constraints.size() == 0 &&
      req.motion_plan_request.trajectory_constraints.constraints.size() == 0)
  {
    ROS_ERROR("Invalid goal constraints");
    return;
  }
  
  bool success = false;
  bool reachable = false;
  if (req.motion_plan_request.goal_constraints.size() > 0 &&
      req.motion_plan_request.goal_constraints[0].position_constraints.size() > 0 &&
      req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.size() > 0 &&
      req.motion_plan_request.goal_constraints[0].orientation_constraints.size() > 0)
  {
    //Compute IK on goal constraints
    geometry_msgs::Pose ik_pose;
    ik_pose.position.x = req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.x;
    ik_pose.position.y = req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.y;
    ik_pose.position.z = req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.z;
    ik_pose.orientation.x = req.motion_plan_request.goal_constraints[0].orientation_constraints[0].orientation.x;
    ik_pose.orientation.y = req.motion_plan_request.goal_constraints[0].orientation_constraints[0].orientation.y;
    ik_pose.orientation.z = req.motion_plan_request.goal_constraints[0].orientation_constraints[0].orientation.z;
    ik_pose.orientation.w = req.motion_plan_request.goal_constraints[0].orientation_constraints[0].orientation.w;
    
    robot_state::RobotState robot_state(planning_scene_->getCurrentState());
    robot_state::robotStateMsgToRobotState(req.motion_plan_request.start_state, robot_state);
    
    // Compute IK
    ROS_INFO_STREAM("Processing goal " << req.motion_plan_request.goal_constraints[0].name << " ...");
    ros::WallTime startTime = ros::WallTime::now();
    success = robot_state.getJointStateGroup(req.motion_plan_request.group_name)->
      setFromIK(ik_pose, req.motion_plan_request.num_planning_attempts, req.motion_plan_request.allowed_planning_time,
                boost::bind(&isIKSolutionCollisionFree, planning_scene_.get(),  _1, _2, &reachable));
    if (success)
    {
      ROS_INFO("  Success!");
    }
    else if (reachable)
    {
      ROS_INFO("  Reachable, but in collision");
    }
    else
    {
      ROS_INFO("  Not reachable");
    }
    // Log
    double duration = (ros::WallTime::now() - startTime).toSec();
    std::string host = moveit_benchmarks::getHostname();
    std::string filename = req.filename.empty() ? ("moveit_benchmarks_" + host + "_" + boost::posix_time::to_iso_extended_string(startTime.toBoost()) + ".log") : req.filename;
    std::ofstream out(filename.c_str());
    out << "Experiment " << (planning_scene_->getName().empty() ? "NO_NAME" : planning_scene_->getName()) << std::endl;
    out << "Running on " << (host.empty() ? "UNKNOWN" : host) << std::endl;
    out << "Starting at " << boost::posix_time::to_iso_extended_string(startTime.toBoost()) << std::endl;
    out << "<<<|" << std::endl << "ROS" << std::endl << req.motion_plan_request << std::endl << "|>>>" << std::endl;
    out << req.motion_plan_request.allowed_planning_time << " seconds per run" << std::endl;
    out << duration << " seconds spent to collect the data" << std::endl;
    out << "reachable BOOLEAN" << std::endl;
    out << "collision_free BOOLEAN" << std::endl;
    out << "total_time REAL" << std::endl;
    out << reachable << "; " << success << "; " << duration << std::endl;
    out.close();
    ROS_INFO("Results saved to '%s'", filename.c_str());
  }
  
  if (req.motion_plan_request.trajectory_constraints.constraints.size() > 0)
  {
    // Compute IK on trajectory constraints
    // Start Log
    ros::WallTime startTime = ros::WallTime::now();
    std::string host = moveit_benchmarks::getHostname();
    std::string filename = req.filename.empty() ? ("moveit_benchmarks_" + host + "_" + boost::posix_time::to_iso_extended_string(startTime.toBoost()) + ".log") : req.filename;
    std::ofstream out(filename.c_str());
    out << "Experiment " << (planning_scene_->getName().empty() ? "NO_NAME" : planning_scene_->getName()) << std::endl;
    out << "Running on " << (host.empty() ? "UNKNOWN" : host) << std::endl;
    out << "Starting at " << boost::posix_time::to_iso_extended_string(startTime.toBoost()) << std::endl;
    out << "<<<|" << std::endl << "ROS" << std::endl << req.motion_plan_request << std::endl << "|>>>" << std::endl;
    out << req.motion_plan_request.allowed_planning_time << " seconds per run" << std::endl;
    out << "reachable BOOLEAN" << std::endl;
    out << "collision_free BOOLEAN" << std::endl;
    out << "total_time REAL" << std::endl;
    
    for (std::size_t tc = 0; tc < req.motion_plan_request.trajectory_constraints.constraints.size(); ++tc)
    {
      geometry_msgs::Pose ik_pose;
      ik_pose.position.x = req.motion_plan_request.trajectory_constraints.constraints[tc].position_constraints[0].constraint_region.primitive_poses[0].position.x;
      ik_pose.position.y = req.motion_plan_request.trajectory_constraints.constraints[tc].position_constraints[0].constraint_region.primitive_poses[0].position.y;
      ik_pose.position.z = req.motion_plan_request.trajectory_constraints.constraints[tc].position_constraints[0].constraint_region.primitive_poses[0].position.z;
      ik_pose.orientation.x = req.motion_plan_request.trajectory_constraints.constraints[tc].orientation_constraints[0].orientation.x;
      ik_pose.orientation.y = req.motion_plan_request.trajectory_constraints.constraints[tc].orientation_constraints[0].orientation.y;
      ik_pose.orientation.z = req.motion_plan_request.trajectory_constraints.constraints[tc].orientation_constraints[0].orientation.z;
      ik_pose.orientation.w = req.motion_plan_request.trajectory_constraints.constraints[tc].orientation_constraints[0].orientation.w;
      
      robot_state::RobotState robot_state(planning_scene_->getCurrentState());
      robot_state::robotStateMsgToRobotState(req.motion_plan_request.start_state, robot_state);
      
      // Compute IK
      ROS_INFO_STREAM("Processing trajectory waypoint " << req.motion_plan_request.trajectory_constraints.constraints[tc].name << " ...");
      startTime = ros::WallTime::now();
      success = robot_state.getJointStateGroup(req.motion_plan_request.group_name)->
        setFromIK(ik_pose, req.motion_plan_request.num_planning_attempts, req.motion_plan_request.allowed_planning_time,
                  boost::bind(&isIKSolutionCollisionFree, planning_scene_.get(), _1, _2, &reachable));
      double duration = (ros::WallTime::now() - startTime).toSec();
      
      if (success)
      {
        ROS_INFO("  Success!");
      }
      else if (reachable)
      {
        ROS_INFO("  Reachable, but in collision");
      }
      else
      {
        ROS_INFO("  Not reachable");
      }
      
      out << reachable << "; " << success << "; " << duration << std::endl;
    }
    out.close();
    ROS_INFO("Results saved to '%s'", filename.c_str());
  }
}
