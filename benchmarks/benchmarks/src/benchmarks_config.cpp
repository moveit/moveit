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

#include <moveit/benchmarks/benchmarks_config.h>
#include <boost/regex.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <ros/ros.h>
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

moveit_benchmarks::BenchmarkConfig::BenchmarkConfig(const std::string &host, std::size_t port) :
  pss_(host, port), 
  psws_(host, port),
  cs_(host, port),
  rs_(host, port)
{  
}

const std::vector<moveit_msgs::ComputePlanningPluginsBenchmark::Request> moveit_benchmarks::BenchmarkConfig::getBenchmarkRequests(void)
{
  std::vector<moveit_msgs::ComputePlanningPluginsBenchmark::Request> req_list;

  moveit_warehouse::PlanningSceneWithMetadata pswm;
  moveit_warehouse::PlanningSceneWorldWithMetadata pswwm;
  bool world_only = false;
  
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
        return req_list;
      world_only = true;
    }
    else
    {  
      ROS_ERROR("Scene '%s' not found in warehouse", opt_.scene.c_str());
      return req_list;
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
      return req_list;
    }
  }

  moveit_msgs::ComputePlanningPluginsBenchmark::Request req;
  if (world_only)
  {
    req.scene.world = static_cast<const moveit_msgs::PlanningSceneWorld&>(*pswwm);
    req.scene.robot_model_name = "NO ROBOT INFORMATION. ONLY WORLD GEOMETRY"; // so that run_benchmark sees a different robot name
  }
  else
    req.scene = static_cast<const moveit_msgs::PlanningScene&>(*pswm);
  req.scene.name = opt_.scene;
  std::vector<moveit_warehouse::MotionPlanRequestWithMetadata> planning_queries;
  try
  {
    pss_.getPlanningQueries(planning_queries, opt_.scene);
  }
  catch (std::runtime_error &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  
  if (planning_queries.empty())
    ROS_INFO("Scene '%s' has no associated queries", opt_.scene.c_str());
  req.default_average_count = opt_.default_run_count;
  req.planner_interfaces.resize(opt_.plugins.size());
  req.average_count.resize(req.planner_interfaces.size());
  for (std::size_t i = 0 ; i < opt_.plugins.size() ; ++i)
  {
    req.planner_interfaces[i].name = opt_.plugins[i].name;
    req.planner_interfaces[i].planner_ids = opt_.plugins[i].planners;
    req.average_count[i] = opt_.plugins[i].runs;
  }

  unsigned int n_call = 0;
  
  // see if we have any start states specified
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
      return req_list;
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
    
    if (!opt_.query_regex.empty())
    {
      boost::regex query_regex(opt_.query_regex);
      for (std::size_t i = 0 ; i < planning_queries.size() ; ++i)
      {
        std::string query_name = planning_queries[i]->lookupString(moveit_warehouse::PlanningSceneStorage::MOTION_PLAN_REQUEST_ID_NAME);
        boost::cmatch match;
        if (boost::regex_match(query_name.c_str(), match, query_regex))
        {
          // read request from db
          req.motion_plan_request = static_cast<const moveit_msgs::MotionPlanRequest&>(*planning_queries[i]);
          
          // update request given .cfg options
          if (start_state_to_use)
            req.motion_plan_request.start_state = *start_state_to_use;
          req.filename = opt_.output + "." + boost::lexical_cast<std::string>(++n_call) + ".log";
          if (!opt_.group_override.empty())
            req.motion_plan_request.group_name = opt_.group_override;
          
          if (opt_.timeout > 0.0)
            req.motion_plan_request.allowed_planning_time = ros::Duration(opt_.timeout);
          
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

          req_list.push_back(req);
        }
      }
    }
    
    if (!opt_.goal_regex.empty())
    {
      boost::regex goal_regex(opt_.goal_regex);
      std::vector<std::string> cnames;
      cs_.getKnownConstraints(cnames);
      for (std::size_t i = 0 ; i < cnames.size() ; ++i)
      {
        boost::cmatch match;
        if (boost::regex_match(cnames[i].c_str(), match, goal_regex))
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
            if (!opt_.group_override.empty())
              req.motion_plan_request.group_name = opt_.group_override;
            if (opt_.timeout > 0.0)
              req.motion_plan_request.allowed_planning_time = ros::Duration(opt_.timeout);
            if (!opt_.default_constrained_link.empty())
              checkConstrainedLink(req.motion_plan_request.goal_constraints[0], opt_.default_constrained_link);
            if (!opt_.planning_frame.empty())
              checkHeader(req.motion_plan_request.goal_constraints[0], opt_.planning_frame);
            req.filename = opt_.output + "." + boost::lexical_cast<std::string>(++n_call) + ".log";
            
            req_list.push_back(req);
          }
        }
      }
    }
  }

  return req_list;
}

bool moveit_benchmarks::BenchmarkConfig::readOptions(const char *filename)
{
  ROS_INFO("Loading '%s'...", filename);
  
  std::ifstream cfg(filename);
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
      ("scene.group", boost::program_options::value<std::string>()->default_value(""), "Override the group to plan for")
      ("scene.planning_frame", boost::program_options::value<std::string>()->default_value(""), "Override the planning frame to use")
      ("scene.default_constrained_link", boost::program_options::value<std::string>()->default_value(""),
       "Specify the default link to consider as constrained when one is not specified in a moveit_msgs::Constraints message")
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
    opt_.group_override = declared_options["scene.group"];
    opt_.default_constrained_link = declared_options["scene.default_constrained_link"];
    opt_.planning_frame = declared_options["scene.planning_frame"];
    
    if (opt_.output.empty())
      opt_.output = std::string(filename);
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
    boost::scoped_ptr<BenchmarkOptions::PluginOptions> bpo;
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
        bpo.reset(new BenchmarkOptions::PluginOptions());
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

void moveit_benchmarks::BenchmarkConfig::printOptions(std::ostream &out)
{
  out << "Benchmark for scene '" << opt_.scene << "' to be saved at location '" << opt_.output << "'" << std::endl;
  if (!opt_.query_regex.empty())
    out << "Planning requests associated to the scene that match '" << opt_.query_regex << "' will be evaluated" << std::endl;
  if (!opt_.goal_regex.empty())
    out << "Planning requests constructed from goal constraints that match '" << opt_.goal_regex << "' will be evaluated" << std::endl;
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
