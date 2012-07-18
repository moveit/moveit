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

#include <ros/ros.h>
#include <planning_scene/planning_scene.h>
#include <robot_model_loader/robot_model_loader.h>
#include <pluginlib/class_loader.h>
#include <planning_interface/planning_interface.h>
#include <planning_models/conversions.h>

#include <moveit_warehouse/warehouse.h>
#include <moveit_msgs/ComputePlanningBenchmark.h>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/scoped_ptr.hpp>

#include <fstream>

static const std::string BENCHMARK_SERVICE_NAME="benchmark_planning_problem"; // name of the advertised benchmarking service (within the ~ namespace)

struct BenchmarkOptions
{
  std::string scene;
  std::string output;
  
  struct PluginOptions
  {
    std::string name;
    std::vector<std::string> planners;
    std::size_t runs;
  };
  
  std::vector<PluginOptions> plugins;
};

void printOptions(std::ostream &out, const BenchmarkOptions &opt)
{
  out << "Benchmark for scene '" << opt.scene << "' to be saved at location '" << opt.output << "'" << std::endl;
  out << "Plugins:" << std::endl;
  for (std::size_t i = 0 ; i < opt.plugins.size() ; ++i)
  {
    out << "   * name: " << opt.plugins[i].name << " (to be run " << opt.plugins[i].runs << " times for each planner)" << std::endl;
    out << "   * planners:";
    for (std::size_t j = 0 ; j < opt.plugins[i].planners.size() ; ++j)
      out << ' ' << opt.plugins[i].planners[j];
    out << std::endl;
  }
}

bool readOptions(const char *filename, BenchmarkOptions &opt)
{ 
  std::ifstream cfg(filename);
  if (!cfg.good())
  {
    std::cerr << "Unable to open file '" << filename << "'" << std::endl;
    return false;
  }
  
  boost::program_options::options_description desc;
  desc.add_options()
    ("scene.name", boost::program_options::value<std::string>(), "Scene name")
    ("scene.runs", boost::program_options::value<std::string>(), "Number of runs")
    ("scene.output", boost::program_options::value<std::string>(), "Location of benchmark log file");
  
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_config_file(cfg, desc, true);
  cfg.close();
  boost::program_options::store(po, vm);

  std::map<std::string, std::string> declared_options;
  for (boost::program_options::variables_map::iterator it = vm.begin() ; it != vm.end() ; ++it)
    declared_options[it->first] = boost::any_cast<std::string>(vm[it->first].value());
  opt.scene = declared_options["scene.name"];
  opt.output = declared_options["scene.output"];
  if (opt.output.empty())
    opt.output = std::string(filename) + ".log";
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
        opt.plugins.push_back(*bpo);
      if (!bpo)
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
    opt.plugins.push_back(*bpo);
  return true;
}

void runBenchmark(const moveit_warehouse::PlanningSceneStorage &pss, const BenchmarkOptions &opt)
{
  moveit_warehouse::PlanningSceneWithMetadata pswm;
  if (!pss.getPlanningScene(pswm, opt.scene))
  {
    ROS_ERROR("Scene '%s' not found in warehouse", opt.scene.c_str());
    return;
  }
  moveit_msgs::ComputePlanningBenchmark::Request req;
  moveit_msgs::ComputePlanningBenchmark::Request res;
  req.scene = static_cast<const moveit_msgs::PlanningScene&>(*pswm);
  std::vector<moveit_warehouse::MotionPlanRequestWithMetadata> planning_queries;
  pss.getPlanningQueries(planning_queries, opt.scene);
  if (planning_queries.empty())
    ROS_WARN("Scene '%s' has no associated queries", opt.scene.c_str());
  req.filename = opt.output;
  req.planner_interfaces.resize(opt.plugins.size());
  req.average_count.resize(req.planner_interfaces.size());
  for (std::size_t i = 0 ; i < opt.plugins.size() ; ++i)
  {
    req.planner_interfaces[i].name = opt.plugins[i].name;
    req.planner_interfaces[i].planner_ids = opt.plugins[i].planners;
    req.average_count[i] = opt.plugins[i].runs;
  }
  ros::NodeHandle nh;
  ros::service::waitForService(BENCHMARK_SERVICE_NAME);
  ros::ServiceClient benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningBenchmark>(BENCHMARK_SERVICE_NAME);
  
  for (std::size_t i = 0 ; i < planning_queries.size() ; ++i)
  {
    req.motion_plan_request = static_cast<const moveit_msgs::MotionPlanRequest&>(*planning_queries[i]);
    ROS_INFO("Calling benchmark %u of %u for scene '%s' ...", (unsigned int)(i + 1), (unsigned int)planning_queries.size(), opt.scene.c_str());
    if (benchmark_service_client.call(req, res))
      ROS_INFO("Success! Log data saved to '%s'", res.filename.c_str());    
    else
      ROS_ERROR("Failed!");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "call_planning_scene_benchmark", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // load the planning plugins
  boost::shared_ptr<pluginlib::ClassLoader<planning_interface::Planner> > planner_plugin_loader;
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::Planner>("planning_interface", "planning_interface::Planner"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  
  const std::vector<std::string> &plugins = planner_plugin_loader->getDeclaredClasses();
  if (plugins.empty())
    ROS_ERROR("There are no plugins to benchmark.");
  else
  { 
    unsigned int proc = 0;
    moveit_warehouse::PlanningSceneStorage pss;
    for (int i = 1 ; i < argc ; ++i)
    {
      BenchmarkOptions opt;
      if (readOptions(argv[i], opt))
      {
        std::stringstream ss;
        printOptions(ss, opt);
        ROS_INFO("Calling benchmark with options:\n%s\n", ss.str().c_str());
        runBenchmark(pss, opt);
        proc++;
      }
    }
    ROS_INFO("Processed %u benchmark configuration files", proc);
  }
  
  return 0;
}
