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
#include <tf/transform_listener.h>

#include <moveit_msgs/ComputePlanningBenchmark.h>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/progress.hpp>

static const std::string ROBOT_DESCRIPTION="robot_description";      // name of the robot description (a param name, so it can be changed externally)
static const std::string BENCHMARK_SERVICE_NAME="benchmark_planning_problem"; // name of the advertised service (within the ~ namespace)

class BenchmarkService
{
public:
    BenchmarkService(void)
    {
	// initialize a planning scene
	robot_model_loader::RobotModelLoader rml(ROBOT_DESCRIPTION);
	rml.getRobotDescription();
	if (rml.getURDF())
	{
	    scene_.reset(new planning_scene::PlanningScene());
	    scene_->configure(rml.getURDF(), rml.getSRDF() ? rml.getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model()));
	    if (scene_->isConfigured())
	    {
		cscene_ = scene_;		
		// load the planning plugins
		try
		{
		    planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::Planner>("planning_interface", "planning_interface::Planner"));
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
			planning_interface::Planner *p = planner_plugin_loader_->createClassInstance(classes[i]);
			p->init(scene_->getKinematicModel());
			planner_interfaces_[classes[i]].reset(p);
		    }
		    catch(pluginlib::PluginlibException& ex)
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
		    benchmark_service_ = nh_.advertiseService(BENCHMARK_SERVICE_NAME, &BenchmarkService::computeBenchmark, this);
		}
	    }
	    else
		ROS_ERROR("Unable to configure planning scene");
	}
	else
	    ROS_ERROR("Unable to load URDF for parameter %s", ROBOT_DESCRIPTION.c_str());
    }
    
    bool computeBenchmark(moveit_msgs::ComputePlanningBenchmark::Request &req, moveit_msgs::ComputePlanningBenchmark::Response &res)
    {      
	if (!req.planner_interfaces.empty())
	    for (std::size_t i = 0 ; i < req.planner_interfaces.size() ; ++i)
		if (planner_interfaces_.find(req.planner_interfaces[i]) == planner_interfaces_.end())
		    ROS_ERROR("Planning interface '%s' was not found", req.planner_interfaces[i].c_str());
	
	std::vector<planning_interface::Planner*> pi;
	std::vector<planning_interface::PlannerCapability> pc;
	planning_interface::PlannerCapability capabilities;	
	moveit_msgs::GetMotionPlan::Request mp_req;
	mp_req.motion_plan_request = req.motion_plan_request;
	for (std::map<std::string, boost::shared_ptr<planning_interface::Planner> >::const_iterator it = planner_interfaces_.begin() ; 
	     it != planner_interfaces_.end(); ++it)
	{
	    if (!req.planner_interfaces.empty())
	    {
		bool found = false;
		for (std::size_t i = 0 ; i < req.planner_interfaces.size() ; ++i)
		    if (req.planner_interfaces[i] == it->first)
		    {
			found = true;
			break;
		    }
		if (!found)
		    continue;
	    }
	    
	    if (it->second->canServiceRequest(mp_req, capabilities))
	    {
		pi.push_back(it->second.get());
		pc.push_back(capabilities);
	    }
	    else
		ROS_WARN_STREAM("Planning interface '" << it->second->getDescription() << "' is not able to solve the specified benchmark problem.");
	}
	
	if (pi.empty())
	{
	    ROS_ERROR("There are no planning interfaces to benchmark");
	    return false;	    
	}

	ROS_INFO("Benchmarking planning interfaces:");
	for (std::size_t i = 0 ; i < pi.size() ; ++i)
	    ROS_INFO_STREAM("  * " << pi[i]->getDescription());
	scene_->setPlanningSceneMsg(req.scene);

	boost::progress_display progress(pi.size() * req.average_count, std::cout);
	moveit_msgs::GetMotionPlan::Response mp_res;
	for (std::size_t i = 0 ; i < pi.size() ; ++i)
	{
	    std::vector<std::map<std::string, std::string> > runs(req.average_count);
	    for (unsigned int c = 0 ; c < req.average_count ; ++c)
	    {
		++progress;
		ros::WallTime start = ros::WallTime::now();
		bool solved = pi[i]->solve(cscene_, mp_req, mp_res);
		runs[c]["runtime"] = boost::lexical_cast<std::string>((ros::WallTime::now() - start).toSec());
		runs[c]["solved"] = boost::lexical_cast<std::string>(solved);
		runs[c]["path length"] = 0.0;
	    }
	}
	
	// write results to file
	return true;
    }
    
private:

    ros::NodeHandle nh_;
    planning_scene::PlanningScenePtr scene_;
    planning_scene::PlanningSceneConstPtr cscene_;
    boost::shared_ptr<pluginlib::ClassLoader<planning_interface::Planner> > planner_plugin_loader_;
    std::map<std::string, boost::shared_ptr<planning_interface::Planner> > planner_interfaces_;
    ros::ServiceServer benchmark_service_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_scene_benchmark", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::NodeHandle nh;
    std::string topic_name = nh.resolveName("planning_scene");
    boost::program_options::options_description desc("You are running a planning scene benchmark. This program will wait for a planning scene to be received on the topic " +
						     topic_name + " and then run the benchmark configured by the command line parameters");
    desc.add_options()
	("help", "Show help message")
	("attempts", boost::program_options::value<unsigned int>(), "Number of times to execute a query.")
	("runtime", boost::program_options::value<double>(), "Amount of time (seconds) to allow planning for each attempt.")
	("plugins", boost::program_options::value<std::string>()->multitoken(), "Names of planning interface plugins to be benchmarked. All known plugins are used if this argument is not specified.");
    
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);
    
    if (vm.count("help"))
    {
	std::cout << desc << std::endl;
	return 1;
    }
    
    BenchmarkService bs;
    ros::waitForShutdown();
    
    return 0;
}
