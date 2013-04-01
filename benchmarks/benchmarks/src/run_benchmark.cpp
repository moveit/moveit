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

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <pluginlib/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/benchmarks/benchmarks_utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/ComputePlanningPluginsBenchmark.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>
#include <boost/lexical_cast.hpp>
#include <boost/progress.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/constants/constants.hpp>
#include <fstream>

static const std::string ROBOT_DESCRIPTION="robot_description";      // name of the robot description (a param name, so it can be changed externally)
static const std::string BENCHMARK_SERVICE_NAME="benchmark_planning_problem"; // name of the advertised benchmarking service
static const std::string QUERY_SERVICE_NAME="query_known_planner_interfaces"; // name of the advertised query service

class BenchmarkService
{
public:
  
  BenchmarkService() : scene_monitor_(ROBOT_DESCRIPTION)
  {
    // initialize a planning scene
    
    if (scene_monitor_.getPlanningScene())
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
          p->initialize(scene_monitor_.getRobotModel());
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
        benchmark_service_ = nh_.advertiseService(BENCHMARK_SERVICE_NAME, &BenchmarkService::computeBenchmark, this);
        query_service_ = nh_.advertiseService(QUERY_SERVICE_NAME, &BenchmarkService::queryInterfaces, this);
      }
    }
    else
      ROS_ERROR("Unable to construct planning model for parameter %s", ROBOT_DESCRIPTION.c_str());
  }
  
  bool queryInterfaces(moveit_msgs::QueryPlannerInterfaces::Request &req, moveit_msgs::QueryPlannerInterfaces::Response &res)
  {    
    for (std::map<std::string, boost::shared_ptr<planning_interface::Planner> >::const_iterator it = planner_interfaces_.begin() ; 
         it != planner_interfaces_.end(); ++it)
    {
      moveit_msgs::PlannerInterfaceDescription pi_desc;
      pi_desc.name = it->first;
      it->second->getPlanningAlgorithms(pi_desc.planner_ids);
      res.planner_interfaces.push_back(pi_desc);
    }
    return true;
  }
  
  void collectMetrics(std::map<std::string, std::string> &rundata, const planning_interface::MotionPlanDetailedResponse &mp_res, bool solved, double total_time)
  {
    const planning_scene::PlanningScene &ps = *scene_monitor_.getPlanningScene();
    
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
          ps.checkCollisionUnpadded(req, res, p.getWayPoint(k));
          if (res.collision)
            correct = false;
          if (!p.getWayPoint(k).satisfiesBounds())
            correct = false;
          double d = ps.distanceToCollisionUnpadded(p.getWayPoint(k));
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
  
  bool computeBenchmark(moveit_msgs::ComputePlanningPluginsBenchmark::Request &req, moveit_msgs::ComputePlanningPluginsBenchmark::Response &res)
  {
    if (req.benchmark_request.evaluate_goal_existence_only)
      return runGoalExistenceBenchmark(req.benchmark_request, res.benchmark_response);
    else
      return runPlanningBenchmark(req.benchmark_request, res.benchmark_response);
  }
  
  bool runPlanningBenchmark(moveit_msgs::BenchmarkPluginRequest &req, moveit_msgs::BenchmarkPluginResponse &res)
  {
    // figure out which planners to test
    if (!req.planner_interfaces.empty())
      for (std::size_t i = 0 ; i < req.planner_interfaces.size() ; ++i)
        if (planner_interfaces_.find(req.planner_interfaces[i].name) == planner_interfaces_.end())
          ROS_ERROR("Planning interface '%s' was not found", req.planner_interfaces[i].name.c_str());
    
    res.planner_interfaces.clear();
    std::vector<planning_interface::Planner*> planner_interfaces_to_benchmark;
    std::vector<std::vector<std::string> > planner_ids_to_benchmark_per_planner_interface;
    std::vector<std::size_t> average_count_per_planner_interface;
    planning_interface::MotionPlanRequest mp_req = req.motion_plan_request;
    
    for (std::map<std::string, boost::shared_ptr<planning_interface::Planner> >::const_iterator it = planner_interfaces_.begin() ; 
         it != planner_interfaces_.end(); ++it)
    {
      int found = -1;
      if (!req.planner_interfaces.empty())
      {
        for (std::size_t i = 0 ; i < req.planner_interfaces.size() ; ++i)
        {
          if (req.planner_interfaces[i].name == it->first)
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
        res.planner_interfaces.resize(res.planner_interfaces.size() + 1);
        res.planner_interfaces.back().name = it->first;
        planner_interfaces_to_benchmark.push_back(it->second.get());
        planner_ids_to_benchmark_per_planner_interface.resize(planner_ids_to_benchmark_per_planner_interface.size() + 1);
        average_count_per_planner_interface.resize(average_count_per_planner_interface.size() + 1, std::max<std::size_t>(1, req.default_average_count));
        std::vector<std::string> known;
        it->second->getPlanningAlgorithms(known);

        if (found < 0 || req.planner_interfaces[found].planner_ids.empty())
          planner_ids_to_benchmark_per_planner_interface.back() = known;
        else
        {
          if ((int)req.average_count.size() > found)
            average_count_per_planner_interface.back() = std::max<std::size_t>(1, req.average_count[found]);
          for (std::size_t k = 0 ; k < req.planner_interfaces[found].planner_ids.size() ; ++k)
          {
            bool fnd = false;
            for (std::size_t q = 0 ; q < known.size() ; ++q)
              if (known[q] == req.planner_interfaces[found].planner_ids[k] ||
                  mp_req.group_name + "[" + req.planner_interfaces[found].planner_ids[k] + "]" == known[q])
              {
                fnd = true;
                break;
              }
            if (fnd)
              planner_ids_to_benchmark_per_planner_interface.back().push_back(req.planner_interfaces[found].planner_ids[k]);
            else
            {
              ROS_ERROR("The planner id '%s' is not known to the planning interface '%s'", req.planner_interfaces[found].planner_ids[k].c_str(), it->first.c_str());
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
      res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      return true;
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
    
    if (req.scene.robot_model_name != scene_monitor_.getRobotModel()->getName())
    {
      // if we have a different robot, use the world geometry only
      
      // clear all geometry from the scene
      scene_monitor_.getPlanningScene()->getWorldNonConst()->clearObjects();
      scene_monitor_.getPlanningScene()->getCurrentStateNonConst().clearAttachedBodies();
      scene_monitor_.getPlanningScene()->getCurrentStateNonConst().setToDefaultValues();
      
      scene_monitor_.getPlanningScene()->processPlanningSceneWorldMsg(req.scene.world);
    }
    else
      scene_monitor_.getPlanningScene()->usePlanningSceneMsg(req.scene);
    
    res.responses.resize(planner_interfaces_to_benchmark.size());

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
          bool solved = planner_interfaces_to_benchmark[i]->solve(scene_monitor_.getPlanningScene(), mp_req, mp_res);
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
            mp_res.getMessage(res.responses[i]);
          }
        }
        data.push_back(runs);
      }
    }
    
    double duration = (ros::WallTime::now() - startTime).toSec();
    std::string host = moveit_benchmarks::getHostname();
    res.filename = req.filename.empty() ? ("moveit_benchmarks_" + host + "_" + boost::posix_time::to_iso_extended_string(startTime.toBoost()) + ".log") : req.filename;
    std::ofstream out(res.filename.c_str());
    out << "Experiment " << (scene_monitor_.getPlanningScene()->getName().empty() ? "NO_NAME" : scene_monitor_.getPlanningScene()->getName()) << std::endl;
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
    ROS_INFO("Results saved to '%s'", res.filename.c_str());
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
  }

  bool runGoalExistenceBenchmark(moveit_msgs::BenchmarkPluginRequest &req, moveit_msgs::BenchmarkPluginResponse &res)
  {
    // configure planning context
    if (req.scene.robot_model_name != scene_monitor_.getRobotModel()->getName())
    {
      // if we have a different robot, use the world geometry only
      // clear all geometry from the scene
      scene_monitor_.getPlanningScene()->getWorldNonConst()->clearObjects();
      scene_monitor_.getPlanningScene()->getCurrentStateNonConst().clearAttachedBodies();
      scene_monitor_.getPlanningScene()->getCurrentStateNonConst().setToDefaultValues();

      scene_monitor_.getPlanningScene()->processPlanningSceneWorldMsg(req.scene.world);
      scene_monitor_.getPlanningScene()->setName(req.scene.name);
    }
    else
      scene_monitor_.getPlanningScene()->usePlanningSceneMsg(req.scene);

    // \todo the code below needs to be replaced with using constraint samplers;
    
    if (req.motion_plan_request.goal_constraints.size() == 0 &&
        req.motion_plan_request.goal_constraints[0].position_constraints.size() == 0 &&
        req.motion_plan_request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.size() == 0 &&
        req.motion_plan_request.goal_constraints[0].orientation_constraints.size() == 0 &&
        req.motion_plan_request.trajectory_constraints.constraints.size() == 0)
    {
      ROS_ERROR("Invalid goal constraints");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
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

      robot_state::RobotState robot_state(scene_monitor_.getPlanningScene()->getCurrentState());
      robot_state::robotStateMsgToRobotState(req.motion_plan_request.start_state, robot_state);

      // Compute IK
      ROS_INFO_STREAM("Processing goal " << req.motion_plan_request.goal_constraints[0].name << " ...");
      ros::WallTime startTime = ros::WallTime::now();
      success = robot_state.getJointStateGroup(req.motion_plan_request.group_name)->setFromIK(ik_pose, req.motion_plan_request.num_planning_attempts,
                                                                                              req.motion_plan_request.allowed_planning_time,
                                                                                              boost::bind(&BenchmarkService::isIKSolutionCollisionFree, this, &reachable, _1, _2));
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
      res.filename = req.filename.empty() ? ("moveit_benchmarks_" + host + "_" + boost::posix_time::to_iso_extended_string(startTime.toBoost()) + ".log") : req.filename;
      std::ofstream out(res.filename.c_str());
      out << "Experiment " << (scene_monitor_.getPlanningScene()->getName().empty() ? "NO_NAME" : scene_monitor_.getPlanningScene()->getName()) << std::endl;
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
      ROS_INFO("Results saved to '%s'", res.filename.c_str());
    }

    if (req.motion_plan_request.trajectory_constraints.constraints.size() > 0)
    {
      // Compute IK on trajectory constraints
      // Start Log
      ros::WallTime startTime = ros::WallTime::now();
      std::string host = moveit_benchmarks::getHostname();
      res.filename = req.filename.empty() ? ("moveit_benchmarks_" + host + "_" + boost::posix_time::to_iso_extended_string(startTime.toBoost()) + ".log") : req.filename;
      std::ofstream out(res.filename.c_str());
      out << "Experiment " << (scene_monitor_.getPlanningScene()->getName().empty() ? "NO_NAME" : scene_monitor_.getPlanningScene()->getName()) << std::endl;
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

        robot_state::RobotState robot_state(scene_monitor_.getPlanningScene()->getCurrentState());
        robot_state::robotStateMsgToRobotState(req.motion_plan_request.start_state, robot_state);

        // Compute IK
        ROS_INFO_STREAM("Processing trajectory waypoint " << req.motion_plan_request.trajectory_constraints.constraints[tc].name << " ...");
        startTime = ros::WallTime::now();
        success = robot_state.getJointStateGroup(req.motion_plan_request.group_name)->setFromIK(ik_pose, req.motion_plan_request.num_planning_attempts,
                                                                                                req.motion_plan_request.allowed_planning_time,
                                                                                                boost::bind(&BenchmarkService::isIKSolutionCollisionFree, this, &reachable, _1, _2));
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
      ROS_INFO("Results saved to '%s'", res.filename.c_str());
    }


    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
  }
  
  void status() const
  {
  }
  
private:
  
  bool isIKSolutionCollisionFree(bool *reachable, robot_state::JointStateGroup *group, const std::vector<double> &ik_solution) const
  {
    group->setVariableValues(ik_solution);
    *reachable = true;
    if (scene_monitor_.getPlanningScene()->isStateColliding(*group->getRobotState(), group->getName(), false))
      return false;
    else
      return true;
  }
  
  ros::NodeHandle nh_;
  planning_scene_monitor::PlanningSceneMonitor scene_monitor_;
  boost::shared_ptr<pluginlib::ClassLoader<planning_interface::Planner> > planner_plugin_loader_;
  std::map<std::string, boost::shared_ptr<planning_interface::Planner> > planner_interfaces_;
  ros::ServiceServer benchmark_service_;
  ros::ServiceServer query_service_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  BenchmarkService bs;
  bs.status();
  ros::waitForShutdown();
  
  return 0;
}
