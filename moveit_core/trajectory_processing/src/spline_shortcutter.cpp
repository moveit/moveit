/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Sachin Chitta */

#include <trajectory_processing/spline_shortcutter.h>
#include <trajectory_processing/trajectory_processing_utils.h>
#include <random_numbers/random_numbers.h>

namespace trajectory_processing
{

SplineShortcutter::SplineShortcutter(double discretization) :
  discretization_(discretization)
{
}

SplineShortcutter::~SplineShortcutter()
{
}

bool SplineShortcutter::shortcut(const planning_scene::PlanningSceneConstPtr& scene,
                                      const std::string& group,
                                      const planning_models::KinematicState* start_state,
                                      const std::vector<moveit_msgs::JointLimits>& limits,
                                      const moveit_msgs::Constraints& path_constraints,
                                      const moveit_msgs::Constraints& goal_constraints,
                                      const trajectory_msgs::JointTrajectory& trajectory_in,
                                      const ros::Duration& allowed_time,
                                      trajectory_msgs::JointTrajectory& trajectory_out,
                                      moveit_msgs::MoveItErrorCodes& error_code) const
{
  ROS_INFO("Got trajectory with %d points",(int)trajectory_in.points.size());
  
  spline_msgs::SplineTrajectory spline;

  //TODO - get this function
  //collision_models_interface_->disableCollisionsForNonUpdatedLinks(trajectory_in.group_name);

  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory = trajectory_in;

  if(!scene->isPathValid(start_state,
                         path_constraints,
                         goal_constraints,
                         traj)) {
    //TODO - get error codes
    ROS_INFO_STREAM("Original trajectory invalid");
    return false;
  }

  trajectory_out = trajectory_in;

  if (!trajectory_processing::checkTrajectoryConsistency(trajectory_out)) {
    ROS_INFO_STREAM("Trajectory is inconsistent");
    return false;
  }

  // shortcut.limits = trajectory_in.limits;
  // shortcut.joint_names = trajectory_in.joint_names;

  // discretized_trajectory.limits = trajectory_in.limits;
  // discretized_trajectory.joint_names = trajectory_in.joint_names;

  ros::WallTime start_time = ros::WallTime::now();

  bool success = parameterize(trajectory_out, limits,spline);      
  getWaypoints(spline,trajectory_out);
  //printTrajectory(trajectory_out.trajectory);
  
  for(unsigned int i = 0; i < limits.size(); i++) {
    ROS_DEBUG_STREAM("Joint " << limits[i].joint_name 
                     << " has " << (bool) limits[i].has_position_limits
                     << " low " << limits[i].min_position
                     << " high " << limits[i].max_position);
  }

  traj.joint_trajectory = trajectory_out;
  if(!scene->isPathValid(start_state,
                         path_constraints,
                         goal_constraints,
                         traj)) {
    //TODO - get error codes
    ROS_INFO_STREAM("Sampled trajectory invalid");
    return false;
  }
  
  random_numbers::RandomNumberGenerator rng;

  std::vector<double> sample_times;
  sample_times.resize(2);
  bool first_try = true;
  while(ros::WallTime::now() - start_time < ros::WallDuration(allowed_time.toSec()))
  {
    double total_time = trajectory_out.points.back().time_from_start.toSec();
    double segment_start_time = rng.uniformReal(0.0,total_time);
    double segment_end_time = rng.uniformReal(segment_start_time,total_time);
    if(segment_start_time == segment_end_time)
      continue;
    if(first_try)
    {
      segment_start_time = 0.0;
      segment_end_time = total_time;
      first_try = false;
    }
    sample_times[0] = segment_start_time;
    sample_times[1] = segment_end_time;
    trajectory_msgs::JointTrajectory shortcut;
    trajectory_processing::sampleSplineTrajectory(spline,sample_times,shortcut);
    ROS_DEBUG("Start time: %f, %f",segment_start_time,shortcut.points[0].positions[0]);
    ROS_DEBUG("End time  : %f, %f",segment_end_time,shortcut.points[1].positions[0]);
    shortcut.points[0].time_from_start = ros::Duration(0.0);
    shortcut.points[1].time_from_start = ros::Duration(0.0);
    
    spline_msgs::SplineTrajectory shortcut_spline;
    if(!parameterize(shortcut,limits,shortcut_spline))
      return false;
    trajectory_msgs::JointTrajectory discretized_trajectory;
    discretizeTrajectory(shortcut_spline,discretization_,discretized_trajectory);
    ROS_DEBUG("Succeeded in sampling trajectory with size: %d",(int)discretized_trajectory.points.size());

    moveit_msgs::Constraints empty_goal_constraints;

    traj.joint_trajectory = discretized_trajectory;
    if(scene->isPathValid(start_state,
                          path_constraints,
                          empty_goal_constraints,
                          traj)) {
      ros::Duration shortcut_duration = discretized_trajectory.points.back().time_from_start - discretized_trajectory.points.front().time_from_start;
      if(segment_end_time-segment_start_time <= shortcut_duration.toSec())
        continue;
      if(!trimTrajectory(trajectory_out,segment_start_time,segment_end_time))
        continue;
      ROS_DEBUG_STREAM("Trimmed trajectory has " << trajectory_out.points.size() << " points");

      ROS_DEBUG("Shortcut reduced duration from: %f to %f",
                segment_end_time-segment_start_time,
                shortcut_duration.toSec());
      shortcut.points[0].time_from_start = ros::Duration(segment_start_time);
      shortcut.points[1].time_from_start = ros::Duration(segment_start_time) + shortcut_duration;
      addToTrajectory(trajectory_out,
                      shortcut.points[0],
                      ros::Duration(0.0));
      addToTrajectory(trajectory_out,
                      shortcut.points[1],
                      shortcut_duration-ros::Duration(segment_end_time-segment_start_time));
      spline.segments.clear();
      if(!parameterize(trajectory_out,limits,spline))
        return false;
      if(!getWaypoints(spline,trajectory_out))
        return false;
      printTrajectory(trajectory_out);
      if(trajectory_out.points.size() < 3)
        break;
    }
    else 
    {
      ROS_DEBUG("Traj segment rejected with error code: %d",error_code.val);
      continue;
    }
  }
  ROS_INFO("Trajectory filter took %f seconds",(ros::WallTime::now() - start_time).toSec());
  for(unsigned int i=0; i < trajectory_out.points.size(); i++)
  {
    trajectory_out.points[i].accelerations.clear();
  }
  refineTrajectory(trajectory_out, limits);
  
  if(!parameterize(trajectory_out,limits,spline))
    return false;
  if(!getWaypoints(spline,trajectory_out))
    return false;

  discretizeTrajectory(spline,discretization_,trajectory_out);

  ROS_DEBUG("Final trajectory has %d points and %f total time",(int)trajectory_out.points.size(),
            trajectory_out.points.back().time_from_start.toSec());
  
  return success;
}

void SplineShortcutter::refineTrajectory(trajectory_msgs::JointTrajectory &trajectory,
                                              const std::vector<moveit_msgs::JointLimits>& limits) const
{
  if(trajectory.points.size() < 3)
    return;
  
  for(unsigned int i=1; i < trajectory.points.size()-1; i++)
  {
    for(unsigned int j=0; j < trajectory.points[i].positions.size(); j++)
    {
      double dq_first = trajectory.points[i].positions[j] - trajectory.points[i-1].positions[j];
      double dq_second = trajectory.points[i+1].positions[j] - trajectory.points[i].positions[j];
      //double dq_dot = trajectory.points[i].velocities[j];
      double dt_first = (trajectory.points[i].time_from_start - trajectory.points[i-1].time_from_start).toSec();
      double dt_second = (trajectory.points[i+1].time_from_start - trajectory.points[i].time_from_start).toSec();
      if( (dq_first > 0 && dq_second > 0) || (dq_first < 0 && dq_second < 0)) 
      {       
        if(trajectory.points[i].velocities[j] == 0.0)
        {
          trajectory.points[i].velocities[j] = 0.5*(dq_first/dt_first + dq_second/dt_second);
          trajectory.points[i].velocities[j] = std::max(std::min(trajectory.points[i].velocities[j],
                                                                 limits[j].max_velocity),
                                                        -limits[j].max_velocity);
        }
      }
    }
  }
}

void SplineShortcutter::printTrajectory(const trajectory_msgs::JointTrajectory &trajectory) const
{
  for(unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    ROS_DEBUG("%f: %f %f %f %f %f %f %f, %f %f %f %f %f %f %f, %f %f %f %f %f %f %f",
             trajectory.points[i].time_from_start.toSec(),
             trajectory.points[i].positions[0],
             trajectory.points[i].positions[1],
             trajectory.points[i].positions[2],
             trajectory.points[i].positions[3],
             trajectory.points[i].positions[4],
             trajectory.points[i].positions[5],
             trajectory.points[i].positions[6],
             trajectory.points[i].velocities[0],
             trajectory.points[i].velocities[1],
             trajectory.points[i].velocities[2],
             trajectory.points[i].velocities[3],
             trajectory.points[i].velocities[4],
             trajectory.points[i].velocities[5],
             trajectory.points[i].velocities[6],
             trajectory.points[i].accelerations[0],
             trajectory.points[i].accelerations[1],
             trajectory.points[i].accelerations[2],
             trajectory.points[i].accelerations[3],
             trajectory.points[i].accelerations[4],
             trajectory.points[i].accelerations[5],
             trajectory.points[i].accelerations[6]);
  }
  ROS_DEBUG(" ");
}

/*template <typename T>
void SplineShortcutter<T>::discretizeTrajectory(const trajectory_processing::SplineTrajectory &spline, 
                                                     const double &discretization,
                                                     trajectory_msgs::JointTrajectory &joint_trajectory) const
{
  double total_time;
  trajectory_processing::getTotalTime(spline,total_time);
  std::vector<double> times;
  double time_index = 0.0;
  while(time_index < total_time)
  {
    times.push_back(time_index);
    time_index += discretization;
  }
  times.push_back(total_time);  
  trajectory_processing::sampleSplineTrajectory(spline,times,joint_trajectory);
}
*/

void SplineShortcutter::discretizeTrajectory(const spline_msgs::SplineTrajectory &spline, 
                                                  const double &discretization,
                                                  trajectory_msgs::JointTrajectory &joint_trajectory) const
{
  if(spline.segments.empty())
    return;
  joint_trajectory.points.clear();
  ros::Duration segment_start_time(0.0);
  for(unsigned int i=0; i < spline.segments.size(); i++)
  {
    if(i == spline.segments.size()-1)
      discretizeAndAppendSegment(spline.segments[i],discretization,joint_trajectory,segment_start_time,true);
    else
      discretizeAndAppendSegment(spline.segments[i],discretization,joint_trajectory,segment_start_time,false);
    segment_start_time += spline.segments[i].duration;
    ROS_DEBUG("Discretizing and appending segment %d",i);
  }
}

void SplineShortcutter::discretizeAndAppendSegment(const spline_msgs::SplineTrajectorySegment &spline_segment,
                                                        const double &discretization,
                                                        trajectory_msgs::JointTrajectory &joint_trajectory,
                                                        const ros::Duration &segment_start_time,
                                                        const bool &include_segment_end) const
{
  ros::Duration time_from_start = segment_start_time;
  double total_time = spline_segment.duration.toSec();
  double sample_time = 0.0;
  trajectory_msgs::JointTrajectoryPoint start,end;
  trajectory_processing::sampleSplineTrajectory(spline_segment,0.0,start);
  if(joint_trajectory.points.empty())
  {
    start.time_from_start = ros::Duration(0.0);
    joint_trajectory.points.push_back(start);
    sample_time += 0.01;
  }
  start = joint_trajectory.points.back();
  while(sample_time < total_time)
  {
     ROS_DEBUG("Sample time is %f",sample_time);
     trajectory_processing::sampleSplineTrajectory(spline_segment,sample_time,end);
     double max_diff = maxLInfDistance(start,end);
     if(sample_time > 0 && max_diff < discretization)
     {
       ROS_DEBUG("Max diff is %f. Skipping",max_diff);
       sample_time += 0.01;
       continue;
     }
     end.time_from_start = time_from_start + ros::Duration(sample_time);
     joint_trajectory.points.push_back(end);
     ROS_DEBUG("Pushing back point with time: %f",end.time_from_start.toSec());
     sample_time += 0.01;
     start = end;
  }
  if(include_segment_end)
  {
    trajectory_processing::sampleSplineTrajectory(spline_segment,total_time,end);
    end.time_from_start = time_from_start + ros::Duration(total_time);
    joint_trajectory.points.push_back(end);
  }    
}

double SplineShortcutter::maxLInfDistance(const trajectory_msgs::JointTrajectoryPoint &start, 
                                               const trajectory_msgs::JointTrajectoryPoint &end) const
{
  double max_diff = 0.0;
  for(unsigned int i=0; i< start.positions.size(); i++)
  {
    double diff = fabs(end.positions[i]-start.positions[i]);
    if(diff > max_diff)
      max_diff = diff;
  }
  return max_diff;
}
      
bool SplineShortcutter::trimTrajectory(trajectory_msgs::JointTrajectory &trajectory_out, 
                                            const double &segment_start_time, 
                                            const double &segment_end_time) const
{
  int index1;
  int index2;
  if(findTrajectoryPointsInInterval(trajectory_out,segment_start_time,segment_end_time,index1,index2))
  {
    ROS_DEBUG("Trimming trajectory between segments: %d and %d",index1,index2);
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator remove_start = trajectory_out.points.begin() + index1;
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator remove_end;
    if((unsigned int) index2 >= trajectory_out.points.size())
      remove_end = trajectory_out.points.end();
    else
      remove_end = trajectory_out.points.begin()+index2;
      

    if(remove_start != remove_end)
      trajectory_out.points.erase(remove_start,remove_end);
    else
      trajectory_out.points.erase(remove_start);
  }
  else
    return false;
  return true;
}

bool SplineShortcutter::findTrajectoryPointsInInterval(const trajectory_msgs::JointTrajectory &trajectory,
                                                            const double &segment_start_time, 
                                                            const double &segment_end_time,
                                                            int &index_1,
                                                            int &index_2) const
{
  index_1 = -1;
  index_2 = -1;
  if(segment_start_time > segment_end_time)
    return false;
  for(unsigned int i=0; i < trajectory.points.size(); i++)
    if(trajectory.points[i].time_from_start.toSec() >= segment_start_time)
    {
      index_1 = i;
      break;
    }
  ROS_DEBUG("First trim index: %d",index_1);
  if(index_1>=0)
    for(unsigned int i=index_1; i < trajectory.points.size(); i++)
    {
      if(trajectory.points[i].time_from_start.toSec() > segment_end_time)
      {
        index_2 = i;
        break;
      }
      if(trajectory.points[i].time_from_start.toSec() == segment_end_time)
      {
        index_2 = i+1;
        break;
      }
    }
  ROS_DEBUG("Second trim index: %d",index_2);
  if(index_1 >= index_2 || index_1 < 0 || index_2 < 0)
    return false;
  return true;
}

bool SplineShortcutter::getWaypoints(const spline_msgs::SplineTrajectory &spline, 
                                          trajectory_msgs::JointTrajectory &joint_trajectory) const
{
  std::vector<double> waypoint_times_vector;
  double waypoint_time = 0.0;
  waypoint_times_vector.push_back(waypoint_time);
  for(unsigned int i=0; i < spline.segments.size(); i++)
  {
    waypoint_time = waypoint_time + spline.segments[i].duration.toSec();
    waypoint_times_vector.push_back(waypoint_time);
    ROS_DEBUG("Spline segment time: %f",spline.segments[i].duration.toSec());
  }
  if(!trajectory_processing::sampleSplineTrajectory(spline,waypoint_times_vector,joint_trajectory))
    return false;
  return true;
}

bool SplineShortcutter::addToTrajectory(trajectory_msgs::JointTrajectory &trajectory_out, 
                                             const trajectory_msgs::JointTrajectoryPoint &trajectory_point,
                                             const ros::Duration &delta_time) const
{
  
  ROS_DEBUG("Inserting point at time: %f",trajectory_point.time_from_start.toSec());
  ROS_DEBUG_STREAM("Old trajectory has " << trajectory_out.points.size() << " points");

  if(trajectory_out.points.empty())
  {
    trajectory_out.points.push_back(trajectory_point);
    return true;
  }

  unsigned int counter = 0;
  unsigned int old_size = trajectory_out.points.size();
  for(std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator iter = trajectory_out.points.begin(); 
      iter != trajectory_out.points.end() ; iter++)
  {   
    if(iter->time_from_start >= trajectory_point.time_from_start)
    {
      trajectory_out.points.insert(iter,trajectory_point);
      break;
    }
    counter++;
  }

  if(delta_time == ros::Duration(0.0))
    return true;

  if(counter == old_size)
    trajectory_out.points.push_back(trajectory_point);
  else
    if(counter+1 < trajectory_out.points.size())
      for(unsigned int i= counter+1; i < trajectory_out.points.size(); i++)
      {
        trajectory_out.points[i].time_from_start += delta_time;
      } 
  return true;
}

}
