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

#include <trajectory_smoother/spline_smoother_utils.h>
#include <spline_msgs/LSPBTrajectoryMsg.h>
#include <spline_msgs/SplineTrajectory.h>
#include <angles/angles.h>

namespace trajectory_processing
{

/*! 
  \brief Internal function that helps determine which spline segment corresponds to an input time. 
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory
  \param time Time at which trajectory needs to be sampled. time = 0.0 corresponds to start of the trajectory.
  \param spline_segment Reference to output spline segment containing the waypoint corresponding to the desired time
  \param segment_time Reference to output segment time corresponding to the waypoint in trajectory at the (absolute) input time. segment_time = 0.0 corresponds to the start of this spline segment.
  \param start_index (optional) parameter to specify the starting index to start the (linear) search for required input time
  \param end_index (optional) parameter to specify the end index for the (linear) search for required input time
*/
bool findSplineSegment(const spline_msgs::SplineTrajectory &spline,
                       const double& time, 
                       spline_msgs::SplineTrajectorySegment& spline_segment,
                       double& segment_time, 
                       int start_index, 
                       int end_index)
{
  if(end_index == -1)
    end_index = spline.segments.size();

  double segment_start_time = 0.0;
  double segment_end_time = 0.0;

  for(int i=0; i < (int)spline.segments.size(); i++)
  {
    segment_start_time = segment_end_time;
    segment_end_time += spline.segments[i].duration.toSec();
    if(time <= segment_end_time)
    {
      segment_time = time-segment_start_time;
      spline_segment = spline.segments[i];
      ROS_DEBUG("Found spline segment: %d, trajectory time: %f, segment_time: %f, segment_end_time: %f",i,time,segment_time,segment_end_time);
      return true;
    }
  }
  if(time >= segment_end_time)
  {
    ROS_DEBUG("Did not find spline segment corresponding to input time: %f",time);
    segment_time = segment_end_time - segment_start_time;
    spline_segment = spline.segments.back();
    return true;
  }
  ROS_ERROR("Should not be here in findSplineSegment. Input time: %f is invalid",time);
  return false;
}

bool findSplineSegment(const spline_msgs::LSPBTrajectoryMsg &spline,
                       const double& time, 
                       spline_msgs::LSPBTrajectorySegmentMsg& spline_segment,
                       double& segment_time, 
                       int start_index, 
                       int end_index)
{
  if(end_index == -1)
    end_index = spline.segments.size();

  double segment_start_time = 0.0;
  double segment_end_time = 0.0;

  for(int i=0; i < (int)spline.segments.size(); i++)
  {
    segment_start_time = segment_end_time;
    segment_end_time += spline.segments[i].duration.toSec();
    if(time <= segment_end_time)
    {
      segment_time = time-segment_start_time;
      spline_segment = spline.segments[i];
      ROS_DEBUG("Found spline segment: %d, trajectory time: %f, segment_time: %f, segment_end_time: %f",i,time,segment_time,segment_end_time);
      return true;
    }
  }
  if(time >= segment_end_time)
  {
    ROS_DEBUG("Did not find spline segment corresponding to input time: %f",time);
    segment_time = segment_end_time - segment_start_time;
    spline_segment = spline.segments.back();
    return true;
  }
  ROS_ERROR("Should not be here in findSplineSegment. Input time: %f is invalid",time);
  return false;
}


/*! 
  \brief Get the total time for the provided spline trajectory
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param t A reference that will be filled in with the total time for the input spline trajectory.
*/
bool getTotalTime(const spline_msgs::SplineTrajectory &spline, 
                  double &t)
{
  t = 0.0;
  for(int i=0; i < (int)spline.segments.size(); i++)
  {
    t += spline.segments[i].duration.toSec();
  }
  return true;
}


/*! 
  \brief Get the total time for the provided spline trajectory
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param t A reference that will be filled in with the total time for the input spline trajectory.
*/
bool getTotalTime(const spline_msgs::LSPBTrajectoryMsg &spline, 
                  double &t)
{
  t = 0.0;
  for(int i=0; i < (int)spline.segments.size(); i++)
  {
    t += spline.segments[i].duration.toSec();
  }
  return true;
}


bool sampleSplineTrajectory(const spline_msgs::SplineTrajectorySegment &spline, 
                            const double& input_time, 
                            trajectory_msgs::JointTrajectoryPoint &point_out)
{
  double t = input_time;
  if(t > spline.duration.toSec())
  {
    t = spline.duration.toSec();
  }
  int joint_num = spline.joints.size();
  point_out.positions.resize(joint_num);
  point_out.velocities.resize(joint_num);
  point_out.accelerations.resize(joint_num);
  for(unsigned int i=0; i< spline.joints.size(); i++)
  {
    point_out.positions[i] = 0.0;
    point_out.velocities[i] = 0.0;
    point_out.accelerations[i] = 0.0;

    for(unsigned int j =0; j < spline.joints[i].coefficients.size(); j++)
    {
      point_out.positions[i] += spline.joints[i].coefficients[j]*pow(t,j);
      if(j > 0)
        point_out.velocities[i] += j*spline.joints[i].coefficients[j]*pow(t,j-1);
      if(j > 1)
        point_out.accelerations[i] += j*(j-1)*spline.joints[i].coefficients[j]*pow(t,j-2);
    }      
  }
  point_out.time_from_start = ros::Duration(t);
  return true;
}

bool sampleSplineTrajectory(const spline_msgs::LSPBTrajectorySegmentMsg &spline, 
                            const double& input_time, 
                            trajectory_msgs::JointTrajectoryPoint &point_out)
{
  double t = input_time;
  if(t > spline.duration.toSec())
  {
    t = spline.duration.toSec();
  }
  int joint_num = spline.joints.size();
  point_out.positions.resize(joint_num);
  point_out.velocities.resize(joint_num);
  point_out.accelerations.resize(joint_num);

  double taccend(0.0), tvelend(0.0), tvel(0.0), acc(0.0), v0(0.0);

  for(int i=0; i< joint_num; i++)
  {
    taccend = spline.joints[i].quadratic_segment_duration;
    tvelend = spline.joints[i].linear_segment_duration + spline.joints[i].quadratic_segment_duration;
    tvel = spline.joints[i].linear_segment_duration;
    acc = spline.joints[i].coefficients[2]*2;
    v0 = spline.joints[i].coefficients[1];
    
    if(t <= taccend)
    {
      point_out.positions[i]  =  spline.joints[i].coefficients[0] + t * v0 + 0.5 * t * t * acc;
      point_out.velocities[i] =  spline.joints[i].coefficients[1] + t * acc;
      point_out.accelerations[i] = acc;
    }
    else if(t >= tvelend)
    {
      double dT = t - tvelend;
      point_out.positions[i] = spline.joints[i].coefficients[0] +  v0 * taccend + 0.5 * acc * taccend * taccend + acc * taccend * tvel + acc * taccend * dT - 0.5 * acc * dT * dT;
      point_out.velocities[i] = acc*taccend - acc*dT;
      point_out.accelerations[i] = -acc;
    }
    else
    {
      double dT = t - taccend;
      point_out.positions[i] = spline.joints[i].coefficients[0] +  v0 * taccend + 0.5 * acc * taccend * taccend + acc * taccend * dT;
      point_out.velocities[i] = acc * taccend;
      point_out.accelerations[i] = 0.0;
    }
  }
  point_out.time_from_start = ros::Duration(t);
  return true;
}

bool sampleSplineTrajectory(const spline_msgs::SplineTrajectory& spline, 
                            const std::vector<double> &times, 
                            trajectory_msgs::JointTrajectory& traj_out)
{
  bool success = true;
  trajectory_msgs::JointTrajectoryPoint point_out;
  spline_msgs::SplineTrajectorySegment spline_segment;
  double segment_time;
  traj_out.points.clear();
  traj_out.points.resize(times.size());
  for(int i=0; i < (int)times.size(); i++)
  {
    ROS_DEBUG("Input time:%d %f",i,times[i]);
    success = success && findSplineSegment(spline,times[i],spline_segment,segment_time);
    success = success && sampleSplineTrajectory(spline_segment,segment_time,point_out);    
    point_out.time_from_start = ros::Duration(times[i]);
    traj_out.points[i] = point_out;
  }
  traj_out.joint_names = spline.names;
  return success;
}

bool sampleSplineTrajectory(const spline_msgs::LSPBTrajectoryMsg& spline, 
                            const std::vector<double> &times, 
                            trajectory_msgs::JointTrajectory& traj_out)
{
  bool success = true;
  trajectory_msgs::JointTrajectoryPoint point_out;
  spline_msgs::LSPBTrajectorySegmentMsg spline_segment;
  double segment_time;
  traj_out.points.clear();
  traj_out.points.resize(times.size());
  for(int i=0; i < (int)times.size(); i++)
  {
    ROS_DEBUG("Input time:%d %f",i,times[i]);
    success = success && findSplineSegment(spline,times[i],spline_segment,segment_time);
    success = success && sampleSplineTrajectory(spline_segment,segment_time,point_out);    
    point_out.time_from_start = ros::Duration(times[i]);
    traj_out.points[i] = point_out;
  }
  traj_out.joint_names = spline.names;
  return success;
}

/*! 
  \brief Write the trajectory out to a file after sampling at the specified times
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param times The set of times (in seconds) where the trajectory needs to be sampled, time = 0.0 corresponds to the start of the trajectory.
  \param filename The string representation of the name of the file where the trajectory should be written
*/
bool write(const spline_msgs::SplineTrajectory &spline, 
           const std::vector<double> &times, 
           const std::string &filename)
{
  trajectory_msgs::JointTrajectory tp;
  if(!sampleSplineTrajectory(spline,times,tp))
  {
    ROS_ERROR("Input spline was not sampled successfully");
    return false;
  }

  if(tp.points.empty())
  {
    ROS_ERROR("Input trajectory is empty");
    return false;
  }

  FILE *f = fopen(filename.c_str(),"w");
  if(!f)
    return false;

  int num_joints = tp.points[0].positions.size();
  for(int i=0; i < (int) tp.points.size(); i++)
  {
    fprintf(f,"%f ",tp.points[i].time_from_start.toSec());
    for(int j=0; j < num_joints; j++)
    {
      fprintf(f,"%f ",tp.points[i].positions[j]);
    }
    for(int j=0; j < num_joints; j++)
    {
      fprintf(f,"%f ",tp.points[i].velocities[j]);
    }
    for(int j=0; j < num_joints; j++)
    {
      fprintf(f,"%f ",tp.points[i].accelerations[j]);
    }
    fprintf(f,"\n");
  }
  fclose(f);
  return true;
}

/*! 
  \brief Write the spline representation of the trajectory out
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param filename The string representation of the name of the file where the spline representation of the trajectory should be written
*/
bool writeSpline(const spline_msgs::SplineTrajectory &spline, 
                 const std::string &filename)
{
  if(spline.segments.empty())
    return false;

  int num_segments = spline.segments.size();
  int num_joints = spline.segments[0].joints.size();

  FILE *f = fopen(filename.c_str(),"w");
  if(!f)
    return false;
  for(int i=0; i < num_segments; i++)
  {
    fprintf(f,"%f ",spline.segments[i].duration.toSec());
    for(int j=0; j < num_joints; j++)
    {
      for(unsigned int k=0; k < spline.segments[i].joints[j].coefficients.size(); k++)
        fprintf(f,"%f ",spline.segments[i].joints[j].coefficients[k]);
    }
    fprintf(f,"\n");
  }
  fclose(f);
  return true;    
}

/*! 
  \brief Write the trajectory out to a file after sampling at the specified times
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param times The set of times (in seconds) where the trajectory needs to be sampled, time = 0.0 corresponds to the start of the trajectory.
  \param filename The string representation of the name of the file where the trajectory should be written
*/
bool write(const spline_msgs::LSPBTrajectoryMsg &spline, 
           const std::vector<double> &times, 
           const std::string &filename)
{
  trajectory_msgs::JointTrajectory tp;
  if(!sampleSplineTrajectory(spline,times,tp))
  {
    ROS_ERROR("Input spline was not sampled successfully");
    return false;
  }

  if(tp.points.empty())
  {
    ROS_ERROR("Input trajectory is empty");
    return false;
  }

  FILE *f = fopen(filename.c_str(),"w");
  if(!f)
    return false;

  int num_joints = tp.points[0].positions.size();
  for(int i=0; i < (int) tp.points.size(); i++)
  {
    fprintf(f,"%f ",tp.points[i].time_from_start.toSec());
    for(int j=0; j < num_joints; j++)
    {
      fprintf(f,"%f ",tp.points[i].positions[j]);
    }
    for(int j=0; j < num_joints; j++)
    {
      fprintf(f,"%f ",tp.points[i].velocities[j]);
    }
    for(int j=0; j < num_joints; j++)
    {
      fprintf(f,"%f ",tp.points[i].accelerations[j]);
    }
    fprintf(f,"\n");
  }
  fclose(f);
  return true;
}

/*! 
  \brief Write the spline representation of the trajectory out
  \return true on success, false if any failure occurs
  \param spline The input spline representation of the trajectory.
  \param filename The string representation of the name of the file where the spline representation of the trajectory should be written
*/
bool writeSpline(const spline_msgs::LSPBTrajectoryMsg &spline, 
                 const std::string &filename)
{
  if(spline.segments.empty())
    return false;

  int num_segments = spline.segments.size();
  int num_joints = spline.segments[0].joints.size();

  FILE *f = fopen(filename.c_str(),"w");
  if(!f)
    return false;
  for(int i=0; i < num_segments; i++)
  {
    fprintf(f,"%f ",spline.segments[i].duration.toSec());
    for(int j=0; j < num_joints; j++)
    {
      for(unsigned int k=0; k < spline.segments[i].joints[j].coefficients.size(); k++)
        fprintf(f,"%f ",spline.segments[i].joints[j].coefficients[k]);
    }
    fprintf(f,"\n");
  }
  fclose(f);
  return true;    
}

}
