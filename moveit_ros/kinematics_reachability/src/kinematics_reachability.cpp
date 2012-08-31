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

#include <kinematics_reachability/kinematics_reachability.h>

namespace kinematics_reachability
{

KinematicsReachability::KinematicsReachability():node_handle_("~")
{
  visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("workspace_markers",0,true);
  workspace_publisher_ = node_handle_.advertise<kinematics_reachability::WorkspacePoints>("workspace",0,true);
  tool_offset_.setIdentity();
  tool_offset_inverse_.setIdentity();
}

bool KinematicsReachability::getOnlyReachableWorkspace(kinematics_reachability::WorkspacePoints &workspace, const geometry_msgs::Pose &tool_frame_offset)
{
  if(!computeWorkspace(workspace, tool_frame_offset))
    return false;
  removeUnreachableWorkspace(workspace);
  return true;
}

bool KinematicsReachability::computeWorkspace(kinematics_reachability::WorkspacePoints &workspace, const geometry_msgs::Pose &tool_frame_offset)
{
  setToolFrameOffset(tool_frame_offset);
  if(!sampleUniform(workspace))
    return false;
  findIKSolutions(workspace);
  return true;
}

bool KinematicsReachability::computeWorkspace(kinematics_reachability::WorkspacePoints &workspace)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  return computeWorkspace(workspace,pose);
}

bool KinematicsReachability::isActive()
{
  //  return kinematics_solver_.isActive();
  return true;  
}

void KinematicsReachability::findIKSolutions(kinematics_reachability::WorkspacePoints &workspace)
{
  kinematics_msgs::GetConstraintAwarePositionIK::Request request;
  kinematics_msgs::GetConstraintAwarePositionIK::Response response;
  getDefaultIKRequest(request);
  for(unsigned int i=0; i < workspace.points.size(); i++)
  {
    tf::Pose tmp_pose;
    tf::poseMsgToTF(workspace.points[i].pose,tmp_pose);
    tmp_pose = tmp_pose * tool_offset_inverse_;
    tf::poseTFToMsg(tmp_pose,workspace.points[i].pose);

    request.ik_request.pose_stamped.header = workspace.header;    
    request.ik_request.pose_stamped.pose = workspace.points[i].pose;
    //    kinematics_solver_.getConstraintAwarePositionIK(request,response);
    workspace.points[i].solution_code = response.error_code;
    if(response.error_code.val == response.error_code.SUCCESS)
    {      
      ROS_DEBUG("Solution   : Point %d of %d",(int) i,(int) workspace.points.size());
      workspace.points[i].robot_state = response.solution;
    }
    else
    {
      ROS_DEBUG("No Solution: Point %d of %d",(int) i,(int) workspace.points.size());
    }
    if(i%1000 == 0)
      ROS_INFO("At sample %d",i);
  }
}

void KinematicsReachability::getPositionIndexedArrowMarkers(const kinematics_reachability::WorkspacePoints &workspace,
                                                               const std::string &marker_namespace,
                                                               visualization_msgs::MarkerArray &marker_array)
{
  unsigned int x_num_points,y_num_points,z_num_points;
  getNumPoints(workspace,x_num_points,y_num_points,z_num_points);
  unsigned int num_rotations = workspace.orientations.size();
  unsigned int num_positions = x_num_points*y_num_points*z_num_points;

  visualization_msgs::Marker marker;
  marker.type = marker.ARROW;
  marker.action = 0;
  for(unsigned int i=0; i < num_positions; i++)
  {
    unsigned int start_index = i*num_rotations;
    unsigned int end_index = (i+1)*num_rotations;
    moveit_msgs::MoveItErrorCodes error_code;
    for(unsigned int j = start_index; j < end_index; j++)
    {
      if(workspace.points[j].solution_code.val == workspace.points[j].solution_code.SUCCESS)
      {
        marker.ns = marker_namespace + "/reachable";
        marker.color.r = 0.0;
        marker.color.g = 1.0;
      }
      else
      {
        marker.ns = marker_namespace + "/unreachable";
        marker.color.r = 1.0;
        marker.color.g = 0.0;
      }
      marker.header = workspace.header;
      marker.pose = workspace.points[j].pose;
      marker.header.stamp = ros::Time::now();
      
      marker.scale.x = 0.08;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      
      marker.id = j;
      marker.color.a = 1.0;
      marker.color.b = 0.0;
      marker_array.markers.push_back(marker);
    }
  }
}

void KinematicsReachability::getPositionIndexedMarkers(const kinematics_reachability::WorkspacePoints &workspace,
                                                          const std::string &marker_namespace,
                                                          visualization_msgs::MarkerArray &marker_array)
{
  unsigned int x_num_points,y_num_points,z_num_points;
  getNumPoints(workspace,x_num_points,y_num_points,z_num_points);
  unsigned int num_rotations = workspace.orientations.size();
  unsigned int num_positions = x_num_points*y_num_points*z_num_points;

  visualization_msgs::Marker marker;
  marker.type = marker.SPHERE;
  marker.action = 0;
  for(unsigned int i=0; i < num_positions; i++)
  {
    unsigned int start_index = i*num_rotations;
    unsigned int end_index = (i+1)*num_rotations;
    unsigned int num_solutions = 0;
    moveit_msgs::MoveItErrorCodes error_code;
    for(unsigned int j = start_index; j < end_index; j++)
    {
      if(workspace.points[j].solution_code.val == workspace.points[j].solution_code.SUCCESS)
        num_solutions++;
    }
    double color_scale = num_solutions/(double) num_rotations;
    marker.header = workspace.header;
    marker.pose = workspace.points[start_index].pose;
    marker.header.stamp = ros::Time::now();

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.id = i;
    if(num_solutions > 0)    
    {
      marker.ns = marker_namespace + "/reachable";
      marker.color.r = 0.0;
      marker.color.g = color_scale;
    }
    else
    {
      marker.ns = marker_namespace + "/unreachable";
      marker.color.r = 1.0;
      marker.color.g = 0.0;
    }
      marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
  }
}

void KinematicsReachability::getMarkers(const kinematics_reachability::WorkspacePoints &workspace,
                                           const std::string &marker_namespace,
                                           std::vector<const kinematics_reachability::WorkspacePoint*> points,
                                           visualization_msgs::MarkerArray &marker_array)
{
  visualization_msgs::Marker marker;
  marker.type = marker.SPHERE;
  marker.action = 0;
  for(unsigned int i=0; i < points.size(); i++)
  {
    moveit_msgs::MoveItErrorCodes error_code;
    if(points[i]->solution_code.val == points[i]->solution_code.SUCCESS)
    {
        marker.color.g = 1.0;
        marker.color.r = 0.0;
        marker.ns = marker_namespace + "/reachable";
    }
    else
    {
      marker.color.g = 0.0;
      marker.color.r = 1.0;
      marker.ns = marker_namespace + "/unreachable";
    }
    marker.header = workspace.header;
    marker.pose = points[i]->pose;
    marker.header.stamp = ros::Time::now();

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.id = i;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
  }
}

void KinematicsReachability::visualize(const kinematics_reachability::WorkspacePoints &workspace,
                                          const std::string &marker_namespace)
{
  visualization_msgs::MarkerArray marker_array;
  getPositionIndexedMarkers(workspace,marker_namespace,marker_array);
  visualization_publisher_.publish(marker_array);
}

void KinematicsReachability::setToolFrameOffset(const geometry_msgs::Pose &pose)
{
  tf::poseMsgToTF(pose,tool_offset_);
  tool_offset_inverse_ = tool_offset_.inverse();
}

void KinematicsReachability::visualizeWithArrows(const kinematics_reachability::WorkspacePoints &workspace,
                                                    const std::string &marker_namespace)
{
  visualization_msgs::MarkerArray marker_array;
  getPositionIndexedArrowMarkers(workspace,marker_namespace,marker_array);
  visualization_publisher_.publish(marker_array);
}

void KinematicsReachability::publishWorkspace(const kinematics_reachability::WorkspacePoints &workspace)
{
  workspace_publisher_.publish(workspace); 
}

void KinematicsReachability::visualize(const kinematics_reachability::WorkspacePoints &workspace,
                                          const std::string &marker_namespace,
                                          const geometry_msgs::Quaternion &orientation)
{
  visualization_msgs::MarkerArray marker_array;
  std::vector<const kinematics_reachability::WorkspacePoint*> points = getPointsAtOrientation(workspace,orientation);
  getMarkers(workspace,marker_namespace,points,marker_array);
  visualization_publisher_.publish(marker_array);
}

void KinematicsReachability::visualize(const kinematics_reachability::WorkspacePoints &workspace,
                                          const std::string &marker_namespace,
                                          const std::vector<geometry_msgs::Quaternion> &orientations)
{
  visualization_msgs::MarkerArray marker_array;
  for(unsigned int i=0; i < orientations.size(); i++)
  {
    std::vector<const kinematics_reachability::WorkspacePoint*> points = getPointsAtOrientation(workspace,orientations[i]);
    std::ostringstream name;
    name << "orientation_" << i;
    std::string marker_name = marker_namespace+name.str();
    getMarkers(workspace,marker_name,points,marker_array);
  }
  visualization_publisher_.publish(marker_array);
}
                                        
void KinematicsReachability::getDefaultIKRequest(kinematics_msgs::GetConstraintAwarePositionIK::Request &req)
{
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  //  kinematics_solver_.getIKSolverInfo(request,response);

  req.timeout = ros::Duration(5.0);
  req.ik_request.ik_link_name = response.kinematic_solver_info.link_names[0];
  req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
  req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  for(unsigned int i=0; i < response.kinematic_solver_info.joint_names.size(); i++)
    req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
}

void KinematicsReachability::removeUnreachableWorkspace(kinematics_reachability::WorkspacePoints &workspace)
{
  unsigned int remove_counter = 0;
  moveit_msgs::MoveItErrorCodes error_code;
  std::vector<kinematics_reachability::WorkspacePoint>::iterator it = workspace.points.begin();
  while(it != workspace.points.end())
  {
    if(it->solution_code.val != it->solution_code.SUCCESS)
    {
      it = workspace.points.erase(it);
      remove_counter++;
    }
    else
      ++it;
  }
  if(remove_counter)
    ROS_DEBUG("Removed %d points from workspace",remove_counter);
}

std::vector<const kinematics_reachability::WorkspacePoint*> KinematicsReachability::getPointsAtOrientation(const kinematics_reachability::WorkspacePoints &workspace,
                                                                                                      const geometry_msgs::Quaternion &orientation)
{
  std::vector<const kinematics_reachability::WorkspacePoint*> wp;
  for(unsigned int i = 0; i < workspace.points.size(); i++)
  {
    if(isEqual(workspace.points[i].pose.orientation,orientation))
      wp.push_back(&(workspace.points[i]));
  }
  return wp;
}

bool KinematicsReachability::isEqual(const geometry_msgs::Quaternion &orientation_1, 
                                        const geometry_msgs::Quaternion &orientation_2)
{
  tf::Quaternion quat_1,quat_2;
  tf::quaternionMsgToTF(orientation_1,quat_1);
  tf::quaternionMsgToTF(orientation_2,quat_2);
  if(quat_1.angleShortestPath(quat_2) < 0.001)
    return true;
  return false;
}

std::vector<const kinematics_reachability::WorkspacePoint*> KinematicsReachability::getPointsWithinRange(const kinematics_reachability::WorkspacePoints &workspace,
                                                                                                    const double min_radius,
                                                                                                    const double max_radius)
{
  std::vector<const kinematics_reachability::WorkspacePoint*> wp;
  for(unsigned int i = 0; i < workspace.points.size(); i++)
  {
    tf::Vector3 vector;
    tf::pointMsgToTF(workspace.points[i].pose.position,vector);
    if(vector.length() >= min_radius && vector.length() <= max_radius)
      wp.push_back(&(workspace.points[i]));
  }
  return wp;
}

void KinematicsReachability::getNumPoints(const kinematics_reachability::WorkspacePoints &workspace,
                                             unsigned int &x_num_points,
                                             unsigned int &y_num_points,
                                             unsigned int &z_num_points)
{
  double position_resolution = workspace.position_resolution;
  double x_dim = std::fabs(workspace.parameters.min_corner.x - workspace.parameters.max_corner.x);
  double y_dim = std::fabs(workspace.parameters.min_corner.y - workspace.parameters.max_corner.y);
  double z_dim = std::fabs(workspace.parameters.min_corner.z - workspace.parameters.max_corner.z);

  x_num_points = (unsigned int) (x_dim/position_resolution) + 1;
  y_num_points = (unsigned int) (y_dim/position_resolution) + 1;
  z_num_points = (unsigned int) (z_dim/position_resolution) + 1;

  ROS_INFO("Num points: %d %d %d",x_num_points,y_num_points,z_num_points);
}

bool KinematicsReachability::sampleUniform(kinematics_reachability::WorkspacePoints &workspace)
{
  if(workspace.orientations.empty())
  {
    ROS_ERROR("Must specify at least one orientation");
    return false;
  }

  double position_resolution = workspace.position_resolution;
  double x_min = -workspace.parameters.min_corner.x;
  double y_min = -workspace.parameters.min_corner.y;
  double z_min = -workspace.parameters.min_corner.z;

  unsigned int x_num_points,y_num_points,z_num_points;
  getNumPoints(workspace,x_num_points,y_num_points,z_num_points);

  unsigned int num_rotations = workspace.orientations.size();
  kinematics_reachability::WorkspacePoint ws_point;
  geometry_msgs::Pose pose;

  for(unsigned int i=0; i < x_num_points; i++)
  {
    pose.position.x = x_min + i * position_resolution;
    for(unsigned int j=0; j < y_num_points; j++)
    {
      pose.position.y = y_min + j * position_resolution;
      for(unsigned int k=0; k < z_num_points; k++)
      {
        pose.position.z = z_min + k * position_resolution;
        for(unsigned int m=0; m < num_rotations; m++)
        {
          tf::Vector3 point(pose.position.x,pose.position.y,pose.position.z);
          tf::pointTFToMsg(point,ws_point.pose.position);
          ws_point.pose.orientation = workspace.orientations[m];
          workspace.points.push_back(ws_point);
        }
      }
    }
  }
  ROS_INFO("Generated %d samples",(int) workspace.points.size());
  return true;
}

} // namespace
