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

#include <moveit/kinematics_reachability/kinematics_reachability.h>

namespace kinematics_reachability
{

const static unsigned int ARROW_MARKER_OFFSET = 4;

KinematicsReachability::KinematicsReachability(const kinematics_constraint_aware::KinematicsConstraintAwarePtr &kinematics_solver,
                                               const planning_scene_monitor::PlanningSceneMonitorPtr &planning_scene_monitor) : node_handle_("~"), 
                                                                                                                                kinematics_solver_(kinematics_solver), 
                                                                                                                                planning_scene_monitor_(planning_scene_monitor),
                                                                                                                                canceled_(false)
{
}

bool KinematicsReachability::initialize()
{
  //visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("workspace_markers",0,true);
  visualization_success_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("workspace_markers/reachable",0,true);
  visualization_fail_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("workspace_markers/unreachable",0,true);
  visualization_evaluating_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("workspace_markers/evaluating",0,true);
  visualization_manipulability_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("workspace_markers/manipulability",0,true);
  visualization_orientation_success_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("workspace_markers/orientation_percentage",0,true); 
  workspace_publisher_ = node_handle_.advertise<moveit_ros_planning::WorkspacePoints>("workspace",0,true);
  boundary_publisher_ = node_handle_.advertise<moveit_ros_planning::WorkspacePoints>("workspace_boundary",0,true);
  robot_trajectory_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("display_state",0,true);
  progress_publisher_ = node_handle_.advertise<moveit_ros_planning::Progress>("planner_progress", 0, false);
  tool_offset_.setIdentity();
  tool_offset_inverse_.setIdentity();
  
  node_handle_.param("cache_origin/x", default_cache_options_.origin.x, 0.0);
  node_handle_.param("cache_origin/y", default_cache_options_.origin.y, 0.0);
  node_handle_.param("cache_origin/z", default_cache_options_.origin.z, 0.0);

  node_handle_.param("cache_workspace_size/x", default_cache_options_.workspace_size[0], 2.0);
  node_handle_.param("cache_workspace_size/y", default_cache_options_.workspace_size[1], 2.0);
  node_handle_.param("cache_workspace_size/z", default_cache_options_.workspace_size[2], 2.0);
  node_handle_.param("cache_workspace_resolution/x", default_cache_options_.resolution[0], 0.01);
  node_handle_.param("cache_workspace_resolution/y", default_cache_options_.resolution[1], 0.01);
  node_handle_.param("cache_workspace_resolution/z", default_cache_options_.resolution[2], 0.01);
  int tmp;  
  node_handle_.param("cache_num_solutions_per_point", tmp, 1);
  default_cache_options_.max_solutions_per_grid_location = (unsigned int) tmp;
  
  if(!node_handle_.getParam("cache_filename", cache_filename_))
  {
    ROS_ERROR("Must specify cache_filename");
    return false;
  }  
  node_handle_.param<double>("cache_timeout",default_cache_timeout_,60.0);  
  node_handle_.param<double>("kinematics_solver_timeout",kinematics_solver_timeout_,5.0);  
  node_handle_.param<int>("max_fk_points",max_fk_points_,5000);

  // Visualization
  node_handle_.param("arrow_marker_scale/x", arrow_marker_scale_.x, 0.07);
  node_handle_.param("arrow_marker_scale/y", arrow_marker_scale_.y, 0.005);
  node_handle_.param("arrow_marker_scale/z", arrow_marker_scale_.z, 0.005);

  double sphere_marker_radius;
  node_handle_.param("sphere_marker_radius", sphere_marker_radius, 0.02);
  sphere_marker_scale_.x = sphere_marker_radius;
  sphere_marker_scale_.y = sphere_marker_radius;
  sphere_marker_scale_.z = sphere_marker_radius;

  initializeColor("reachable_color",reachable_color_,0.0,1.0,0.0);
  initializeColor("unreachable_color",unreachable_color_,1.0,0.0,0.0);
  initializeColor("evaluating_color",evaluating_color_,0.0,0.0,1.0);
  initializeColor("default_manipulability_color",default_manipulability_color_,0.5,0.5,0.5);
  initializeColor("default_orientation_color",default_manipulability_color_,0.5,0.5,0.5);

  first_time_ = true;  
  use_cache_ = false;  

  ROS_INFO("Initialized: Waiting for request");  
  return true;  
}

void KinematicsReachability::initializeColor(const std::string &color_name,
                                             std_msgs::ColorRGBA &color_msg,
                                             double default_r,
                                             double default_g,
                                             double default_b)
{
  double color;  
  node_handle_.param(color_name+"/r", color, default_r);
  color_msg.r = color;  
  node_handle_.param(color_name+"/g", color, default_g);
  color_msg.g = color;  
  node_handle_.param(color_name+"/b", color, default_b);
  color_msg.b = color; 
  color_msg.a = 1.0;  
}


/////////////////////////////////////////////////////////////////////////////////////////////
// Public API Workspace Functions ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


bool KinematicsReachability::computeWorkspace(moveit_ros_planning::WorkspacePoints &workspace, 
                                              bool visualize)
{
  if(first_time_)
  {  
    /*    if(generateCache(workspace.group_name,default_cache_timeout_,default_cache_options_,cache_filename_))
      use_cache_ = true;    
    first_time_ = false;    
    */
  }

  planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor_->lockSceneRead();    

  setToolFrameOffset(workspace.tool_frame_offset);
  if(!sampleUniform(workspace))
  {
    planning_scene_monitor_->unlockSceneRead();
    return false;
  }
  
  if(visualize)
    visualizeWorkspaceSamples(workspace);
  
  findIKSolutions(workspace,visualize);
  planning_scene_monitor_->unlockSceneRead();

  return true;
}

bool KinematicsReachability::computeWorkspaceFK(moveit_ros_planning::WorkspacePoints &workspace,
                                                double timeout)
{
  if(workspace.group_name != kinematics_solver_->getGroupName())
  {
    ROS_ERROR("This solver is not configured for: %s", workspace.group_name.c_str());
    return false;    
  }
  
  ros::WallTime start_time = ros::WallTime::now();  
  robot_state::RobotState kinematic_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
  robot_state::JointStateGroup* joint_state_group = kinematic_state.getJointStateGroup(workspace.group_name);  

  moveit_msgs::MoveItErrorCodes error_code;  
  int points = 0;

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;  
  collision_request.group_name = kinematics_solver_->getGroupName();

  planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor_->lockSceneRead();
  std::string link_name = joint_state_group->getJointModelGroup()->getLinkModelNames().back();
  ROS_DEBUG("Link name: %s",link_name.c_str());

  while(((ros::WallTime::now()-start_time).toSec() <= timeout) && (points < max_fk_points_))
  {
    std::vector<double> joint_values;
    joint_state_group->setToRandomValues();
    joint_state_group->getVariableValues(joint_values);

    moveit_ros_planning::WorkspacePoint point;
    Eigen::Affine3d point_pose = kinematic_state.getLinkState(link_name)->getGlobalLinkTransform();    
    Eigen::Quaterniond quat(point_pose.rotation());
    Eigen::Vector3d point_trans(point_pose.translation());
    point.pose.position.x = point_trans.x();
    point.pose.position.y = point_trans.y();
    point.pose.position.z = point_trans.z();
    point.pose.orientation.x = quat.x();
    point.pose.orientation.y = quat.y();
    point.pose.orientation.z = quat.z();
    point.pose.orientation.w = quat.w();
    ROS_DEBUG("Pose: %f %f %f, %f %f %f %f", point_trans.x(), 
             point_trans.y(), 
             point_trans.z(), 
             quat.x(),
             quat.y(),
             quat.z(),
             quat.w());

    point.robot_state.joint_state.position = joint_values;
    point.robot_state.joint_state.name = joint_state_group->getJointModelGroup()->getJointModelNames();
    point.solution_code.val = point.solution_code.SUCCESS;

    planning_scene_monitor_->getPlanningScene()->checkCollision(collision_request, collision_result, kinematic_state);    
    if(collision_result.collision)
    {
      point.solution_code.val = point.solution_code.NO_IK_SOLUTION;
    }    
    workspace.points.push_back(point);
    points++;    
    ROS_DEBUG("Points: %d", points);    
  }
  workspace.header.frame_id = planning_scene_monitor_->getRobotModel()->getModelFrame();  
  planning_scene_monitor_->unlockSceneRead();
  return true;  
}

bool KinematicsReachability::getOnlyReachableWorkspace(moveit_ros_planning::WorkspacePoints &workspace, 
                                                       bool visualize)
{
  if(!computeWorkspace(workspace, visualize))
    return false;
  removeUnreachableWorkspace(workspace);
  return true;
}

bool KinematicsReachability::getManipulabilityIndex(const robot_state::RobotState &kinematic_state,
                                                    const std::string &group_name,
                                                    double &manipulability_index) const
{
  if(!checkState(kinematic_state,group_name))
    return false;
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state,group_name);
  Eigen::MatrixXd matrix = jacobian*jacobian.transpose();
  // Get manipulability index
  manipulability_index = sqrt(matrix.determinant());
  return true;
}

bool KinematicsReachability::checkState(const robot_state::RobotState &kinematic_state,
                                        const std::string &group_name) const
{
  if(!kinematic_state.hasJointStateGroup(group_name))
    return false;  
  return true;
}

Eigen::MatrixXd KinematicsReachability::getJacobian(const robot_state::RobotState &kinematic_state,
                                                    const std::string &group_name) const
{
  Eigen::MatrixXd jcbian;
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  std::string link_name = kinematic_state.getRobotModel()->getJointModelGroup(group_name)->getLinkModelNames().back();
  kinematic_state.getJointStateGroup(group_name)->getJacobian(link_name,reference_point_position,jcbian);
  return jcbian;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// IK Functions //////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

moveit_ros_planning::WorkspacePoints KinematicsReachability::computeRedundantSolutions(const std::string &group_name,
                                                                                           const geometry_msgs::PoseStamped &pose_stamped,
                                                                                           double timeout,
                                                                                           bool visualize_workspace)
{
    moveit_ros_planning::WorkspacePoints workspace;
    workspace.header = pose_stamped.header;
    workspace.group_name = group_name;    
    setToolFrameOffset(workspace.tool_frame_offset);

    bool use_cache_old_value = use_cache_;
    use_cache_ = false;    
    ros::WallTime start_time = ros::WallTime::now();    
    while(ros::WallTime::now()-start_time <= ros::WallDuration(timeout) && ros::ok())
    {
      moveit_msgs::MoveItErrorCodes error_code;
      moveit_msgs::RobotState solution;
      moveit_ros_planning::WorkspacePoint point;
      geometry_msgs::PoseStamped desired_pose = pose_stamped;    
      findIK(group_name,pose_stamped,error_code,solution);
      point.pose = pose_stamped.pose;
      point.solution_code = error_code;
      if(error_code.val == error_code.SUCCESS)
      {
        ROS_INFO("Succeeded");        
        point.robot_state = solution;
        if(visualize_workspace)
        {
          visualize(workspace,"");        
        }      
      }
      workspace.points.push_back(point);      
    }    
    use_cache_ = use_cache_old_value;    
    return workspace;    
}


void KinematicsReachability::findIKSolutions(moveit_ros_planning::WorkspacePoints &workspace,
                                             bool visualize_workspace)
{  
  double max_manipulability = 0.0;

  for(unsigned int i=0; i < workspace.points.size(); ++i)
  {
    if (canceled_)
    {
      ROS_INFO("Computation canceled.");
      return;
    }
    geometry_msgs::PoseStamped ik_pose;
    ik_pose.pose = workspace.points[i].pose;
    ik_pose.header = workspace.header;
    moveit_msgs::MoveItErrorCodes error_code;
    moveit_msgs::RobotState solution;
    
    findIK(workspace.group_name,ik_pose,error_code,solution);
    workspace.points[i].solution_code = error_code;

    std::vector<double> xyz;

    xyz.push_back(workspace.points[i].pose.position.x);
    xyz.push_back(workspace.points[i].pose.position.y);
    xyz.push_back(workspace.points[i].pose.position.z); 

    if(error_code.val == error_code.SUCCESS)
    {      
      ROS_DEBUG("Solution   : Point %d of %d",(int) i,(int) workspace.points.size());
      workspace.points[i].robot_state = solution;
      if(use_cache_)
        kinematics_cache_->addToCache(workspace.points[i].pose,solution.joint_state.position,true);
      point_map_[xyz].push_back(true);

    }
    else
    {
      ROS_ERROR("No Solution: Point %d of %d",(int) i,(int) workspace.points.size());
      point_map_[xyz].push_back(false);
    }

    if(visualize_workspace)
    {
      visualize(workspace,"solutions");
      //visualization_msgs::Marker marker;
      robot_state::RobotState kinematic_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      double manipulability_index;
      std::string group_name = "arm";
      bool manipulability_success = getManipulabilityIndex(kinematic_state, group_name, manipulability_index); 
      if (manipulability_success)
      {
        manipulability_map_[i] = manipulability_index;
        if (manipulability_index > max_manipulability)
          max_manipulability = manipulability_index;
      }
      else
      {
        ROS_INFO("No Manipulability Index Found: Point %d of %d", (int) i,(int) workspace.points.size());
      }
      
      //getManipulabilityMarkers(workspace,marker, i);
      //visualization_manipulability_publisher_.publish(marker);
      animateWorkspace(workspace,i);      
    }
    
    if(i%1000 == 0 || workspace.points.size() <= 1000)
    {
      ROS_INFO("At sample %d, (%f,%f,%f)",i,workspace.points[i].pose.position.x,workspace.points[i].pose.position.y,workspace.points[i].pose.position.z);
      moveit_ros_planning::Progress progress;
      progress.current = (int) i;
      progress.total = (int) workspace.points.size();
      progress_publisher_.publish(progress);
      ros::spinOnce();
    }
  }

  visualization_msgs::Marker marker;
  marker.id = 3;
  marker.action = 1;
  visualization_manipulability_publisher_.publish(marker);
  getManipulabilityMarkers(workspace, marker, max_manipulability);
  visualization_manipulability_publisher_.publish(marker);

  marker.id = 4;
  marker.action = 1;
  visualization_orientation_success_publisher_.publish(marker);
  getOrientationSuccessMarkers(workspace, marker);
  visualization_orientation_success_publisher_.publish(marker);

  if(use_cache_)
  {
    if(!kinematics_cache_->writeToFile(cache_filename_))
    {
      ROS_WARN("Could not write cache to file");
    }    
  }  
}

void KinematicsReachability::findIK(const std::string &group_name,
                                    const geometry_msgs::PoseStamped &pose_stamped,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    moveit_msgs::RobotState &robot_state)
{
  moveit_msgs::GetConstraintAwarePositionIK::Request request;
  moveit_msgs::GetConstraintAwarePositionIK::Response response;
  getDefaultIKRequest(group_name,request);
  tf::Pose tmp_pose;
  geometry_msgs::PoseStamped transformed_pose = pose_stamped;  
  tf::poseMsgToTF(pose_stamped.pose,tmp_pose);
  tmp_pose = tmp_pose * tool_offset_inverse_;
  tf::poseTFToMsg(tmp_pose,transformed_pose.pose);  
  request.ik_request.pose_stamped = transformed_pose;
  if(use_cache_)
  {      
    if(!updateFromCache(request))
    {
      error_code.val = error_code.NO_IK_SOLUTION;
      return;      
    }       
  }        
  kinematics_solver_->getIK(planning_scene_monitor_->getPlanningScene(),request,response);
  error_code = response.error_code;
  ROS_INFO("Error code: %d", response.error_code.val);  
  robot_state = response.solution;  
}

                                        
void KinematicsReachability::getDefaultIKRequest(const std::string &group_name,
                                                 moveit_msgs::GetConstraintAwarePositionIK::Request &req)
{
  moveit_msgs::GetKinematicSolverInfo::Request request;
  moveit_msgs::GetKinematicSolverInfo::Response response;

  robot_model::RobotModelConstPtr kinematic_model = kinematics_solver_->getRobotModel();
  robot_state::RobotState kinematic_state(kinematic_model);
  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);
  robot_state::JointStateGroup* joint_state_group = kinematic_state.getJointStateGroup(group_name);
  joint_state_group->setToRandomValues();
  
  req.timeout = ros::Duration(kinematics_solver_timeout_);
  //  req.ik_request.ik_link_name = joint_model_group->getLinkModelNames().back();
  req.ik_request.robot_state.joint_state.name = joint_model_group->getJointModelNames();
  joint_state_group->getVariableValues(req.ik_request.robot_state.joint_state.position);  
  req.ik_request.group_name = group_name;  
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Cache Functions //////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

bool KinematicsReachability::updateFromCache(moveit_msgs::GetConstraintAwarePositionIK::Request &request)
{
  geometry_msgs::Pose pose = request.ik_request.pose_stamped.pose;
  double distance_squared = (pose.position.x*pose.position.x + pose.position.y*pose.position.y + pose.position.z*pose.position.z);
  std::pair<double,double> distances;
  distances = kinematics_cache_->getMinMaxSquaredDistance(); 
  if(distance_squared >= distances.second)
    return false;  
  kinematics_cache_->getSolution(request.ik_request.pose_stamped.pose,
                                 0,
                                 request.ik_request.robot_state.joint_state.position);
  return true;  
}

bool KinematicsReachability::generateCache(const std::string &group_name,
                                           double timeout,
                                           const kinematics_cache::KinematicsCache::Options &options,
                                           const std::string &cache_filename)
{
  if(!kinematics_cache_ || kinematics_cache_->getGroupName()!= group_name)
  {    
    std::map<std::string,kinematics::KinematicsBasePtr> kinematics_solver_map = planning_scene_monitor_->getRDFLoader()->generateKinematicsSolversMap();
    if(kinematics_solver_map.find(group_name) == kinematics_solver_map.end())
    {
      ROS_ERROR("Group name: %s incorrect",group_name.c_str());      
      return false;
    }    
    kinematics::KinematicsBaseConstPtr kinematics_solver_local = kinematics_solver_map.find(group_name)->second;    
    kinematics_cache_.reset(new kinematics_cache::KinematicsCache());
    kinematics_cache_->initialize(kinematics_solver_local,
                                  kinematics_solver_->getRobotModel(),
                                  options);    
  }  
  if(!kinematics_cache_->readFromFile(cache_filename))
  {
    ROS_INFO("Generating cache map online");    
    if(!kinematics_cache_->generateCacheMap(timeout))
    {
      return false;
    }          
    if(!kinematics_cache_->writeToFile(cache_filename))
    {
      ROS_ERROR("Could not write to file");
      return false;      
    }    
  }
  return true;  
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Helper Functions //////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

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

void KinematicsReachability::publishWorkspace(const moveit_ros_planning::WorkspacePoints &workspace)
{
  workspace_publisher_.publish(workspace); 
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Workspace Functions //////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void KinematicsReachability::getPositionIndex(const moveit_ros_planning::WorkspacePoints &workspace,
                                              std::vector<unsigned int> &reachable_workspace,
                                              std::vector<unsigned int> &unreachable_workspace)
{
  for(unsigned int i=0; i < workspace.points.size(); ++i)
  {
    if(workspace.points[i].solution_code.val == workspace.points[i].solution_code.SUCCESS)
      reachable_workspace.push_back(i);
    else
      unreachable_workspace.push_back(i);
  }
}

void KinematicsReachability::removeUnreachableWorkspace(moveit_ros_planning::WorkspacePoints &workspace)
{
  unsigned int remove_counter = 0;
  moveit_msgs::MoveItErrorCodes error_code;
  std::vector<moveit_ros_planning::WorkspacePoint>::iterator it = workspace.points.begin();
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

std::vector<unsigned int> KinematicsReachability::getPointsAtOrientation(const moveit_ros_planning::WorkspacePoints &workspace,
                                                                                                           const geometry_msgs::Quaternion &orientation)
{
  std::vector<unsigned int> wp;
  for(unsigned int i = 0; i < workspace.points.size(); ++i)
  {
    if(isEqual(workspace.points[i].pose.orientation,orientation))
      wp.push_back(i);
  }
  return wp;
}

std::vector<unsigned int> KinematicsReachability::getPointsWithinRange(const moveit_ros_planning::WorkspacePoints &workspace,
                                                                       const double min_radius,
                                                                       const double max_radius)
{
  std::vector<unsigned int> wp;
  for(unsigned int i = 0; i < workspace.points.size(); ++i)
  {
    tf::Vector3 vector;
    tf::pointMsgToTF(workspace.points[i].pose.position,vector);
    if(vector.length() >= min_radius && vector.length() <= max_radius)
      wp.push_back(i);
  }
  return wp;
}

void KinematicsReachability::getNumPoints(const moveit_ros_planning::WorkspacePoints &workspace,
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

  ROS_DEBUG("Cache dimension (num grid points) in (x,y,z): %d %d %d",x_num_points,y_num_points,z_num_points);
}



bool KinematicsReachability::sampleUniform(moveit_ros_planning::WorkspacePoints &workspace)
{
  if(workspace.orientations.empty())
  {
    ROS_ERROR("Must specify at least one orientation");
    return false;
  }
  workspace.ordered = true;  
  double position_resolution = workspace.position_resolution;
  double x_min = workspace.parameters.min_corner.x;
  double y_min = workspace.parameters.min_corner.y;
  double z_min = workspace.parameters.min_corner.z;

  unsigned int x_num_points,y_num_points,z_num_points;
  getNumPoints(workspace,x_num_points,y_num_points,z_num_points);

  unsigned int num_rotations = workspace.orientations.size();
  moveit_ros_planning::WorkspacePoint ws_point;
  geometry_msgs::Pose pose;

  for(unsigned int i=0; i < x_num_points; ++i)
  {
    pose.position.x = x_min + i * position_resolution;
    for(unsigned int j=0; j < y_num_points; ++j)
    {
      pose.position.y = y_min + j * position_resolution;
      for(unsigned int k=0; k < z_num_points; ++k)
      {
        pose.position.z = z_min + k * position_resolution;
        for(unsigned int m=0; m < num_rotations; ++m)
        {
          tf::Vector3 point(pose.position.x,pose.position.y,pose.position.z);
          tf::pointTFToMsg(point,ws_point.pose.position);
          ws_point.pose.orientation = workspace.orientations[m];
          ws_point.solution_code.val = ws_point.solution_code.PLANNING_FAILED;          
          workspace.points.push_back(ws_point);
        }
      }
    }
  }
  ROS_DEBUG("Generated %d samples for workspace points",(int) workspace.points.size());
  return true;
}

void KinematicsReachability::setToolFrameOffset(const geometry_msgs::Pose &pose)
{
  tf::poseMsgToTF(pose,tool_offset_);
  tool_offset_inverse_ = tool_offset_.inverse();
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Visualization functions //////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void KinematicsReachability::getMarkers(const moveit_ros_planning::WorkspacePoints &workspace,
                                        const std::string &marker_namespace,
                                        const std::vector<unsigned int> &points,
                                        std::vector<visualization_msgs::Marker> &markers)
{
  std::vector<unsigned int> marker_ids(3);
  marker_ids[0] = 0;
  marker_ids[1] = 1;
  marker_ids[2] = 2;
  //marker_ids[3] = 3;

  std::map<int, unsigned int> error_code_map;
  moveit_msgs::MoveItErrorCodes error_code;  
  error_code_map[error_code.SUCCESS] = marker_ids[0];
  error_code_map[error_code.PLANNING_FAILED] = marker_ids[1];
  error_code_map[error_code.NO_IK_SOLUTION] = marker_ids[2];
  error_code_map[error_code.GOAL_IN_COLLISION] = marker_ids[2];

  std::vector<std_msgs::ColorRGBA> colors(3);
  colors[0] = reachable_color_;
  colors[1] = evaluating_color_;
  colors[2] = unreachable_color_;
  //colors[3] = default_manipulability_color_;

  markers = getSphereMarker(workspace,marker_namespace,points,colors,error_code_map,marker_ids);
  //std::vector<visualization_msgs::Marker> markers = getSphereMarker(workspace,marker_namespace,points,colors,error_code_map,marker_ids);    
  //for(unsigned int i=0; i < markers.size(); ++i)
    //marker_array.markers.push_back(markers[i]);  
}

void KinematicsReachability::getOrientationSuccessMarkers(const moveit_ros_planning::WorkspacePoints &workspace, visualization_msgs::Marker &marker)
{

  geometry_msgs::Point point;
  marker.type = marker.SPHERE_LIST;
  marker.action = 0;
  marker.pose.orientation.w = 1.0;
  //marker.ns = marker_namespace;
  marker.header = workspace.header;
  marker.scale = sphere_marker_scale_;
  marker.id = 4;
  marker.color = default_manipulability_color_;


  for (std::map<std::vector<double>, std::vector<bool> >::iterator it = point_map_.begin(); it!=point_map_.end(); ++it)
  {
    getColorFromSuccessList(it->second);
    if (default_orientation_color_.g != 0.0)
    {
      marker.colors.push_back(default_orientation_color_);
      point.x = it->first[0];
      point.y = it->first[1];
      point.z = it->first[2];
      marker.points.push_back(point);
    }
  }

}

void  KinematicsReachability::getColorFromSuccessList(std::vector<bool> successes)
{
  //std_msgs::ColorRGBA color_msg;
  int success = 0;
  for (std::vector<bool>::iterator it = successes.begin(); it!=successes.end(); ++it)
  {
    if (*it == true)
    {
      success++;
    }  
  }

  double r, g, b;
  b = 0.0;

  double percent = (double) success / (double) successes.size();
  if (percent >= 0.5)
  {
    g = 1.0;
    r = 2.0 * (1.0 - percent);
  }
  else
  {
    g = 2.0 * percent;
    r = 1.0;
  }

  //r = 1.0 - ((double) success / (double) successes.size());
  //g = (double) success / (double) successes.size();

  //ROS_INFO("Success: %d, Total: %d", success, successes.size());
  //ROS_INFO("Green: %f, Red: %f", g, r);
  //initializeColor("orientation_color", color_msg, r, g, b);

  default_orientation_color_.r = r;
  default_orientation_color_.g = g;
  default_orientation_color_.b = b;  

  //return color_msg;
}


void KinematicsReachability::getManipulabilityMarkers(const moveit_ros_planning::WorkspacePoints &workspace, visualization_msgs::Marker &marker, double max_manipulability)
{
  marker.type = marker.SPHERE_LIST;
  marker.action = 0;
  marker.pose.orientation.w = 1.0;
  //marker.ns = marker_namespace;
  marker.header = workspace.header;
  marker.scale = sphere_marker_scale_;
  marker.id = 3;
  marker.color = default_manipulability_color_;

  for (std::map<int,double>::iterator it = manipulability_map_.begin(); it!=manipulability_map_.end(); ++it)
  {
    geometry_msgs::Point point = workspace.points[it->first].pose.position;
    if (workspace.points[it->first].solution_code.val == 1)
    {
      getColorFromManipulability(it->second, max_manipulability);
      marker.colors.push_back(default_manipulability_color_);
      marker.points.push_back(point);
    }
  }
}

void  KinematicsReachability::getColorFromManipulability(double manipulability_index, double highest)
{
  //std_msgs::ColorRGBA color_msg;
  double r, g, b;
  g = 0.0;
  //r = 1.0 - (manipulability_index / highest);
  //g = manipulability_index / highest;

  double percent = manipulability_index / highest; 
  if (percent >= 0.5)
  {
    b = 1.0;
    r = 2.0 * (1.0 - percent);
  }
  else
  {
    b = 2.0 * percent;
    r = 1.0;
  }

  //initializeColor("manipulability_color", color_msg, r, g, b);
  default_manipulability_color_.r = r;
  default_manipulability_color_.g = g;
  default_manipulability_color_.b = b;


  //return color_msg;
}

std::vector<visualization_msgs::Marker> KinematicsReachability::getSphereMarker(const moveit_ros_planning::WorkspacePoints &workspace,
                                                                                const std::string &marker_namespace,
                                                                                const std::vector<unsigned int> &indices,
                                                                                const std::vector<std_msgs::ColorRGBA> &colors,
                                                                                const std::map<int, unsigned int> &error_code_map,
                                                                                const std::vector<unsigned int> &marker_id)
{
  std::vector<visualization_msgs::Marker> markers;
  if(marker_id.size() != colors.size())
    return markers;
  markers.resize(marker_id.size());
  
  for(unsigned int i=0; i < markers.size(); ++i)
  {
    markers[i].type = markers[i].SPHERE_LIST;
    markers[i].action = 0;
    markers[i].pose.orientation.w = 1.0;  
    markers[i].ns = marker_namespace;
    markers[i].header = workspace.header;
    markers[i].scale = sphere_marker_scale_;
    markers[i].id = marker_id[i];
    markers[i].color = colors[i];
  }
  
  if(indices.empty())
  {
    for(unsigned int i=0; i < workspace.points.size(); ++i)
    {
      geometry_msgs::Point point = workspace.points[i].pose.position;
      if(error_code_map.find(workspace.points[i].solution_code.val) == error_code_map.end())
      {
        ROS_ERROR("Unknown error code: %d",workspace.points[i].solution_code.val);
      }
      else
      {         
        unsigned int marker_index = error_code_map.find(workspace.points[i].solution_code.val)->second;          
        //        ROS_INFO("Point %d, marker index: %d", i, marker_index);        
        markers[marker_index].colors.push_back(colors[marker_index]);
        markers[marker_index].points.push_back(point); 
      }
    }    
  }
  else
  {
    for(unsigned int i=0; i < indices.size(); ++i)
    {
      if(indices[i] >= workspace.points.size())
      {
        ROS_WARN("Invalid point: %d",indices[i]);
        continue;        
      }
      geometry_msgs::Point point = workspace.points[indices[i]].pose.position;
      if(error_code_map.find(workspace.points[indices[i]].solution_code.val) == error_code_map.end())
      {
        ROS_ERROR("Unknown error code: %d",workspace.points[indices[i]].solution_code.val);
      }
      else
      {         
        unsigned int marker_index = error_code_map.find(workspace.points[indices[i]].solution_code.val)->second;         
        markers[marker_index].colors.push_back(colors[marker_index]);
        markers[marker_index].points.push_back(point); 
        
      }

    }    
  }  
  return markers;  
}

std_msgs::ColorRGBA KinematicsReachability::getMarkerColor(const moveit_ros_planning::WorkspacePoint &workspace_point)
{
  if(workspace_point.solution_code.val == workspace_point.solution_code.SUCCESS)
  {
    return reachable_color_;    
  }
  else if(workspace_point.solution_code.val == workspace_point.solution_code.NO_IK_SOLUTION || workspace_point.solution_code.val == workspace_point.solution_code.GOAL_IN_COLLISION)
  {
    return unreachable_color_;    
  }
  else
  {
    return evaluating_color_;    
  }    
}

/*void KinematicsReachability::getOrientationSuccessMarkers(const moveit_ros_planning::WorkspacePoints &workspace,
                                             visualization_msgs::Marker &marker)
{
  marker.type = marker.SPHERE_LIST;
  marker.action = 0;
  //marker.ns = marker_namespace;
  marker.id = 4;
  marker.header = workspace.header;
  marker.scale = sphere_marker_scale_;
  

  for(unsigned int i=0; i < workspace.points.size(); ++i)
  {
    marker.pose = workspace.points[i].pose;
    marker.color = getMarkerColor(workspace.points[i]);
    markers.push_back(marker);
  } 

}*/

void KinematicsReachability::getArrowMarkers(const moveit_ros_planning::WorkspacePoints &workspace,
                                             const std::string &marker_namespace,
                                             const std::vector<unsigned int> &points,
                                             std::vector<visualization_msgs::Marker> &markers)
{
  visualization_msgs::Marker marker;
  
  marker.type = marker.ARROW;
  marker.action = 0;
  marker.ns = marker_namespace;
  marker.header = workspace.header;
  marker.scale = arrow_marker_scale_;
  
  if(points.empty())
  {
    for(unsigned int i=0; i < workspace.points.size(); ++i)
    {
      marker.pose = workspace.points[i].pose;    
      marker.id = i+ARROW_MARKER_OFFSET;    
      marker.color = getMarkerColor(workspace.points[i]);    
      markers.push_back(marker);    
    }    
  }
  else
  {
    for(unsigned int i=0; i < points.size(); ++i)
    {
      if(points[i] >= workspace.points.size())
      {
        ROS_WARN("Invalid point: %d",points[i]);
        continue;        
      }
      marker.pose = workspace.points[points[i]].pose;    
      marker.id = points[i];    
      marker.color = getMarkerColor(workspace.points[points[i]]);    
      markers.push_back(marker);    
    }    
  }
}

bool KinematicsReachability::getDisplayTrajectory(const moveit_ros_planning::WorkspacePoints &workspace,
                                                  moveit_msgs::DisplayTrajectory &display_trajectory)
{
  if(workspace.points.empty())
     return false;

  std::vector<unsigned int> reachable_workspace, unreachable_workspace;
  getPositionIndex(workspace,reachable_workspace,unreachable_workspace);
  ros::Duration time_from_start(0.0);
  bool first_time(true);  
  for(unsigned int i=0; i < reachable_workspace.size(); ++i)
  {
    if(first_time)
    {
        display_trajectory.trajectory.joint_trajectory.joint_names = workspace.points[reachable_workspace[i]].robot_state.joint_state.name;
        first_time = false;
    }    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = workspace.points[reachable_workspace[i]].robot_state.joint_state.position;
    point.time_from_start = time_from_start;
    display_trajectory.trajectory.joint_trajectory.points.push_back(point);
  }
  return true;
}

bool KinematicsReachability::getDisplayTrajectory(const moveit_ros_planning::WorkspacePoint &workspace_point,
                                                  moveit_msgs::DisplayTrajectory &display_trajectory)
{
  trajectory_msgs::JointTrajectoryPoint point;
  display_trajectory.trajectory.joint_trajectory.joint_names = workspace_point.robot_state.joint_state.name;
  point.positions = workspace_point.robot_state.joint_state.position;
  display_trajectory.trajectory.joint_trajectory.points.push_back(point);
  display_trajectory.trajectory.joint_trajectory.points.push_back(point);
  return true;
}

void KinematicsReachability::animateWorkspace(const moveit_ros_planning::WorkspacePoints &workspace)
{
  moveit_msgs::DisplayTrajectory display_trajectory;
  if (!getDisplayTrajectory(workspace,display_trajectory))
  {
    ROS_WARN("No trajectory to display");
    return;
  }  
  robot_trajectory_publisher_.publish(display_trajectory);
  ROS_INFO("Animating trajectory");  
}

void KinematicsReachability::animateWorkspace(const moveit_ros_planning::WorkspacePoints &workspace,
                                              unsigned int index)
{
  moveit_msgs::DisplayTrajectory display_trajectory;
  if (index >= workspace.points.size() ||   
      workspace.points[index].solution_code.val != workspace.points[index].solution_code.SUCCESS ||
      !getDisplayTrajectory(workspace.points[index],display_trajectory))
  {
    ROS_DEBUG("No trajectory to display");
    return;
  }  
  robot_trajectory_publisher_.publish(display_trajectory);
}

void KinematicsReachability::visualize(const moveit_ros_planning::WorkspacePoints &workspace,
                                       const std::string &marker_namespace)
{
  //visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker> markers;
  std::vector<unsigned int> points;  
  getMarkers(workspace,marker_namespace,points,markers);
  //  getArrowMarkers(workspace,marker_namespace,points,markers);
  for(std::vector<visualization_msgs::Marker>::const_iterator it = markers.begin() ; it != markers.end() ; ++it)
  { 
    /*if ((it->color.r == default_manipulability_color_.r) && (it->color.g == default_manipulability_color_.g) && (it->color.b == default_manipulability_color_.b))
      visualization_manipulability_publisher_.publish(*it);
    else*/ if((it->color.r == reachable_color_.r) && (it->color.g == reachable_color_.g) && (it->color.b == reachable_color_.b) && (it->color.a == reachable_color_.a))
      visualization_success_publisher_.publish(*it);
    else if((it->color.r == unreachable_color_.r) && (it->color.g == unreachable_color_.g) && (it->color.b == unreachable_color_.b) && (it->color.a == unreachable_color_.a))
      visualization_fail_publisher_.publish(*it);
    else if((it->color.r == evaluating_color_.r) && (it->color.g == evaluating_color_.g) && (it->color.b == evaluating_color_.b) && (it->color.a == evaluating_color_.a))
      visualization_evaluating_publisher_.publish(*it);
  }
}

void KinematicsReachability::visualize(const moveit_ros_planning::WorkspacePoints &workspace,
                                       const std::string &marker_namespace,
                                       const std::vector<geometry_msgs::Quaternion> &orientations)
{
  //visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker> markers;
  for(unsigned int i=0; i < orientations.size(); ++i)
  {
    std::vector<unsigned int> points = getPointsAtOrientation(workspace,orientations[i]);
    std::ostringstream name;
    name << "orientation_" << i;
    std::string marker_name = marker_namespace+name.str();
    getMarkers(workspace,marker_name,points,markers);
  }
  //visualization_publisher_.publish(marker_array);
  for(std::vector<visualization_msgs::Marker>::const_iterator it = markers.begin() ; it != markers.end() ; ++it)
  { 
    if((it->color.r == reachable_color_.r) && (it->color.g == reachable_color_.g) && (it->color.b == reachable_color_.b) && (it->color.a == reachable_color_.a))
      visualization_success_publisher_.publish(*it);
    else if((it->color.r == unreachable_color_.r) && (it->color.g == unreachable_color_.g) && (it->color.b == unreachable_color_.b) && (it->color.a == unreachable_color_.a))
      visualization_fail_publisher_.publish(*it);
    else if((it->color.r == evaluating_color_.r) && (it->color.g == evaluating_color_.g) && (it->color.b == evaluating_color_.b) && (it->color.a == evaluating_color_.a))
      visualization_evaluating_publisher_.publish(*it);
  }
}

void KinematicsReachability::visualizeWithArrows(const moveit_ros_planning::WorkspacePoints &workspace,
                                                 const std::string &marker_namespace)
{
  //visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker> markers;
  std::vector<unsigned int> points;  
  getArrowMarkers(workspace,marker_namespace,points,markers);
  //visualization_publisher_.publish(marker_array);
  for(std::vector<visualization_msgs::Marker>::const_iterator it = markers.begin() ; it != markers.end() ; ++it)
  { 
    if((it->color.r == reachable_color_.r) && (it->color.g == reachable_color_.g) && (it->color.b == reachable_color_.b) && (it->color.a == reachable_color_.a))
      visualization_success_publisher_.publish(*it);
    else if((it->color.r == unreachable_color_.r) && (it->color.g == unreachable_color_.g) && (it->color.b == unreachable_color_.b) && (it->color.a == unreachable_color_.a))
      visualization_fail_publisher_.publish(*it);
    else if((it->color.r == evaluating_color_.r) && (it->color.g == evaluating_color_.g) && (it->color.b == evaluating_color_.b) && (it->color.a == evaluating_color_.a))
      visualization_evaluating_publisher_.publish(*it);
  }
}

void KinematicsReachability::visualizeWorkspaceSamples(const moveit_ros_planning::WorkspacePoints &workspace_in)
{
  moveit_ros_planning::WorkspacePoints workspace = workspace_in;
  
  //visualization_msgs::MarkerArray marker_array;

  std::vector<visualization_msgs::Marker> markers;
  visualization_msgs::Marker marker;
  marker.type = marker.CUBE;
  marker.action = 0;

  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.2;
  
  marker.ns = "samples";
  marker.header = workspace.header;

  marker.pose.position.x = (workspace.parameters.min_corner.x + workspace.parameters.max_corner.x)/2.0;
  marker.pose.position.y = (workspace.parameters.min_corner.y + workspace.parameters.max_corner.y)/2.0;
  marker.pose.position.z = (workspace.parameters.min_corner.z + workspace.parameters.max_corner.z)/2.0;
  marker.pose.orientation.w = 1.0;
  
  marker.scale.x = std::fabs(workspace.parameters.min_corner.x - workspace.parameters.max_corner.x);
  marker.scale.y = std::fabs(workspace.parameters.min_corner.y - workspace.parameters.max_corner.y);
  marker.scale.z = std::fabs(workspace.parameters.min_corner.z - workspace.parameters.max_corner.z);
  marker.id = 3; 
  markers.push_back(marker);
  
  if(workspace.points.empty())
    sampleUniform(workspace);

  std::vector<unsigned int> indices;  
  std::vector<moveit_msgs::MoveItErrorCodes> error_code(1); 
  std::vector<std_msgs::ColorRGBA> colors;
  colors.push_back(evaluating_color_);
  std::vector<unsigned int> marker_ids;
  marker_ids.push_back(1);  

  getMarkers(workspace,"samples",indices,markers);

  /*  std::vector<visualization_msgs::Marker> marker_points = getSphereMarker(workspace,"samples",indices,colors,error_code,marker_ids);      
  marker_array.markers.push_back(marker_points[0]);
  */

  getArrowMarkers(workspace,"samples",indices,markers);
  
  ROS_INFO("Publishing initial set of markers");  
  //visualization_publisher_.publish(marker_array); 
   for(std::vector<visualization_msgs::Marker>::const_iterator it = markers.begin() ; it != markers.end() ; ++it)
  { 
    if((it->color.r == reachable_color_.r) && (it->color.g == reachable_color_.g) && (it->color.b == reachable_color_.b) && (it->color.a == reachable_color_.a))
      visualization_success_publisher_.publish(*it);
    else if((it->color.r == unreachable_color_.r) && (it->color.g == unreachable_color_.g) && (it->color.b == unreachable_color_.b) && (it->color.a == unreachable_color_.a))
      visualization_fail_publisher_.publish(*it);
    else if((it->color.r == evaluating_color_.r) && (it->color.g == evaluating_color_.g) && (it->color.b == evaluating_color_.b) && (it->color.a == evaluating_color_.a))
      visualization_evaluating_publisher_.publish(*it);
  } 

  boundary_publisher_.publish(workspace);  
}

} // namespace
