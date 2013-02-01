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

#ifndef KINEMATICS_REACHABILITY_H_
#define KINEMATICS_REACHABILITY_H_

// System
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <moveit_msgs/GetConstraintAwarePositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_ros_planning/WorkspacePoints.h>
#include <moveit_ros_planning/WorkspacePoint.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/ColorRGBA.h>
#include <moveit_ros_planning/Progress.h>
#include <geometry_msgs/Point.h>

// MoveIt!
#include <moveit/kinematics_constraint_aware/kinematics_constraint_aware.h>
#include <moveit/kinematics_cache/kinematics_cache.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Eigenvalues>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace kinematics_reachability
{
class KinematicsReachability
{
public:

  /** @class
   *  @brief Compute and visualize reachable workspace for an arm
   *  @author Sachin Chitta <sachinc@willowgarage.com>
   *
   */
  KinematicsReachability(const kinematics_constraint_aware::KinematicsConstraintAwarePtr &kinematics_solver,
                         const planning_scene_monitor::PlanningSceneMonitorPtr &planning_scene_monitor);

  virtual ~KinematicsReachability()
  {
  };

  bool initialize();
  
  /**
   * @brief This method computes and returns a discretized workspace for an arm (including the reachable 
   * and unreachable parts of the grid that is explored)
   * @param workspace The user is expected to fill up this message with parameters defining the workspace. The method will 
   * then populate the workspace points
   * The parameters that the user is expected to fill up include
   * workspace.parameters - specify the bounding box for the workspace
   * workspace.orientations - specify a set of orientations to explore 
   * workspace.position_resolution - the resolution (in m) for the grid to explore the workspace
   * workspace.header.frame_id - The base frame of the arm
   * workspace.group_name - The name of the group that you are exploring
   */
  bool computeWorkspace(moveit_ros_planning::WorkspacePoints &workspace,
                        bool visualize = false);


  bool computeWorkspaceFK(moveit_ros_planning::WorkspacePoints &workspace,
                          double timeout);
  

  /**
   * @brief This method computes and returns a set of redundant solutions for a particular pose of the arm
   * @param group_name The name of the group to explore the redundant solution space for
   * @param pose_stamped The pose of the arm for which to explore the redundant workspace
   * @param timeout The time (in seconds) to spend on exploring the redundant space
   */
  
  moveit_ros_planning::WorkspacePoints computeRedundantSolutions(const std::string &group_name,
                                                                     const geometry_msgs::PoseStamped &pose_stamped,
                                                                     double timeout,
                                                                     bool visualize = false);
    
  /**
   * @brief This method computes and returns only the discretized reachable workspace for an arm
   * @param workspace The user is expected to fill up this message with parameters defining the workspace. The method will 
   * then populate the workspace points
   * The parameters that the user is expected to fill up include
   * workspace.parameters - specify the bounding box for the workspace
   * workspace.orientations - specify a set of orientations to explore 
   * workspace.position_resolution - the resolution (in m) for the grid to explore the workspace
   * workspace.header.frame_id - The base frame of the arm
   * workspace.group_name - The name of the group that you are exploring
   */
  bool getOnlyReachableWorkspace(moveit_ros_planning::WorkspacePoints &workspace, 
                                 bool visualize = false);

  /**
   * @brief This method publishes (on a topic) the workspace
   * @param workspace The workspace message to publish
   */
  void publishWorkspace(const moveit_ros_planning::WorkspacePoints &workspace);

  /**
   * @brief This method visualizes the workspace by publishing it to rviz
   * @param workspace The workspace message to visualize
   * @param marker_namespace The namespace in which the markers are visualized
   */
  void visualize(const moveit_ros_planning::WorkspacePoints &workspace,
                 const std::string &marker_namespace);

  /**
   * @brief This method visualizes the workspace by publishing it to rviz
   * @param workspace The workspace message to visualize
   * @param marker_namespace The namespace in which the markers are visualized
   * @param orientation The particular set of orientations to visualize
   */
  void visualize(const moveit_ros_planning::WorkspacePoints &workspace,
                 const std::string &marker_namespace,
                 const std::vector<geometry_msgs::Quaternion> &orientations);

  /**
   * @brief This method visualizes the workspace by publishing it to rviz using a set of arrows
   * @param workspace The workspace message to visualize
   * @param marker_namespace The namespace in which the markers are visualized
   */
  void animateWorkspace(const moveit_ros_planning::WorkspacePoints &workspace);

  /**
   * @brief This method visualizes just the points in the workspace and the boundary of the workspace.
   * @param workspace The workspace message to visualize
   * @param marker_namespace The namespace in which the markers are visualized
   */
  void visualizeWorkspaceSamples(const moveit_ros_planning::WorkspacePoints &workspace);

  void cancelFindIKSolutions(bool canceled)
  {
    canceled_ = canceled;
  }

  /**
   * @brief This method visualizes the workspace by publishing it to rviz using a set of arrows
   * @param workspace The workspace message to visualize
   * @param marker_namespace The namespace in which the markers are visualized
   */
  void visualizeWithArrows(const moveit_ros_planning::WorkspacePoints &workspace,
                           const std::string &marker_namespace);

protected:

  ros::NodeHandle node_handle_;
  tf::Pose tool_offset_, tool_offset_inverse_;
  geometry_msgs::Vector3 arrow_marker_scale_, sphere_marker_scale_;  

private:


  bool getDisplayTrajectory(const moveit_ros_planning::WorkspacePoints &workspace, 
                            moveit_msgs::DisplayTrajectory &display_trajectory);  

  bool getDisplayTrajectory(const moveit_ros_planning::WorkspacePoint &workspace_point,
                            moveit_msgs::DisplayTrajectory &display_trajectory);

  void getMarkers(const moveit_ros_planning::WorkspacePoints &workspace,
                  const std::string &marker_namespace,
                  const std::vector<unsigned int> &points,
                  std::vector<visualization_msgs::Marker> &markers);  
  
  std::vector<visualization_msgs::Marker> getSphereMarker(const moveit_ros_planning::WorkspacePoints &workspace,
                                                          const std::string &marker_namespace,
                                                          const std::vector<unsigned int> &indices,
                                                          const std::vector<std_msgs::ColorRGBA> &color,
                                                          const std::map<int, unsigned int> &error_code_map,
                                                          const std::vector<unsigned int> &marker_id);

  std_msgs::ColorRGBA getMarkerColor(const moveit_ros_planning::WorkspacePoint &workspace_point);
  
  void getArrowMarkers(const moveit_ros_planning::WorkspacePoints &workspace,
                       const std::string &marker_namespace,
                       const std::vector<unsigned int> &points,
                       std::vector<visualization_msgs::Marker> &markers);
  
  void setToolFrameOffset(const geometry_msgs::Pose &pose);

  void findIKSolutions(moveit_ros_planning::WorkspacePoints &workspace, bool visualize = false);

  void findIK(const std::string &group_name,
              const geometry_msgs::PoseStamped &pose_stamped,
              moveit_msgs::MoveItErrorCodes &error_code,
              moveit_msgs::RobotState &robot_state);

  void getDefaultIKRequest(const std::string &group_name,
                           moveit_msgs::GetConstraintAwarePositionIK::Request &req);

  void removeUnreachableWorkspace(moveit_ros_planning::WorkspacePoints &workspace);

  std::vector<unsigned int> getPointsAtOrientation(const moveit_ros_planning::WorkspacePoints &workspace,
                                                   const geometry_msgs::Quaternion &orientation);

  std::vector<unsigned int > getPointsWithinRange(const moveit_ros_planning::WorkspacePoints &workspace,
                                                  const double min_radius,
                                                  const double max_radius);

  void getNumPoints(const moveit_ros_planning::WorkspacePoints &workspace,
                    unsigned int &x_num_points,
                    unsigned int &y_num_points,
                    unsigned int &z_num_points);
  
  bool isEqual(const geometry_msgs::Quaternion &orientation_1, 
               const geometry_msgs::Quaternion &orientation_2);

  ros::Publisher visualization_success_publisher_, visualization_fail_publisher_, visualization_evaluating_publisher_, visualization_manipulability_publisher_, visualization_orientation_success_publisher_, workspace_publisher_, boundary_publisher_, robot_trajectory_publisher_, progress_publisher_;

  void getPositionIndex(const moveit_ros_planning::WorkspacePoints &workspace,
			std::vector<unsigned int> &reachable_workspace,
			std::vector<unsigned int> &unreachable_workspace);

  bool generateCache(const std::string &group_name,
                     double timeout,
                     const kinematics_cache::KinematicsCache::Options &options,
                     const std::string &cache_filename);
  
  bool updateFromCache(moveit_msgs::GetConstraintAwarePositionIK::Request &request);
  
  bool first_time_, use_cache_, canceled_;  
  std::string cache_filename_;  
  double default_cache_timeout_,kinematics_solver_timeout_;
  int max_fk_points_;
  kinematics_cache::KinematicsCachePtr kinematics_cache_;
  kinematics_constraint_aware::KinematicsConstraintAwarePtr kinematics_solver_;
  kinematics_cache::KinematicsCache::Options default_cache_options_;
  std_msgs::ColorRGBA reachable_color_, unreachable_color_, evaluating_color_, default_manipulability_color_, default_orientation_color_;
  
  void initializeColor(const std::string &color_name,
                       std_msgs::ColorRGBA &color_msg,
                       double default_r,
                       double default_g,
                       double default_b);  
  /**
   * @brief This method creates the samples inside the workspace to explore
   * @param workspace The message containing workspace parameters to use for sampling
   */
  bool sampleUniform(moveit_ros_planning::WorkspacePoints &workspace);

  void animateWorkspace(const moveit_ros_planning::WorkspacePoints &workspace,
                        unsigned int index);

  void  getColorFromManipulability(double manipulability_index, double highest);

  bool getManipulabilityIndex(const robot_state::RobotState &kinematic_state,
                              const std::string &group_name,
                              double &manipulability_index) const;

  bool checkState(const robot_state::RobotState &kinematic_state,
                  const std::string &group_name) const;  

  Eigen::MatrixXd getJacobian(const robot_state::RobotState &kinematic_state,
                              const std::string &group_name) const;  

  void getManipulabilityMarkers(const moveit_ros_planning::WorkspacePoints &workspace,
                                visualization_msgs::Marker &marker, double max_manipulability);

  std::map<int,double> manipulability_map_; 

  std::map <std::vector<double>, std::vector<bool> > point_map_; 

  void getOrientationSuccessMarkers(const moveit_ros_planning::WorkspacePoints &workspace, 
                                    visualization_msgs::Marker &marker);

  void  getColorFromSuccessList(std::vector<bool> successes);

  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
};

}
#endif
