/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Ioan Sucan */

#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <moveit/robot_state/transforms.h>
#include <interactive_markers/interactive_marker_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robot_interaction
{

static const float END_EFFECTOR_UNREACHABLE_COLOR[4] = { 1.0, 0.3, 0.3, 1.0 };
static const float END_EFFECTOR_REACHABLE_COLOR[4] = { 0.2, 1.0, 0.2, 1.0 };
static const float END_EFFECTOR_WHITE_COLOR[4] = { 1.0, 1.0, 1.0, 1.0 };
static const float END_EFFECTOR_COLLISION_COLOR[4] = { 0.8, 0.8, 0.0, 1.0 };

const std::string RobotInteraction::INTERACTIVE_MARKER_TOPIC = "robot_interaction_interactive_marker_topic";

RobotInteraction::InteractionHandler::InteractionHandler(const std::string &name,
                                                         const robot_state::RobotState &kstate,
                                                         const boost::shared_ptr<tf::Transformer> &tf) :
  name_(name),
  kstate_(new robot_state::RobotState(kstate)),
  tf_(tf),
  interaction_mode_(POSITION_IK),
  display_meshes_(true),
  display_controls_(true)
{
  setup();
}

RobotInteraction::InteractionHandler::InteractionHandler(const std::string &name,
                                                         const robot_model::RobotModelConstPtr &kmodel,
                                                         const boost::shared_ptr<tf::Transformer> &tf) :
  name_(name),
  kstate_(new robot_state::RobotState(kmodel)),
  tf_(tf),
  interaction_mode_(POSITION_IK),
  display_meshes_(true),
  display_controls_(true)
{
  setup();
}

void RobotInteraction::InteractionHandler::setup()
{
  std::replace(name_.begin(), name_.end(), '_', '-'); // we use _ as a special char in marker name  
  ik_timeout_ = 0.0; // so that the default IK timeout is used in setFromIK()
  ik_attempts_ = 0; // so that the default IK attempts is used in setFromIK()
  velocity_gain_ = 0.1;
  planning_frame_ = kstate_->getRobotModel()->getModelFrame();
}

namespace
{
/* This function converts a transformation given as an Eigen::Affine3d object to a
 * 6-dimensional pose/orientation vector in the convention (t utheta), being t
 * the translation and utheta the angle-axis representation of the rotation
 * TODO: move functions like this into an eigen_utils package
 */
void eigenTransformToEigenVector(const Eigen::Affine3d &M, Eigen::VectorXd &pose)
{
  pose.resize(6);

  //fill translation
  pose.matrix().block(0,0,3,1) = M.matrix().block(0,3,3,1);

  //Compute and fill rotation (theta-u convention)
  //Most of this code is adapted from ViSP vpThetaUVector::build_from(vpRotationMatrix)
  const Eigen::Matrix3d R = M.matrix().block(0, 0, 3, 3);

  double s,c,theta,sinc;
  s = (R(1,0) - R(0,1)) * (R(1,0) - R(0,1))
    + (R(2,0) - R(0,2)) * (R(2,0) - R(0,2))
    + (R(2,1) - R(1,2)) * (R(2,1) - R(1,2));
  s = sqrt(s) / 2.0;
  c = (R(0,0) + R(1,1) + R(2,2) - 1.0) / 2.0;
  theta = atan2(s,c);  /* theta in [0, PI] since s > 0 */

  // General case when theta != pi. If theta=pi, c=-1
  static const double minimum = 0.0001;
  if ( (1 + c) > minimum) // Since -1 <= c <= 1, no fabs(1+c) is required
  {
    static const double threshold = 1.0e-8;
    if (fabs(theta) < threshold) sinc = 1.0 ;
    else  sinc = (s / theta) ;

    pose(3) = (R(2,1) - R(1,2)) / (2*sinc);
    pose(4) = (R(0,2) - R(2,0)) / (2*sinc);
    pose(5) = (R(1,0) - R(0,1)) / (2*sinc);
  }
  else /* theta near PI */
  {
    if ( (R(0,0) - c) < std::numeric_limits<double>::epsilon() )
      pose(3) = 0.;
    else
      pose(3) = theta * (sqrt((R(0,0) - c) / (1 - c)));
    if ((R(2,1) - R(1,2)) < 0) pose(3) = -pose(3);

    if ( (R(1,1) - c) < std::numeric_limits<double>::epsilon() )
      pose(4) = 0.;
    else
      pose(4) = theta * (sqrt((R(1,1) - c) / (1 - c)));

    if ((R(0,2) - R(2,0)) < 0) pose(4) = -pose(4);

    if ( (R(2,2) - c) < std::numeric_limits<double>::epsilon() )
      pose(5) = 0.;
    else
      pose(5) = theta * (sqrt((R(2,2) - c) / (1 - c)));

    if ((R(1,0) - R(0,1)) < 0) pose(5) = -pose(5);
  }
}
}

void RobotInteraction::InteractionHandler::setPoseOffset(const RobotInteraction::EndEffector& eef, const geometry_msgs::Pose& m)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_[eef.eef_group] = m;
}

void RobotInteraction::InteractionHandler::clearPoseOffset(const RobotInteraction::EndEffector& eef)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_.erase(eef.eef_group);
}

void RobotInteraction::InteractionHandler::clearPoseOffsets()
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_.clear();
}

bool RobotInteraction::InteractionHandler::getPoseOffset(const RobotInteraction::EndEffector& eef, geometry_msgs::Pose& m)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  std::map<std::string, geometry_msgs::Pose>::iterator it = offset_map_.find(eef.eef_group);
  if (it != offset_map_.end())
  {
    m = it->second;
    return true;
  }
  return false;
}

bool RobotInteraction::InteractionHandler::getPoseOffset(const RobotInteraction::VirtualJoint& vj, geometry_msgs::Pose& m)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  std::map<std::string, geometry_msgs::Pose>::iterator it = offset_map_.find(vj.joint_name);
  if (it != offset_map_.end())
  {
    m = it->second;
    return true;
  }
  return false;
}

bool RobotInteraction::InteractionHandler::getLastEndEffectorMarkerPose(const RobotInteraction::EndEffector& eef, geometry_msgs::PoseStamped& ps)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it = pose_map_.find(eef.eef_group);
  if (it != pose_map_.end())
  {
    ps = it->second;
    return true;
  }
  return false;
}

bool RobotInteraction::InteractionHandler::getLastVirtualJointMarkerPose(const RobotInteraction::VirtualJoint& vj, geometry_msgs::PoseStamped& ps)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it = pose_map_.find(vj.joint_name);
  if (it != pose_map_.end())
  {
    ps = it->second;
    return true;
  }
  return false;
}

void RobotInteraction::InteractionHandler::clearSavedEndEffectorMarkerPose(const RobotInteraction::EndEffector& eef)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  pose_map_.erase(eef.eef_group);
}

void RobotInteraction::InteractionHandler::clearSavedVirtualJointMarkerPose(const RobotInteraction::VirtualJoint& vj)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  pose_map_.erase(vj.joint_name);
}

void RobotInteraction::InteractionHandler::clearSavedMarkerPoses()
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  pose_map_.clear();
}


robot_state::RobotStateConstPtr RobotInteraction::InteractionHandler::getState() const
{
  boost::unique_lock<boost::mutex> ulock(state_lock_);
  if (kstate_)
    return kstate_;
  else
  {  
    do
    {
      state_available_condition_.wait(ulock);
    } while (!kstate_);
    return kstate_;
  }
}

void RobotInteraction::InteractionHandler::setState(const robot_state::RobotState& kstate)
{
  boost::unique_lock<boost::mutex> ulock(state_lock_);
  if (!kstate_)
  {   
    do
    {
      state_available_condition_.wait(ulock);
    } while (!kstate_);
  }
  if (kstate_.unique())
    *kstate_ = kstate;
  else
    kstate_.reset(new robot_state::RobotState(kstate));
}

robot_state::RobotStatePtr RobotInteraction::InteractionHandler::getUniqueStateAccess()
{
  robot_state::RobotStatePtr result;
  {
    boost::unique_lock<boost::mutex> ulock(state_lock_);
    if (!kstate_)
    {  
      do
      {
        state_available_condition_.wait(ulock);
      } while (!kstate_);
    }  
    result.swap(kstate_);
  }
  if (!result.unique())
    result.reset(new robot_state::RobotState(*result));
  return result;
}

void RobotInteraction::InteractionHandler::setStateToAccess(robot_state::RobotStatePtr &state)
{      
  boost::unique_lock<boost::mutex> ulock(state_lock_);
  if (state != kstate_)
    kstate_.swap(state);
  state_available_condition_.notify_all(); 
}

bool RobotInteraction::InteractionHandler::handleEndEffector(const robot_interaction::RobotInteraction::EndEffector &eef,
                                                             const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{ 
  geometry_msgs::PoseStamped tpose;
  geometry_msgs::Pose offset;
  if(!getPoseOffset(eef, offset))
    offset.orientation.w = 1;
  if (transformFeedbackPose(feedback, offset, tpose))
  {
    pose_map_lock_.lock();
    pose_map_[eef.eef_group] = tpose;
    pose_map_lock_.unlock();
  }
  else
    return false;
  bool update_state_result = false;
  if (interaction_mode_ == POSITION_IK)
  {
    robot_state::RobotStatePtr state = getUniqueStateAccess();
    update_state_result = robot_interaction::RobotInteraction::updateState(*state, eef, tpose.pose, ik_attempts_, ik_timeout_, state_validity_callback_fn_);
    setStateToAccess(state);
  }
  else 
    if (interaction_mode_ == VELOCITY_IK)
    {
      robot_state::RobotStatePtr state = getUniqueStateAccess();
      // compute velocity from current pose to goal pose, in the current end-effector frame
      const Eigen::Affine3d &wMe = state->getLinkState(eef.parent_link)->getGlobalLinkTransform();
      Eigen::Affine3d wMt;
      tf::poseMsgToEigen(tpose.pose, wMt);
      Eigen::Affine3d eMt = wMe.inverse() * wMt;
      Eigen::VectorXd twist(6);
      
      eigenTransformToEigenVector(eMt, twist);
      
      geometry_msgs::Twist twist_msg;
      tf::twistEigenToMsg(twist, twist_msg);
      
      update_state_result = robot_interaction::RobotInteraction::updateState(*state, eef, twist_msg, velocity_gain_, state_validity_callback_fn_);
      setStateToAccess(state);
    }
  
  bool error_state_changed = false;
  if (!update_state_result)
  {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {
      error_state_changed = inError(eef) ? false : true;
      error_state_.insert(eef.parent_group);
    }
  }
  else 
  {
    error_state_changed = inError(eef) ? true : false;
    error_state_.erase(eef.parent_group);
  }

  if (update_callback_)
    update_callback_(this, error_state_changed);

  return error_state_changed;
}

bool RobotInteraction::InteractionHandler::handleVirtualJoint(const robot_interaction::RobotInteraction::VirtualJoint &vj,
                                                              const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  geometry_msgs::PoseStamped tpose;
  geometry_msgs::Pose offset;
  if(!getPoseOffset(vj, offset))
    offset.orientation.w = 1;
  if (transformFeedbackPose(feedback, offset, tpose))
  {
    pose_map_lock_.lock();
    pose_map_[vj.joint_name] = tpose;
    pose_map_lock_.unlock();
  }
  else
    return false;
  
  robot_state::RobotStatePtr state = getUniqueStateAccess();
  robot_interaction::RobotInteraction::updateState(*state, vj, tpose.pose);
  setStateToAccess(state);
  
  if (update_callback_)
    update_callback_(this, false);
  
  return false;
}

bool RobotInteraction::InteractionHandler::inError(const robot_interaction::RobotInteraction::EndEffector& eef) const
{
  return error_state_.find(eef.parent_group) != error_state_.end();
}

bool RobotInteraction::InteractionHandler::inError(const robot_interaction::RobotInteraction::VirtualJoint& vj) const
{
  return false;
}

bool RobotInteraction::InteractionHandler::transformFeedbackPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                                                                 const geometry_msgs::Pose &offset,
                                                                 geometry_msgs::PoseStamped &tpose)
{
  tpose.header = feedback->header;
  tpose.pose = feedback->pose;
  if (feedback->header.frame_id != planning_frame_)
  {
    if (tf_)
      try
      {
        tf::Stamped<tf::Pose> spose;
        tf::poseStampedMsgToTF(tpose, spose);
        // Express feedback (marker) pose in planning frame
        tf_->transformPose(planning_frame_, spose, spose);
        // Apply inverse of offset to bring feedback pose back into the end-effector support link frame
        tf::Transform tf_offset;
        tf::poseMsgToTF(offset, tf_offset);
        spose.setData(spose * tf_offset.inverse());
        tf::poseStampedTFToMsg(spose, tpose);
      }
      catch (tf::TransformException& e)
      {
        ROS_ERROR("Error transforming from frame '%s' to frame '%s'", tpose.header.frame_id.c_str(), planning_frame_.c_str());
        return false;
      }
    else
    {   
      ROS_ERROR("Cannot transform from frame '%s' to frame '%s' (no TF instance provided)", tpose.header.frame_id.c_str(), planning_frame_.c_str());
      return false;
    }
  }
  return true;
}

RobotInteraction::RobotInteraction(const robot_model::RobotModelConstPtr &kmodel, const std::string &ns) :
  kmodel_(kmodel)
{  
  int_marker_server_ = new interactive_markers::InteractiveMarkerServer(ns.empty() ? INTERACTIVE_MARKER_TOPIC : ns + "/" + INTERACTIVE_MARKER_TOPIC);

  // spin a thread that will process feedback events
  run_processing_thread_ = true;
  processing_thread_.reset(new boost::thread(boost::bind(&RobotInteraction::processingThread, this)));
}

RobotInteraction::~RobotInteraction()
{
  run_processing_thread_ = false;
  new_feedback_condition_.notify_all();
  processing_thread_->join();

  clear();
  delete int_marker_server_;
}

void RobotInteraction::decideActiveComponents(const std::string &group)
{
  decideActiveEndEffectors(group);
  decideActiveVirtualJoints(group);
}

double RobotInteraction::computeGroupScale(const std::string &group)
{
  static const double DEFAULT_SCALE = 0.2;
  const robot_model::JointModelGroup *jmg = kmodel_->getJointModelGroup(group);
  if (!jmg)
    return 0.0;
  
  const std::vector<std::string> &links = jmg->getLinkModelNames();
  if (links.empty())
    return DEFAULT_SCALE;
  
  std::vector<double> scale(3, 0.0);
  std::vector<double> low(3, std::numeric_limits<double>::infinity());
  std::vector<double> hi(3, -std::numeric_limits<double>::infinity());
  robot_state::RobotState default_state(kmodel_);
  default_state.setToDefaultValues();
  
  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    robot_state::LinkState *ls = default_state.getLinkState(links[i]);
    if (!ls)
      continue;
    const Eigen::Vector3d &ext = ls->getLinkModel()->getShapeExtentsAtOrigin();
    
    Eigen::Vector3d corner1 = ext/2.0;
    corner1 = ls->getGlobalLinkTransform() * corner1;
    Eigen::Vector3d corner2 = ext/-2.0;
    corner2 = ls->getGlobalLinkTransform() * corner2;
    for (int k = 0 ; k < 3 ; ++k)
    {
      if (low[k] > corner2[k])
        low[k] = corner2[k];
      if (hi[k] < corner1[k])
        hi[k] = corner1[k];
    }
  }
  
  for (int i = 0 ; i < 3 ; ++i)
    scale[i] = hi[i] - low[i];

  static const double sqrt_3 = 1.732050808;
  double s = std::max(std::max(scale[0], scale[1]), scale[2]) * sqrt_3;
  
  // if the scale is less than 1mm, set it to default
  if (s < 1e-3)
    s = DEFAULT_SCALE;  
  return s;
}

void RobotInteraction::decideActiveVirtualJoints(const std::string &group)
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  active_vj_.clear();

  ROS_DEBUG_NAMED("robot_interaction", "Deciding active virtual joints for group '%s'", group.c_str());
  
  if (group.empty())
    return;
  
  const boost::shared_ptr<const srdf::Model> &srdf = kmodel_->getSRDF();
  const robot_model::JointModelGroup *jmg = kmodel_->getJointModelGroup(group);
  
  if (!jmg || !srdf)
    return;
  
  if (!jmg->hasJointModel(kmodel_->getRootJointName()))
    return;

  robot_state::RobotState default_state(kmodel_);
  default_state.setToDefaultValues();
  std::vector<double> aabb;
  default_state.computeAABB(aabb);
  
  const std::vector<srdf::Model::VirtualJoint> &vj = srdf->getVirtualJoints();
  for (std::size_t i = 0 ; i < vj.size() ; ++i)
    if (vj[i].name_ == kmodel_->getRootJointName())
    {
      if (vj[i].type_ == "planar" || vj[i].type_ == "floating")
      {
        VirtualJoint v;
        v.connecting_link = vj[i].child_link_;
        v.joint_name = vj[i].name_;
        if (vj[i].type_ == "planar")
          v.dof = 3;
        else
          v.dof = 6;
        // take the max of the X extent and the Y extent
        v.size = std::max(aabb[1] - aabb[0], aabb[3] - aabb[2]);
        active_vj_.push_back(v);
      }
    }
}

void RobotInteraction::decideActiveEndEffectors(const std::string &group)
{ 
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);

  active_eef_.clear();

  ROS_DEBUG_NAMED("robot_interaction", "Deciding active end-effectors for group '%s'", group.c_str());
  
  if (group.empty())
    return;
  
  const boost::shared_ptr<const srdf::Model> &srdf = kmodel_->getSRDF();
  const robot_model::JointModelGroup *jmg = kmodel_->getJointModelGroup(group);
  
  if (!jmg || !srdf)
    return;
  
  const std::vector<srdf::Model::EndEffector> &eef = srdf->getEndEffectors();
  const std::pair<robot_model::SolverAllocatorFn, robot_model::SolverAllocatorMapFn> &smap = jmg->getSolverAllocators();
  
  // if we have an IK solver for the selected group, we check if there are any end effectors attached to this group
  if (smap.first)
  {
    for (std::size_t i = 0 ; i < eef.size() ; ++i)
      if ((jmg->hasLinkModel(eef[i].parent_link_) || jmg->getName() == eef[i].parent_group_) && jmg->canSetStateFromIK(eef[i].parent_link_))
      {
        // we found an end-effector for the selected group
        EndEffector ee;
        ee.parent_group = group;
        ee.parent_link = eef[i].parent_link_;
        ee.eef_group = eef[i].component_group_;
        active_eef_.push_back(ee);
        break;
      }
  }
  else
    if (!smap.second.empty())
    {
      for (std::map<const robot_model::JointModelGroup*, robot_model::SolverAllocatorFn>::const_iterator it = smap.second.begin() ; 
           it != smap.second.end() ; ++it)
      {
        for (std::size_t i = 0 ; i < eef.size() ; ++i)
          if ((it->first->hasLinkModel(eef[i].parent_link_) || jmg->getName() == eef[i].parent_group_) && it->first->canSetStateFromIK(eef[i].parent_link_))
          {
            // we found an end-effector for the selected group;
            EndEffector ee;
            ee.parent_group = it->first->getName();
            ee.parent_link = eef[i].parent_link_;
            ee.eef_group = eef[i].component_group_;
            active_eef_.push_back(ee);
            break;
          }
      }
    }
  for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
  {
    active_eef_[i].size = computeGroupScale(active_eef_[i].eef_group);
    ROS_DEBUG_NAMED("robot_interaction", "Found active end-effector '%s', of scale %lf", active_eef_[i].eef_group.c_str(), active_eef_[i].size);
  }
}

void RobotInteraction::clear()
{  
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  active_eef_.clear();
  active_vj_.clear();
  clearInteractiveMarkersUnsafe();
  publishInteractiveMarkers();
}

void RobotInteraction::clearInteractiveMarkers()
{  
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  clearInteractiveMarkersUnsafe();
}

void RobotInteraction::clearInteractiveMarkersUnsafe()
{ 
  handlers_.clear();
  shown_markers_.clear();
  int_marker_server_->clear();
}

void RobotInteraction::addEndEffectorMarkers(const InteractionHandlerPtr &handler, const RobotInteraction::EndEffector& eef,
                                             visualization_msgs::InteractiveMarker& im)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  addEndEffectorMarkers(handler, eef, pose, im);
}

void RobotInteraction::addEndEffectorMarkers(const InteractionHandlerPtr &handler, const RobotInteraction::EndEffector& eef,
                                             const geometry_msgs::Pose& im_to_eef,
                                             visualization_msgs::InteractiveMarker& im)
{
  visualization_msgs::InteractiveMarkerControl m_control;
  m_control.always_visible = false;
  m_control.interaction_mode = m_control.MOVE_ROTATE;

  std_msgs::ColorRGBA marker_color;
  const float *color = handler->inError(eef) ? END_EFFECTOR_UNREACHABLE_COLOR : END_EFFECTOR_REACHABLE_COLOR;
  marker_color.r = color[0];
  marker_color.g = color[1];
  marker_color.b = color[2];
  marker_color.a = color[3];

  robot_state::RobotStateConstPtr kinematic_state = handler->getState();
  const std::vector<std::string> &link_names = kinematic_state->getJointStateGroup(eef.eef_group)->getJointModelGroup()->getLinkModelNames();
  visualization_msgs::MarkerArray marker_array;
  kinematic_state->getRobotMarkers(marker_array, link_names, marker_color, eef.eef_group, ros::Duration());
  tf::Pose tf_root_to_link;
  try{
    tf::poseEigenToTF(kinematic_state->getLinkState(eef.parent_link)->getGlobalLinkTransform(), tf_root_to_link);
  }
  catch(...)
  {
    ROS_ERROR("Didn't find link state for [%s]", eef.parent_link.c_str());
  }
  // Release the ptr count on the kinematic state
  kinematic_state.reset();

  for (std::size_t i = 0 ; i < marker_array.markers.size() ; ++i)
  {
    marker_array.markers[i].header = im.header;
    marker_array.markers[i].mesh_use_embedded_materials = true;
    // - - - - - - Do some math for the offset - - - - - -
    tf::Pose tf_root_to_im, tf_root_to_mesh, tf_im_to_eef;
    tf::poseMsgToTF(im.pose, tf_root_to_im);
    tf::poseMsgToTF(marker_array.markers[i].pose, tf_root_to_mesh);
    tf::poseMsgToTF(im_to_eef, tf_im_to_eef);
    tf::Pose tf_eef_to_mesh = tf_root_to_link.inverse() * tf_root_to_mesh;
    tf::Pose tf_im_to_mesh = tf_im_to_eef * tf_eef_to_mesh;
    tf::Pose tf_root_to_mesh_new = tf_root_to_im * tf_im_to_mesh;
    tf::poseTFToMsg(tf_root_to_mesh_new, marker_array.markers[i].pose);
    // - - - - - - - - - - - - - - - - - - - - - - - - - -
    m_control.markers.push_back(marker_array.markers[i]);
  }
  
  im.controls.push_back(m_control);
}

void RobotInteraction::addInteractiveMarkers(const InteractionHandlerPtr &handler, double marker_scale)
{ 
  // If scale is left at default size of 0, scale will be based on end effector link size. a good value is between 0-1
  
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  
  for (std::size_t i = 0 ; i < active_eef_.size() ; ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = kmodel_->getModelFrame();
    pose.header.stamp = ros::Time::now();
    robot_state::RobotStateConstPtr s = handler->getState(); 
    const robot_state::LinkState *ls = s->getLinkState(active_eef_[i].parent_link);
    // Need to allow for control pose offsets
    tf::Transform tf_root_to_link, tf_root_to_control;
    tf::poseEigenToTF(ls->getGlobalLinkTransform(), tf_root_to_link);
    s.reset(); // to avoid spurious copies of states
    geometry_msgs::Pose msg_link_to_control;
    geometry_msgs::Pose msg_control_to_eef_mesh;
    if(handler->getPoseOffset(active_eef_[i], msg_link_to_control))
    {
      tf::Transform tf_link_to_control;
      tf::poseMsgToTF(msg_link_to_control, tf_link_to_control);

      tf_root_to_control = tf_root_to_link * tf_link_to_control;
      tf::poseTFToMsg(tf_link_to_control.inverse(), msg_control_to_eef_mesh);
    }
    else
    {
      tf_root_to_control = tf_root_to_link;
      msg_control_to_eef_mesh.orientation.w = 1;
    }
    tf::poseTFToMsg(tf_root_to_control, pose.pose);

    std::string marker_name = "EE:" + handler->getName() + "_" + active_eef_[i].parent_link;
    shown_markers_[marker_name] = i;
    
    // Determine interactive maker size
    if (marker_scale < std::numeric_limits<double>::epsilon())
      marker_scale = active_eef_[i].size;
    
    visualization_msgs::InteractiveMarker im = makeEmptyInteractiveMarker(marker_name, pose, marker_scale);
    if(handler && handler->getControlsVisible())
      add6DOFControl(im, false);
    if (handler && handler->getMeshesVisible())
      addEndEffectorMarkers(handler, active_eef_[i], msg_control_to_eef_mesh, im);
    else
      if (handler && handler->inError(active_eef_[i]))
        addErrorMarker(im);
    int_marker_server_->insert(im);
    int_marker_server_->setCallback(im.name, boost::bind(&RobotInteraction::processInteractiveMarkerFeedback, this, _1));
    ROS_DEBUG_NAMED("robot_interaction", "Publishing interactive marker %s (size = %lf)", marker_name.c_str(), marker_scale);
  }
  
  for (std::size_t i = 0 ; i < active_vj_.size() ; ++i)
    if (active_vj_[i].dof == 3)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = kmodel_->getModelFrame();
      pose.header.stamp = ros::Time::now();
      robot_state::RobotStateConstPtr s = handler->getState(); 
      const robot_state::LinkState *ls = s->getLinkState(active_vj_[i].connecting_link);
      tf::poseEigenToMsg(ls->getGlobalLinkTransform(), pose.pose);
      s.reset(); // to avoid spurious copies of states     
      std::string marker_name = "VJ:" + handler->getName() + "_" + active_vj_[i].connecting_link;
      shown_markers_[marker_name] = i;
      visualization_msgs::InteractiveMarker im = makeEmptyInteractiveMarker(marker_name, pose, active_vj_[i].size);
      if(handler && handler->getControlsVisible())
        add3DOFControl(im, false);
      int_marker_server_->insert(im);
      int_marker_server_->setCallback(im.name, boost::bind(&RobotInteraction::processInteractiveMarkerFeedback, this, _1));
      ROS_DEBUG_NAMED("robot_interaction", "Publishing interactive marker %s (size = %lf)", marker_name.c_str(), active_vj_[i].size);
    }
  handlers_[handler->getName()] = handler;
}

void RobotInteraction::publishInteractiveMarkers()
{
  int_marker_server_->applyChanges();
}

bool RobotInteraction::updateState(robot_state::RobotState &state, const VirtualJoint &vj, const geometry_msgs::Pose &pose)
{
  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(pose.orientation, q);
  std::map<std::string, double> vals;
  vals[vj.joint_name + "/x"] = pose.position.x;
  vals[vj.joint_name + "/y"] = pose.position.y;
  Eigen::Vector3d xyz = q.matrix().eulerAngles(0, 1, 2);
  vals[vj.joint_name + "/theta"] = xyz[2];
  state.getJointState(vj.joint_name)->setVariableValues(vals);
  state.updateLinkTransforms();
  return true;
}

bool RobotInteraction::updateState(robot_state::RobotState &state, const EndEffector &eef, const geometry_msgs::Pose &pose,
                                   unsigned int attempts, double ik_timeout, const robot_state::StateValidityCallbackFn &validity_callback)
{ 
  return state.getJointStateGroup(eef.parent_group)->setFromIK(pose, eef.parent_link, attempts, ik_timeout, validity_callback);
}

bool RobotInteraction::updateState(robot_state::RobotState &state, const EndEffector &eef, const geometry_msgs::Twist &twist, double gain,
                                   const robot_state::StateValidityCallbackFn &validity_callback,
                                   const robot_state::SecondaryTaskFn &st_callback)
{
  return state.getJointStateGroup(eef.parent_group)->setFromDiffIK(twist, eef.parent_link, gain, validity_callback, st_callback);
}

void RobotInteraction::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // perform some validity checks
  
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(feedback->marker_name);
  if (it == shown_markers_.end())
  {
    ROS_ERROR("Unknown marker name: '%s' (not published by RobotInteraction class)", feedback->marker_name.c_str());
    return;
  }
  
  std::size_t u = feedback->marker_name.find_first_of("_");
  if (u == std::string::npos || u < 4)
  {
    ROS_ERROR("Invalid marker name: '%s'",  feedback->marker_name.c_str());
    return;
  }
  
  feedback_map_[feedback->marker_name] = feedback;
  new_feedback_condition_.notify_all();
}

void RobotInteraction::processingThread()
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  
  while (run_processing_thread_ && ros::ok())
  {
    while (feedback_map_.empty() && run_processing_thread_ && ros::ok())
      new_feedback_condition_.wait(ulock);

    while (!feedback_map_.empty() && ros::ok())
    {
      visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback = feedback_map_.begin()->second;
      feedback_map_.erase(feedback_map_.begin());
      ROS_DEBUG_NAMED("robot_interaction", "Processing feedback from map for marker [%s]", feedback->marker_name.c_str());

      std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(feedback->marker_name);
      if (it == shown_markers_.end())
      {
        ROS_ERROR("Unknown marker name: '%s' (not published by RobotInteraction class) (should never have ended up in the feedback_map!)", feedback->marker_name.c_str());
        continue;
      }
      std::size_t u = feedback->marker_name.find_first_of("_");
      if (u == std::string::npos || u < 4)
      {
        ROS_ERROR("Invalid marker name: '%s' (should never have ended up in the feedback_map!)",  feedback->marker_name.c_str());
        continue;
      }
      std::string marker_class = feedback->marker_name.substr(0, 2);
      std::string handler_name = feedback->marker_name.substr(3, u - 3); // skip the ":"
      std::map<std::string, InteractionHandlerPtr>::const_iterator jt = handlers_.find(handler_name);
      if (jt == handlers_.end())
      {
        ROS_ERROR("Interactive Marker Handler '%s' is not known.", handler_name.c_str());
        continue;
      }

      // we put this in a try-catch because user specified callbacks may be triggered
      try
      {
        if (marker_class == "EE")
        {
          // make a copy of the data, so we do not lose it while we are unlocked
          EndEffector eef = active_eef_[it->second];
          InteractionHandlerPtr ih = jt->second;
          marker_access_lock_.unlock();
          try
          {
            ih->handleEndEffector(eef, feedback);
          }
          catch(std::runtime_error &ex)
          { 
            ROS_ERROR("Exception caught while handling end-effector update: %s", ex.what());
          }
          catch(...)
          {
            ROS_ERROR("Exception caught while handling end-effector update");
          }          
          marker_access_lock_.lock();
        }
        else
          if (marker_class == "VJ")
          {
            // make a copy of the data, so we do not lose it while we are unlocked
            VirtualJoint vj = active_vj_[it->second];  
            InteractionHandlerPtr ih = jt->second;
            marker_access_lock_.unlock();
            try
            {
              jt->second->handleVirtualJoint(vj, feedback);
            }
            catch(std::runtime_error &ex)
            { 
              ROS_ERROR("Exception caught while handling virtual joint update: %s", ex.what());
            }
            catch(...)
            {
              ROS_ERROR("Exception caught while handling virtual joint update");
            }      
            marker_access_lock_.lock();
          }
          else
            ROS_ERROR("Unknown marker class ('%s') for marker '%s'", marker_class.c_str(), feedback->marker_name.c_str());
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("Exception caught while processing event: %s", ex.what());
      }
      catch (...)
      {
        ROS_ERROR("Exception caught while processing event");
      }
    }
  }
}

}
