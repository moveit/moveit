/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*********************************************************************/

/* Author: Ioan Sucan, E. Gil Jones */

#ifndef MOVEIT_ROBOT_STATE_ROBOT_STATE_
#define MOVEIT_ROBOT_STATE_ROBOT_STATE_

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_state/attached_body.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

/** \brief This namespace includes the classes in the robot_state library. */
namespace robot_state
{

typedef boost::function<void(AttachedBody *body, bool attached)> AttachedBodyCallback;

/** @brief Definition of a kinematic state - the parts of the robot
 *   state which can change. Const members are thread safe */
class RobotState
{
  friend class LinkState;
  friend class JointState;
public:

  /// \cond IGNORE
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \endcond

  /** \brief Create a state corresponding to a given kinematic model */
  RobotState(const robot_model::RobotModelConstPtr &kinematic_model);

  /** \brief Copy constructor */
  RobotState(const RobotState& state);

  ~RobotState();

  /** @brief Set the joint state values from a vector of values.
   *  Assumes that the order of the values matches the order of the joints in the state.
   *  Should only be used for fast setting of joint values. */
  bool setStateValues(const std::vector<double>& joint_state_values);

  /** @brief Set the joint state values from a  map of values (matching string ids to actual joint values) */
  void setStateValues(const std::map<std::string, double>& joint_state_map);

  /** @brief Set the joint state values from a  map of values (matching string ids to actual joint values).
   *  Also returns the set of joint names for which joint states have not been provided.*/
  void setStateValues(const std::map<std::string, double>& joint_state_map, std::vector<std::string>& missing);

  /** @brief Set the joint state values from a joint state message */
  void setStateValues(const sensor_msgs::JointState& msg);

  /** @brief Set the joint state values for an array of variable names, given the values are specified in the same order as the names.
      This is just a convenience call equivalent to passing a map from string to double. */
  void setStateValues(const std::vector<std::string>& joint_names,
                      const std::vector<double>& joint_values);

  /** @brief Get the joint state values. The order in which the values are specified matches the order
   *  of the joints in the RobotModel corresponding to this state.*/
  void getStateValues(std::vector<double>& joint_state_values) const;

  /** @brief Get the joint state values as a map between joint state names and values*/
  void getStateValues(std::map<std::string, double>& joint_state_values) const;

  /** @brief Get the joint state values in a sensor_msgs::JointState msg */
  void getStateValues(sensor_msgs::JointState& msg) const;

  /** \brief Perform forward kinematics with the current values and update the link transforms.*/
  void updateLinkTransforms();

  /** \brief Update the state after setting a particular link to the input global transform pose.*/
  bool updateStateWithLinkAt(const std::string& link_name, const Eigen::Affine3d& transform, bool backward = false);

  /** \brief Get the kinematic model corresponding to this state.*/
  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return kinematic_model_;
  }

  /** \brief Get the number of (active) DOFs in the model corresponding to this state.*/
  unsigned int getVariableCount() const
  {
    return kinematic_model_->getVariableCount();
  }

  /** \brief Set all joints to their default values*/
  void setToDefaultValues();

  /** \brief Sample a random state in accordance with the type of joints employed */
  void setToRandomValues();

  /** \brief Check if a particular set of joints satisifes its bounds.*/
  bool satisfiesBounds(const std::vector<std::string>& joints) const;

  /** \brief Check if a joint satisfies its bounds.*/
  bool satisfiesBounds(const std::string& joint) const;

  /** \brief Check if all joints satisfy their bounds.*/
  bool satisfiesBounds() const;

  /** \brief Make sure all state variables are within bounds and normalized.*/
  void enforceBounds();

  /** \brief Get a group by its name */
  const JointStateGroup* getJointStateGroup(const std::string &name) const;

  /** \brief Get a group by its name */
  JointStateGroup* getJointStateGroup(const std::string &name);

  /** \brief Check if a group exists */
  bool hasJointStateGroup(const std::string &name) const;

  /** \brief Check if a joint is part of this state */
  bool hasJointState(const std::string &joint) const;

  /** \brief Check if a link is updated by this state */
  bool hasLinkState(const std::string &link) const;

  /** \brief Get a joint state by its name */
  JointState* getJointState(const std::string &joint) const;

  JointState* getJointState(const robot_model::JointModel *jmodel) const
  {
    // \todo make this more efficient (store index of JointState in the model)
    return getJointState(jmodel->getName());
  }

  /** \brief Get a link state by its name */
  LinkState* getLinkState(const std::string &link) const;

  /** \brief Get a vector of joint state corresponding to this kinematic state */
  const std::vector<JointState*>& getJointStateVector() const
  {
    return joint_state_vector_;
  }

  /** \brief Get all the maintained link states, in the same order as the link models maintained by the kinematic model */
  const std::vector<LinkState*>& getLinkStateVector() const
  {
    return link_state_vector_;
  }

  /** \brief Get a map that returns JointStateGroups corresponding to names*/
  const std::map<std::string, JointStateGroup*>& getJointStateGroupMap() const
  {
    return joint_state_group_map_;
  }

  /** \brief Get the names of all joint groups in the model corresponding to this state*/
  void getJointStateGroupNames(std::vector<std::string>& names) const;

  /** \brief Attach a body to this state */
  void attachBody(AttachedBody *attached_body);

  /**
     @brief Attach a body to a link
     @param id The string id associated with the attached body
     @param shapes The shapes that make up the attached body
     @param attach_trans The desired transform between this link and the attached body
     @param touch_links The set of links that the attached body is allowed to touch
     @param link_name The link to attach to
  */
  void attachBody(const std::string &id,
                  const std::vector<shapes::ShapeConstPtr> &shapes,
                  const EigenSTL::vector_Affine3d &attach_trans,
                  const std::set<std::string> &touch_links,
                  const std::string &link_name,
                  const sensor_msgs::JointState &detach_posture = sensor_msgs::JointState());
  void attachBody(const std::string &id,
                  const std::vector<shapes::ShapeConstPtr> &shapes,
                  const EigenSTL::vector_Affine3d &attach_trans,
                  const std::vector<std::string> &touch_links,
                  const std::string &link_name,
                  const sensor_msgs::JointState &detach_posture = sensor_msgs::JointState());

  /** \brief Get all bodies attached to the model corresponding to this state */
  void getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const;

  /** \brief Remove the attached body named \e id. Return false if the object was not found (and thus not removed). Return true on success. */
  bool clearAttachedBody(const std::string &id);

  /** \brief Clear the bodies attached to a specific link */
  void clearAttachedBodies(const std::string &link_name);

  /** \brief Clear all attached bodies */
  void clearAttachedBodies();

  /** \brief Get the attached body named \e name. Return NULL if not found. */
  const AttachedBody* getAttachedBody(const std::string &name) const;

  /** \brief Check if an attached body named \e id exists in this state */
  bool hasAttachedBody(const std::string &id) const;

  /** \brief Get the transform corresponding to the frame \e id. This will be known if \e id is a link name or an attached body id.
      Return identity when no transform is available. Use knowsFrameTransform() to test if this function will be successful or not. */
  const Eigen::Affine3d& getFrameTransform(const std::string &id) const;

  /** \brief Compute the axis-aligned bounding box for this particular robot state. \e aabb will have 6 values: xmin, xmax, ymin, ymax, zmin, zmax */
  void computeAABB(std::vector<double> &aabb) const;

  /** \brief Check if a transform to the frame \e id is known. This will be known if \e id is a link name or an attached body id */
  bool knowsFrameTransform(const std::string &id) const;

  /** \brief Print information about the constructed model */
  void printStateInfo(std::ostream &out = std::cout) const;

  /** \brief Returns a string showing the joint and link state tree with transforms */
  std::string getStateTreeString(const std::string& prefix = "") const;

  /** \brief Print the pose of every link */
  void printTransforms(std::ostream &out = std::cout) const;

  /** \brief Get the global transform applied to the entire tree of links */
  const Eigen::Affine3d& getRootTransform() const;

  /** \brief Set the global transform applied to the entire tree of links */
  void setRootTransform(const Eigen::Affine3d &transform);

  /** \brief Return the instance of a random number generator */
  random_numbers::RandomNumberGenerator& getRandomNumberGenerator();

  /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
   *  @param arr The returned marker array
   *  @param link_names The list of link names for which the markers should be created.
   *  @param color The color for the marker
   *  @param ns The namespace for the markers
   *  @param dur The ros::Duration for which the markers should stay visible
   */
  void getRobotMarkers(visualization_msgs::MarkerArray& arr,
                       const std::vector<std::string> &link_names,
                       const std_msgs::ColorRGBA& color,
                       const std::string& ns,
                       const ros::Duration& dur,
                       bool include_attached = false) const;

  /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
   *  @param arr The returned marker array
   *  @param link_names The list of link names for which the markers should be created.
   */
  void getRobotMarkers(visualization_msgs::MarkerArray& arr,
                       const std::vector<std::string> &link_names,
                       bool include_attached = false) const;

  /** \brief Interpolate between two states */
  void interpolate(const RobotState &to, const double t, RobotState &dest) const;

  /** \brief Get the infinity norm distance between two kinematic states */
  double infinityNormDistance(const robot_state::RobotState *other) const;

  /** \brief Get the distance between this state and another one. This distance does not consider topology -- it is only the L2 norm on the joint vector */
  double distance(const RobotState &state) const;

  /** \brief Assignment operator. Copies everything, including attached bodies (clones them) */
  RobotState& operator=(const RobotState &other);

  void setAttachedBodyUpdateCallback(const AttachedBodyCallback &callback);

private:

  void buildState();
  void copyFrom(const RobotState &ks);
  void printTransform(const std::string &st, const Eigen::Affine3d &t, std::ostream &out = std::cout) const;
  void getStateTreeJointString(std::stringstream& ss, const robot_state::JointState* js, const std::string& prefix, bool last) const;
  static void getPoseString(std::stringstream& ss, const Eigen::Affine3d& mtx, const std::string& pfx = "");


  robot_model::RobotModelConstPtr kinematic_model_;

  std::vector<JointState*>                joint_state_vector_;
  std::map<std::string, JointState*>      joint_state_map_;

  /** \brief The states for all the links in the robot */
  std::vector<LinkState*>                 link_state_vector_;

  /** \brief A map from link names to their corresponding states */
  std::map<std::string, LinkState*>       link_state_map_;

  /** \brief Additional transform to be applied to the tree of links */
  Eigen::Affine3d                         root_transform_;

  /** \brief A map from group names to instances of the group state */
  std::map<std::string, JointStateGroup*> joint_state_group_map_;

  /** \brief The attached bodies that are part of this state (from all links) */
  std::map<std::string, AttachedBody*>    attached_body_map_;

  /** \brief For certain operations a state needs a random number generator. However, it may be slightly expensive
      to allocate the random number generator if many state instances are generated. For this reason, the generator
      is allocated on a need basis, by the getRandomNumberGenerator() function. Never use the rng_ member directly, but call
      getRandomNumberGenerator() instead. */
  boost::scoped_ptr<random_numbers::RandomNumberGenerator> rng_;

  /** \brief This event is called when there is a change in the attached bodies for this state;
      The event specifies the body that changed and whether it was just attached or about to be detached. */
  AttachedBodyCallback attached_body_update_callback_;
};

typedef boost::shared_ptr<RobotState> RobotStatePtr;
typedef boost::shared_ptr<const RobotState> RobotStateConstPtr;

}

#endif
