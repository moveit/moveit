/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef MOVEIT_PLANNING_SCENE_PLANNING_SCENE_
#define MOVEIT_PLANNING_SCENE_PLANNING_SCENE_

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <collision_detection/collision_world.h>
#include <kinematic_constraints/kinematic_constraint.h>
#include <kinematics_base/kinematics_base.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <octomap_msgs/OctomapBinary.h>
#include <boost/enable_shared_from_this.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/concept_check.hpp>

/** \brief This namespace includes the central class for representing planning contexts */
namespace planning_scene
{

class PlanningScene;
typedef boost::shared_ptr<PlanningScene> PlanningScenePtr;
typedef boost::shared_ptr<const PlanningScene> PlanningSceneConstPtr;

/** \brief This is the function signature for additional feasibility checks to be imposed on states (in addition to respecting constraints and collision avoidance).
    The first argument is the state to check the feasibility for, the second one is whether the check should be verbose or not. */
typedef boost::function<bool(const planning_models::KinematicState&, bool)> StateFeasibilityFn;

/** \brief This is the function signature for additional feasibility checks to be imposed on motions segments between states (in addition to respecting constraints and collision avoidance). 
    The order of the arguments matters: the notion of feasibility is to be checked for motion segments that start at the first state and end at the second state. The third argument indicates
    whether the check should be verbose or not. */
typedef boost::function<bool(const planning_models::KinematicState&, const planning_models::KinematicState&, bool)> MotionFeasibilityFn;

/** \brief A map from object names (e.g., attached bodies, collision objects) to their colors */
typedef std::map<std::string, std_msgs::ColorRGBA> ColorMap;

/** \brief This class maintains the representation of the
    environment as seen by a planning instance. The environment
    geometry, the robot geometry and state are maintained. */
class PlanningScene : private boost::noncopyable,
                      public boost::enable_shared_from_this<PlanningScene>
{
public:
  
  /** \brief Constructor. Allocate an empty planning scene. Before use, this instance needs to be configured
      by calling the configure() function. */
  PlanningScene(void);
  
protected:

  /** \brief Constructor. Allocate a planning scene that is to be maintained as a diff to a \e parent planning scene.
      This representation does not actually copy the parent scene data. Instead, it maintains diffs with respect to the
      specified parent. When data in the parent is modified, the changes are visible in the diff class as well, unless
      the modified objects were previously modified within the diff class (and a modified copy is stored instead).
      It is recommended that the \e parent planning scene is configured before use. Otherwise,
      the configure() function will have to be called on the diff class as well. */
  PlanningScene(const PlanningSceneConstPtr &parent);
  
public:
  
  virtual ~PlanningScene(void)
  {
  }

  /** \brief Get the name of the planning scene. This is empty by default */
  const std::string& getName(void) const
  {
    return name_;
  }
	
  /** \brief Set the name of the planning scene */
  void setName(const std::string &name)
  {
    name_ = name;
  }
  
  /** \brief Set the types that satisfy the interfaces for collision_space::CollisionWorld and collision_space::CollisionRobot that should be used for collision checking. */
  template<typename CollisionWorldType, typename CollisionRobotType>
  void setCollisionDetectionTypes(void)
  {
    collision_detection_allocator_.reset(new CollisionDetectionAlloc<CollisionWorldType, CollisionRobotType>());
  }

  /** \brief Configure this planning scene to use a particular robot model and semantic description of that robot model.
      The information passed in for this function allows the construction of a kinematic model and of all the classed that
      depend on the kinematic model (e.g., collision world/robot classes) */
  bool configure(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                 const boost::shared_ptr<const srdf::Model> &srdf_model,
                 const std::string &root_link = "");

  /** \brief Configure this planning scene to use a particular robot model and semantic description of that robot model.
      The kinematic model constructed from the parsed descriptions is also passed in. */
  virtual bool configure(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                         const boost::shared_ptr<const srdf::Model> &srdf_model,
                         const planning_models::KinematicModelPtr &kmodel);

  /** \brief Return a new planning scene that uses this one as parent. */
  virtual PlanningScenePtr diff(void) const;
  
  /** \brief Return a new planning scene that uses this one as parent and has the diffs specified by \e msg applied. */
  PlanningScenePtr diff(const moveit_msgs::PlanningScene &msg) const;
  
  /** \brief Get the parent scene (whith respect to which the diffs are maintained). This may be empty */
  const PlanningSceneConstPtr& getParent(void) const
  {
    return parent_;
  }

  /** \brief Get the frame in which planning is performed */
  const std::string& getPlanningFrame(void) const
  {
    // if we have an updated set of transforms, return it; otherwise, return the parent one
    return ftf_ ? ftf_->getTargetFrame() : parent_->getPlanningFrame();
  }

  /** \brief Get the kinematic model for which the planning scene is maintained */
  const planning_models::KinematicModelConstPtr& getKinematicModel(void) const
  {
    // the kinematic model does not change
    return parent_ ? parent_->getKinematicModel() : kmodel_const_;
  }

  /** \brief Get the state at which the robot is assumed to be */
  const planning_models::KinematicState& getCurrentState(void) const
  {
    // if we have an updated state, return it; otherwise, return the parent one
    return kstate_ ? *kstate_ : parent_->getCurrentState();
  }
  /** \brief Get the state at which the robot is assumed to be */
  planning_models::KinematicState& getCurrentState(void);

  /** \brief Get the allowed collision matrix */
  const collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrix(void) const
  {
    return acm_ ? *acm_ : parent_->getAllowedCollisionMatrix();
  }
  /** \brief Get the allowed collision matrix */
  collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrix(void);

  /** \brief Get the set of fixed transforms from known frames to the planning frame */
  const planning_models::TransformsConstPtr& getTransforms(void) const
  {
    // if we have updated transforms, return those
    return (ftf_const_ || !parent_) ? ftf_const_ : parent_->getTransforms();
  }
  /** \brief Get the set of fixed transforms from known frames to the planning frame */
  const planning_models::TransformsPtr& getTransforms(void);

  /** \brief Get the representation of the collision world */
  const collision_detection::CollisionWorldConstPtr& getCollisionWorld(void) const
  {
    // we always have a world representation
    return cworld_const_;
  }

  //brief Get the representation of the collision world
  const collision_detection::CollisionWorldPtr& getCollisionWorld(void)
  {
    // we always have a world representation
    return cworld_;
  }

  /** \brief Get the representation of the collision robot */
  const collision_detection::CollisionRobotConstPtr& getCollisionRobot(void) const
  {
    // if we have an updated robot, return that one
    return (crobot_const_ || !parent_) ? crobot_const_ : parent_->getCollisionRobot();
  }

  /** \brief Get the representation of the collision robot */
  const collision_detection::CollisionRobotConstPtr& getCollisionRobotUnpadded(void) const
  {
    // if we have an updated robot, return that one
    return (crobot_unpadded_const_ || !parent_) ? crobot_unpadded_const_ : parent_->getCollisionRobotUnpadded();
  }

  /** \brief Get the representation of the collision robot */
  const collision_detection::CollisionRobotPtr& getCollisionRobot(void);

  /** \brief Check whether the current state is in collision */
  void checkCollision(const collision_detection::CollisionRequest& req,
                      collision_detection::CollisionResult &res) const;

  /** \brief Check whether a specified state (\e kstate) is in collision */
  void checkCollision(const collision_detection::CollisionRequest& req,
                      collision_detection::CollisionResult &res,
                      const planning_models::KinematicState &kstate) const;

  /** \brief Check whether a specified state (\e kstate) is in collision, with respect to a given
      allowed collision matrix (\e acm) */
  void checkCollision(const collision_detection::CollisionRequest& req,
                      collision_detection::CollisionResult &res,
                      const planning_models::KinematicState &kstate,
                      const collision_detection::AllowedCollisionMatrix& acm) const;
    
  /** \brief Check whether the current state is in collision, 
      but use a collision_detection::CollisionRobot instance that has no padding.  */
  void checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
			      collision_detection::CollisionResult &res) const;

  /** \brief Check whether a specified state (\e kstate) is in collision, 
      but use a collision_detection::CollisionRobot instance that has no padding.  */
  void checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                              collision_detection::CollisionResult &res,
                              const planning_models::KinematicState &kstate) const;

  /** \brief Check whether a specified state (\e kstate) is in collision, with respect to a given
      allowed collision matrix (\e acm), but use a collision_detection::CollisionRobot instance that has no padding.  */
  void checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                              collision_detection::CollisionResult &res,
                              const planning_models::KinematicState &kstate,
                              const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Check whether the current state is in self collision */
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult &res) const;

  /** \brief Check whether a specified state (\e kstate) is in self collision */
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult &res,
                          const planning_models::KinematicState &kstate) const;

  /** \brief Check whether a specified state (\e kstate) is in self collision, with respect to a given
      allowed collision matrix (\e acm) */
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult &res,
                          const planning_models::KinematicState &kstate,
                          const collision_detection::AllowedCollisionMatrix& acm) const;
  
  /** \brief Get the names of the links that are involved in collisions for the current state */
  void getCollidingLinks(std::vector<std::string> &links) const;
  
  /** \brief Get the names of the links that are involved in collisions for the state \e kstate */
  void getCollidingLinks(std::vector<std::string> &links,
                         const planning_models::KinematicState &kstate) const;
  
  /** \brief  Get the names of the links that are involved in collisions for the state \e kstate given the
      allowed collision matrix (\e acm) */
  void getCollidingLinks(std::vector<std::string> &links,
                         const planning_models::KinematicState &kstate,
                         const collision_detection::AllowedCollisionMatrix& acm) const;
  
  /** \brief The distance between the robot model at state \e kstate to the nearest collision */
  double distanceToCollision(const planning_models::KinematicState &kstate) const;

  /** \brief The distance between the robot model at state \e kstate to the nearest collision, if the robot has no padding */
  double distanceToCollisionUnpadded(const planning_models::KinematicState &kstate) const;

  /** \brief The distance between the robot model at state \e kstate to the nearest collision, ignoring distances between elements that always allowed to collide. */
  double distanceToCollision(const planning_models::KinematicState &kstate, const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief The distance between the robot model at state \e kstate to the nearest collision, ignoring distances between elements that always allowed to collide, if the robot has no padding. */
  double distanceToCollisionUnpadded(const planning_models::KinematicState &kstate, const collision_detection::AllowedCollisionMatrix& acm) const;
  
  /** \brief Check if this planning scene has been configured or not */
  bool isConfigured(void) const
  {
    return parent_ ? configured_ && parent_->isConfigured() : configured_;
  }

  /** \brief Fill the message \e scene with the differences between this instance of PlanningScene with respect to the parent.
      If there is no parent, everything is considered to be a diff and the function behaves like getPlanningSceneMsg() */
  void getPlanningSceneDiffMsg(moveit_msgs::PlanningScene &scene) const;

  /** \brief Construct a message (\e scene) with all the necessary data so that the scene can be later reconstructed to be
      exactly the same using setPlanningSceneMsg() */
  void getPlanningSceneMsg(moveit_msgs::PlanningScene &scene) const;

  /** \brief Apply changes to this planning scene as diffs, even if the message itself is not marked as being a diff (is_diff
      member). A parent is not required to exist. However, the existing data in the planning instance is not cleared. Data from
      the message is only appended (and in cases such as e.g., the robot state, is overwritten). */
  void setPlanningSceneDiffMsg(const moveit_msgs::PlanningScene &scene);

  /** \brief Set this instance of a planning scene to be the same as the one serialized in the \e scene message, even if the message itself is marked as being a diff (is_diff member) */
  void setPlanningSceneMsg(const moveit_msgs::PlanningScene &scene);
  
  /** \brief Call setPlanningSceneMsg() or setPlanningSceneDiffMsg() depending on how the is_diff member of the message is set */
  void usePlanningSceneMsg(const moveit_msgs::PlanningScene &scene);

  bool processCollisionObjectMsg(const moveit_msgs::CollisionObject &object);
  bool processAttachedCollisionObjectMsg(const moveit_msgs::AttachedCollisionObject &object);

  void processCollisionMapMsg(const moveit_msgs::CollisionMap &map);
  void processOctomapMsg(const octomap_msgs::OctomapBinaryWithPose &map);
  void processOctomapMsg(const octomap_msgs::OctomapBinary &map); 
  void processOctomapPtr(const boost::shared_ptr<const octomap::OcTree> &octree, const Eigen::Affine3d &t);

  /** \brief This function is not consistent with the rest of the functions in this file. It will be removed */
  __attribute__((deprecated)) bool getCollisionObjectMsg(const std::string& ns, moveit_msgs::CollisionObject& obj) const;

  /** \brief Is this really needed? Shapes can be converted to markers;  */
  __attribute__((deprecated))  void getCollisionObjectMarkers(visualization_msgs::MarkerArray& arr,
                                                              const std_msgs::ColorRGBA& default_color,
                                                              const std::string& ns=std::string(""),
                                                              const ros::Duration& lifetime = ros::Duration(0.0)) const;
  
  /** \brief Set the current robot state to be \e state. If not
      all joint values are specified, the previously maintained
      joint values are kept. */
  void setCurrentState(const moveit_msgs::RobotState &state);

  /** \brief Set the current robot state */
  void setCurrentState(const planning_models::KinematicState &state);
  
  /** \brief Get the colors associated to the various objects in the scene */
  const ColorMap& getObjectColors(void) const
  {
    return colors_ ? *colors_ : parent_->getObjectColors();
  }
  
  bool hasColor(const std::string &id) const;

  const std_msgs::ColorRGBA& getColor(const std::string &id) const;
  void setColor(const std::string &id, const std_msgs::ColorRGBA &color);
  void removeColor(const std::string &id);
  void getKnownColors(ColorMap &kc) const;
  
  /** \brief Clear the diffs accumulated for this planning scene, with respect to the parent. This function is a no-op if there is no parent specified. */
  void clearDiffs(void);

  /** \brief If there is a parent specified for this scene, then the diffs with respect to that parent are applied to a specified planning scene, whatever
      that scene may be. If there is no parent specified, this function is a no-op. */
  void pushDiffs(const PlanningScenePtr &scene);

  /** \brief Make sure that all the data maintained in this
      scene is local. All unmodified data is copied from the
      parent and the pointer to the parent is discarded. */
  void decoupleParent(void);
  
  /** \brief Specify a predicate that decides whether states are considered valid or invalid for reasons beyond ones covered by collision checking and contraint evaluation.
      This is useful for setting up problem specific constraints (e.g., stability) */
  void setStateFeasibilityPredicate(const StateFeasibilityFn &fn)
  {
    state_feasibility_ = fn;
  }

  /** \brief Get the predicate that decides whether states are considered valid or invalid for reasons beyond ones covered by collision checking and contraint evaluation. */
  const StateFeasibilityFn& getStateFeasibilityPredicate(void) const
  {
    return state_feasibility_;
  }
  
  /** \brief Specify a predicate that decides whether motion segments are considered valid or invalid for reasons beyond ones covered by collision checking and contraint evaluation.  */
  void setMotionFeasibilityPredicate(const MotionFeasibilityFn &fn)
  {
    motion_feasibility_ = fn;
  }

  /** \brief Get the predicate that decides whether motion segments are considered valid or invalid for reasons beyond ones covered by collision checking and contraint evaluation. */
  const MotionFeasibilityFn& getMotionFeasibilityPredicate(void) const
  {
    return motion_feasibility_;
  }

  /** \brief Check if the current state is in collision (with the environment or self collision) */
  bool isStateColliding(bool verbose = false) const;
  
  /** \brief Check if a given state is in collision (with the environment or self collision) */
  bool isStateColliding(const moveit_msgs::RobotState &state, bool verbose = false) const;

  /** \brief Check if a given state is in collision (with the environment or self collision) */
  bool isStateColliding(const planning_models::KinematicState &state, bool verbose = false) const;

  /** \brief Check if a given state is feasible, in accordance to the feasibility predicate specified by setStateFeasibilityPredicate(). Returns true if no feasibility predicate was specified. */
  bool isStateFeasible(const moveit_msgs::RobotState &state, bool verbose = false) const;

  /** \brief Check if a given state is feasible, in accordance to the feasibility predicate specified by setStateFeasibilityPredicate(). Returns true if no feasibility predicate was specified. */
  bool isStateFeasible(const planning_models::KinematicState &state, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const moveit_msgs::RobotState &state, const moveit_msgs::Constraints &constr, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const planning_models::KinematicState &state, const moveit_msgs::Constraints &constr, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const moveit_msgs::RobotState &state,  const kinematic_constraints::KinematicConstraintSet &constr, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const planning_models::KinematicState &state,  const kinematic_constraints::KinematicConstraintSet &constr, bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions and feasibility */
  bool isStateValid(const moveit_msgs::RobotState &state, bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions and feasibility */
  bool isStateValid(const planning_models::KinematicState &state, bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions, feasibility  and whether the user specified validity conditions hold as well */
  bool isStateValid(const moveit_msgs::RobotState &state, const moveit_msgs::Constraints &constr, bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions, feasibility  and whether the user specified validity conditions hold as well */
  bool isStateValid(const planning_models::KinematicState &state, const moveit_msgs::Constraints &constr, bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions, feasibility  and whether the user specified validity conditions hold as well */
  bool isStateValid(const planning_models::KinematicState &state, const kinematic_constraints::KinematicConstraintSet &constr, bool verbose = false) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance and feasibility) */
  bool isPathValid(const moveit_msgs::RobotState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const moveit_msgs::RobotState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const moveit_msgs::RobotState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const moveit_msgs::Constraints& goal_constraints,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const moveit_msgs::RobotState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const std::vector<moveit_msgs::Constraints>& goal_constraints,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const planning_models::KinematicState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const moveit_msgs::Constraints& goal_constraints,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const planning_models::KinematicState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const planning_models::KinematicState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;
  
  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const planning_models::KinematicState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const std::vector<moveit_msgs::Constraints>& goal_constraints,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const planning_models::KinematicTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const std::vector<moveit_msgs::Constraints>& goal_constraints,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const planning_models::KinematicTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const moveit_msgs::Constraints& goal_constraints,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance and feasibility) */
  bool isPathValid(const planning_models::KinematicTrajectory &trajectory,
                   bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;
  
  /** \brief Get the top \e max_costs cost sources for a specified trajectory. The resulting costs are stored in \e costs */
  void getCostSources(const planning_models::KinematicTrajectory &trajectory, std::size_t max_costs,
                      std::set<collision_detection::CostSource> &costs, double overlap_fraction = 0.9) const;

  /** \brief Get the top \e max_costs cost sources for a specified trajectory, but only for group \e group_name. The resulting costs are stored in \e costs */
  void getCostSources(const planning_models::KinematicTrajectory &trajectory, std::size_t max_costs,
                      const std::string &group_name, std::set<collision_detection::CostSource> &costs, double overlap_fraction = 0.9) const;

  /** \brief Get the top \e max_costs cost sources for a specified state. The resulting costs are stored in \e costs */
  void getCostSources(const planning_models::KinematicState &state, std::size_t max_costs,
                      std::set<collision_detection::CostSource> &costs) const;
  
  /** \brief Get the top \e max_costs cost sources for a specified state, but only for group \e group_name. The resulting costs are stored in \e costs */
  void getCostSources(const planning_models::KinematicState &state, std::size_t max_costs,
                      const std::string &group_name, std::set<collision_detection::CostSource> &costs) const;
  
  /** \brief Check if a message includes any information about a planning scene, or it is just a default, empty message. */
  static bool isEmpty(const moveit_msgs::PlanningScene &msg);
  
  /** \brief Clone a planning scene. Even if the scene \e scene depends on a parent, the cloned scene will not. */
  static PlanningScenePtr clone(const PlanningSceneConstPtr &scene);
  
protected:

  void getPlanningSceneMsgCollisionObject(moveit_msgs::PlanningScene &scene, const std::string &ns) const;
  void getPlanningSceneMsgCollisionObjects(moveit_msgs::PlanningScene &scene) const;
  void getPlanningSceneMsgCollisionMap(moveit_msgs::PlanningScene &scene) const;
  void getPlanningSceneMsgOctomap(moveit_msgs::PlanningScene &scene) const;
  
  struct CollisionDetectionAllocBase
  {         
    virtual collision_detection::CollisionRobotPtr allocateRobot(const planning_models::KinematicModelConstPtr &kmodel) = 0;
    virtual collision_detection::CollisionRobotPtr allocateRobot(const collision_detection::CollisionRobotConstPtr &copy) = 0;
    virtual collision_detection::CollisionWorldPtr allocateWorld(void) = 0;
    virtual collision_detection::CollisionWorldPtr allocateWorld(const collision_detection::CollisionWorldConstPtr &copy) = 0;
    virtual CollisionDetectionAllocBase* clone(void) = 0;
  };
  
  template<typename CollisionWorldType, typename CollisionRobotType>
  struct CollisionDetectionAlloc : public CollisionDetectionAllocBase
  {
    BOOST_CONCEPT_ASSERT((boost::Convertible<CollisionWorldType*, collision_detection::CollisionWorld*>));
    BOOST_CONCEPT_ASSERT((boost::Convertible<CollisionRobotType*, collision_detection::CollisionRobot*>));
    
    virtual collision_detection::CollisionRobotPtr allocateRobot(const planning_models::KinematicModelConstPtr &kmodel)
    {
      return collision_detection::CollisionRobotPtr(new CollisionRobotType(kmodel));
    }
    virtual collision_detection::CollisionWorldPtr allocateWorld(void)
    {    
      return collision_detection::CollisionWorldPtr(new CollisionWorldType());
    }
    virtual collision_detection::CollisionRobotPtr allocateRobot(const collision_detection::CollisionRobotConstPtr &copy)
    {
      return collision_detection::CollisionRobotPtr(new CollisionRobotType(static_cast<const CollisionRobotType&>(*copy)));
    }
    virtual collision_detection::CollisionWorldPtr allocateWorld(const  collision_detection::CollisionWorldConstPtr &copy)
    {    
      return collision_detection::CollisionWorldPtr(new CollisionWorldType(static_cast<const CollisionWorldType&>(*copy)));
    }
    virtual CollisionDetectionAllocBase* clone(void)
    {
      return new CollisionDetectionAlloc<CollisionWorldType, CollisionRobotType>();
    }    
  };

  std::string                                    name_;
	
  PlanningSceneConstPtr                          parent_;

  planning_models::KinematicModelPtr             kmodel_;
  planning_models::KinematicModelConstPtr        kmodel_const_;

  planning_models::KinematicStatePtr             kstate_;

  planning_models::TransformsPtr                 ftf_;
  planning_models::TransformsConstPtr            ftf_const_;

  boost::scoped_ptr<CollisionDetectionAllocBase> collision_detection_allocator_;
  
  collision_detection::CollisionRobotPtr         crobot_unpadded_;
  collision_detection::CollisionRobotConstPtr    crobot_unpadded_const_;
  collision_detection::CollisionRobotPtr         crobot_;
  collision_detection::CollisionRobotConstPtr    crobot_const_;

  collision_detection::CollisionWorldPtr         cworld_;
  collision_detection::CollisionWorldConstPtr    cworld_const_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  StateFeasibilityFn                             state_feasibility_;
  MotionFeasibilityFn                            motion_feasibility_;

  boost::scoped_ptr<ColorMap>                    colors_;
  
  bool                                           configured_;

};

}


#endif
