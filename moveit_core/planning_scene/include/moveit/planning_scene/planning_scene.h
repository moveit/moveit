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

/* Author: Ioan Sucan, Acorn Pooley */

#ifndef MOVEIT_PLANNING_SCENE_PLANNING_SCENE_
#define MOVEIT_PLANNING_SCENE_PLANNING_SCENE_

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection/world_diff.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/macros/deprecation.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PlanningSceneComponents.h>
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
typedef boost::function<bool(const robot_state::RobotState&, bool)> StateFeasibilityFn;

/** \brief This is the function signature for additional feasibility checks to be imposed on motions segments between states (in addition to respecting constraints and collision avoidance).
    The order of the arguments matters: the notion of feasibility is to be checked for motion segments that start at the first state and end at the second state. The third argument indicates
    whether the check should be verbose or not. */
typedef boost::function<bool(const robot_state::RobotState&, const robot_state::RobotState&, bool)> MotionFeasibilityFn;

/** \brief A map from object names (e.g., attached bodies, collision objects) to their colors */
typedef std::map<std::string, std_msgs::ColorRGBA> ObjectColorMap;

/** \brief A map from object names (e.g., attached bodies, collision objects) to their types */
typedef std::map<std::string, object_recognition_msgs::ObjectType> ObjectTypeMap;

/** \brief This class maintains the representation of the
    environment as seen by a planning instance. The environment
    geometry, the robot geometry and state are maintained. */
class PlanningScene : private boost::noncopyable,
                      public boost::enable_shared_from_this<PlanningScene>
{
public:

  /** \brief construct using an existing RobotModel */
  PlanningScene(const robot_model::RobotModelPtr &robot_model,
                collision_detection::WorldPtr world = collision_detection::WorldPtr(new collision_detection::World()));

  /** \brief construct using a urdf and srdf.
   * A RobotModel for the PlanningScene will be created using the urdf and srdf. */
  PlanningScene(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                const boost::shared_ptr<const srdf::Model> &srdf_model,
                collision_detection::WorldPtr world = collision_detection::WorldPtr(new collision_detection::World()));

  static const std::string COLLISION_MAP_NS;
  static const std::string OCTOMAP_NS;
  static const std::string DEFAULT_SCENE_NAME;

  ~PlanningScene();

  /** \brief Get the name of the planning scene. This is empty by default */
  const std::string& getName() const
  {
    return name_;
  }

  /** \brief Set the name of the planning scene */
  void setName(const std::string &name)
  {
    name_ = name;
  }

  /** \brief Add a new collision detector type.
   *
   * A collision detector type is specified with (a shared pointer to) an
   * allocator which is a subclass of CollisionDetectorAllocator.  This
   * identifies a combination of CollisionWorld/CollisionRobot which can ve
   * used together.
   *
   * This does nothing if this type of collision detector has already been added.
   *
   * A new PlanningScene contains an FCL collision detector.  This FCL
   * collision detector will always be available unless it is removed by
   * calling setActiveCollisionDetector() with exclusive=true.
   *
   * example: to add FCL collision detection (normally not necessary) call
   *   planning_scene->addCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
   *
   * */
  void addCollisionDetector(const collision_detection::CollisionDetectorAllocatorPtr& allocator);

  /** \brief Set the type of collision detector to use.
   * Calls addCollisionDetector() to add it if it has not already been added.
   *
   * If exclusive is true then all other collision detectors will be removed
   * and only this one will be available.
   *
   * example: to use FCL collision call
   *   planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
   */
  void setActiveCollisionDetector(const collision_detection::CollisionDetectorAllocatorPtr& allocator,
                                  bool exclusive = false);

  /** \brief Set the type of collision detector to use.
   * This type must have already been added with addCollisionDetector().
   *
   * Returns true on success, false if \e collision_detector_name is not the
   * name of a collision detector that has been previously added with
   * addCollisionDetector(). */
  bool setActiveCollisionDetector(const std::string& collision_detector_name);

  const std::string& getActiveCollisionDetectorName() const
  {
    return active_collision_->alloc_->getName();
  }

  /** \brief get the types of collision detector that have already been added.
   * These are the types which can be passed to setActiveCollisionDetector(). */
  void getCollisionDetectorNames(std::vector<std::string>& names) const;

  /** \brief Return a new child PlanningScene that uses this one as parent.
   *
   *  The child scene has its own copy of the world. It maintains a list (in
   *  world_diff_) of changes made to the child world.
   *
   *  The kmodel_, kstate_, ftf_, and acm_ are not copied.  They are shared
   *  with the parent.  So if changes to these are made in the parent they will
   *  be visible in the child.  But if any of these is modified (i.e. if the
   *  get*NonConst functions are called) in the child then a copy is made and
   *  subsequent changes to the corresponding member of the parent will no
   *  longer be visible in the child.
   */
  PlanningScenePtr diff() const;

  /** \brief Return a new child PlanningScene that uses this one as parent and
   * has the diffs specified by \e msg applied. */
  PlanningScenePtr diff(const moveit_msgs::PlanningScene &msg) const;

  /** \brief Get the parent scene (whith respect to which the diffs are maintained). This may be empty */
  const PlanningSceneConstPtr& getParent() const
  {
    return parent_;
  }

  /** \brief Get the frame in which planning is performed */
  const std::string& getPlanningFrame() const
  {
    // if we have an updated set of transforms, return it; otherwise, return the parent one
    return ftf_ ? ftf_->getTargetFrame() : parent_->getPlanningFrame();
  }

  /** \brief Get the kinematic model for which the planning scene is maintained */
  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    // the kinematic model does not change
    return kmodel_;
  }

  /** \brief Get the state at which the robot is assumed to be. */
  const robot_state::RobotState& getCurrentState() const
  {
    // if we have an updated state, return it; otherwise, return the parent one
    return kstate_ ? *kstate_ : parent_->getCurrentState();
  }
  /** \brief Get the state at which the robot is assumed to be. */
  robot_state::RobotState& getCurrentStateNonConst();

  /** \brief Get a copy of the current state with components overwritten by the state message \e update */
  robot_state::RobotStatePtr getCurrentStateUpdated(const moveit_msgs::RobotState &update) const;

  /** \brief Get the allowed collision matrix */
  const collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrix() const
  {
    return acm_ ? *acm_ : parent_->getAllowedCollisionMatrix();
  }
  /** \brief Get the allowed collision matrix */
  collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrixNonConst();

  /** \brief Get the set of fixed transforms from known frames to the planning frame */
  const robot_state::Transforms& getTransforms() const
  {
    // if we have updated transforms, return those
    return (ftf_ || !parent_) ? *ftf_ : parent_->getTransforms();
  }
  /** \brief Get the set of fixed transforms from known frames to the planning frame */
  robot_state::Transforms& getTransformsNonConst();

  /** \brief Get the transform corresponding to the frame \e id. This will be known if \e id is a link name, an attached body id or a collision object.
      Return identity when no transform is available. Use knowsFrameTransform() to test if this function will be successful or not. */
  const Eigen::Affine3d& getFrameTransform(const std::string &id) const;

  /** \brief Get the transform corresponding to the frame \e id. This will be known if \e id is a link name, an attached body id or a collision object.
      Return identity when no transform is available. Use knowsFrameTransform() to test if this function will be successful or not. */
  const Eigen::Affine3d& getFrameTransform(const robot_state::RobotState &state, const std::string &id) const;

  /** \brief Check if a transform to the frame \e id is known. This will be known if \e id is a link name, an attached body id or a collision object */
  bool knowsFrameTransform(const std::string &id) const;

  /** \brief Check if a transform to the frame \e id is known. This will be known if \e id is a link name, an attached body id or a collision object */
  bool knowsFrameTransform(const robot_state::RobotState &state, const std::string &id) const;

  /** \brief Get the representation of the world */
  const collision_detection::WorldConstPtr& getWorld() const
  {
    // we always have a world representation
    return world_const_;
  }

  //brief Get the representation of the world
  const collision_detection::WorldPtr& getWorldNonConst()
  {
    // we always have a world representation
    return world_;
  }

  /** \brief Get the active collision detector for the world */
  const collision_detection::CollisionWorldConstPtr& getCollisionWorld() const
  {
    // we always have a world representation after configure is called.
    return active_collision_->cworld_const_;
  }

  /** \brief Get the active collision detector for the robot */
  const collision_detection::CollisionRobotConstPtr& getCollisionRobot() const
  {
    return active_collision_->getCollisionRobot();
  }

  /** \brief Get the active collision detector for the robot */
  const collision_detection::CollisionRobotConstPtr& getCollisionRobotUnpadded() const
  {
    return active_collision_->getCollisionRobotUnpadded();
  }

  /** \brief Get a specific collision detector for the world.  If not found return active CollisionWorld. */
  const collision_detection::CollisionWorldConstPtr& getCollisionWorld(const std::string& collision_detector_name) const;

  /** \brief Get a specific collision detector for the padded robot.  If no found return active CollisionRobot. */
  const collision_detection::CollisionRobotConstPtr& getCollisionRobot(const std::string& collision_detector_name) const;

  /** \brief Get a specific collision detector for the unpadded robot.  If no found return active unpadded CollisionRobot. */
  const collision_detection::CollisionRobotConstPtr& getCollisionRobotUnpadded(const std::string& collision_detector_name) const;

  /** \brief Get the representation of the collision robot
   * This can be used to set padding and link scale on the active collision_robot.
   * NOTE: After modifying padding and scale on the active robot call
   * propogateRobotPadding() to copy it to all the other collision detectors. */
  const collision_detection::CollisionRobotPtr& getCollisionRobotNonConst();

  /** \brief Copy scale and padding from active CollisionRobot to other CollisionRobots.
   * This should be called after any changes are made to the scale or padding of the active
   * CollisionRobot.  This has no effect on the unpadded CollisionRobots. */
  void propogateRobotPadding();

  /** \brief Check whether the current state is in collision */
  void checkCollision(const collision_detection::CollisionRequest& req,
                      collision_detection::CollisionResult &res) const;

  /** \brief Check whether a specified state (\e kstate) is in collision */
  void checkCollision(const collision_detection::CollisionRequest& req,
                      collision_detection::CollisionResult &res,
                      const robot_state::RobotState &kstate) const;

  /** \brief Check whether a specified state (\e kstate) is in collision, with respect to a given
      allowed collision matrix (\e acm) */
  void checkCollision(const collision_detection::CollisionRequest& req,
                      collision_detection::CollisionResult &res,
                      const robot_state::RobotState &kstate,
                      const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Check whether the current state is in collision,
      but use a collision_detection::CollisionRobot instance that has no padding.  */
  void checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                              collision_detection::CollisionResult &res) const;

  /** \brief Check whether a specified state (\e kstate) is in collision,
      but use a collision_detection::CollisionRobot instance that has no padding.  */
  void checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                              collision_detection::CollisionResult &res,
                              const robot_state::RobotState &kstate) const;

  /** \brief Check whether a specified state (\e kstate) is in collision, with respect to a given
      allowed collision matrix (\e acm), but use a collision_detection::CollisionRobot instance that has no padding.  */
  void checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                              collision_detection::CollisionResult &res,
                              const robot_state::RobotState &kstate,
                              const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Check whether the current state is in self collision */
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult &res) const;

  /** \brief Check whether a specified state (\e kstate) is in self collision */
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult &res,
                          const robot_state::RobotState &kstate) const;

  /** \brief Check whether a specified state (\e kstate) is in self collision, with respect to a given
      allowed collision matrix (\e acm) */
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult &res,
                          const robot_state::RobotState &kstate,
                          const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Get the names of the links that are involved in collisions for the current state */
  void getCollidingLinks(std::vector<std::string> &links) const;

  /** \brief Get the names of the links that are involved in collisions for the state \e kstate */
  void getCollidingLinks(std::vector<std::string> &links,
                         const robot_state::RobotState &kstate) const;

  /** \brief  Get the names of the links that are involved in collisions for the state \e kstate given the
      allowed collision matrix (\e acm) */
  void getCollidingLinks(std::vector<std::string> &links,
                         const robot_state::RobotState &kstate,
                         const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief The distance between the robot model at state \e kstate to the nearest collision */
  double distanceToCollision(const robot_state::RobotState &kstate) const;

  /** \brief The distance between the robot model at state \e kstate to the nearest collision, if the robot has no padding */
  double distanceToCollisionUnpadded(const robot_state::RobotState &kstate) const;

  /** \brief The distance between the robot model at state \e kstate to the nearest collision, ignoring distances between elements that always allowed to collide. */
  double distanceToCollision(const robot_state::RobotState &kstate, const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief The distance between the robot model at state \e kstate to the nearest collision, ignoring distances between elements that always allowed to collide, if the robot has no padding. */
  double distanceToCollisionUnpadded(const robot_state::RobotState &kstate, const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Save the geometry of the planning scene to a stream, as plain text */
  void saveGeometryToStream(std::ostream &out) const;

  /** \brief Load the geometry of the planning scene from a stream */
  void loadGeometryFromStream(std::istream &in);

  /** \brief Fill the message \e scene with the differences between this instance of PlanningScene with respect to the parent.
      If there is no parent, everything is considered to be a diff and the function behaves like getPlanningSceneMsg() */
  void getPlanningSceneDiffMsg(moveit_msgs::PlanningScene &scene) const;

  /** \brief Construct a message (\e scene) with all the necessary data so that the scene can be later reconstructed to be
      exactly the same using setPlanningSceneMsg() */
  void getPlanningSceneMsg(moveit_msgs::PlanningScene &scene) const;

  /** \brief Construct a message (\e scene) with the data requested in \e comp. If all options in \e comp are filled,
      this will be a complete planning scene message */
  void getPlanningSceneMsg(moveit_msgs::PlanningScene &scene, const moveit_msgs::PlanningSceneComponents &comp) const;

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

  void processPlanningSceneWorldMsg(const moveit_msgs::PlanningSceneWorld &world);

  void processCollisionMapMsg(const moveit_msgs::CollisionMap &map);
  void processOctomapMsg(const octomap_msgs::OctomapWithPose &map);
  void processOctomapMsg(const octomap_msgs::Octomap &map);
  void processOctomapPtr(const boost::shared_ptr<const octomap::OcTree> &octree, const Eigen::Affine3d &t);

  /** \brief Set the current robot state to be \e state. If not
      all joint values are specified, the previously maintained
      joint values are kept. */
  void setCurrentState(const moveit_msgs::RobotState &state);

  /** \brief Set the current robot state */
  void setCurrentState(const robot_state::RobotState &state);

  /** \brief Set the callback to be triggered when changes are made to the current scene state */
  void setAttachedBodyUpdateCallback(const robot_state::AttachedBodyCallback &callback);

  /** \brief Set the callback to be triggered when changes are made to the current scene world */
  void setCollisionObjectUpdateCallback(const collision_detection::World::ObserverCallbackFn &callback);

  bool hasObjectColor(const std::string &id) const;

  const std_msgs::ColorRGBA& getObjectColor(const std::string &id) const;
  void setObjectColor(const std::string &id, const std_msgs::ColorRGBA &color);
  void removeObjectColor(const std::string &id);
  void getKnownObjectColors(ObjectColorMap &kc) const;

  bool hasObjectType(const std::string &id) const;

  const object_recognition_msgs::ObjectType& getObjectType(const std::string &id) const;
  void setObjectType(const std::string &id, const object_recognition_msgs::ObjectType &type);
  void removeObjectType(const std::string &id);
  void getKnownObjectTypes(ObjectTypeMap &kc) const;

  /** \brief Clear the diffs accumulated for this planning scene, with respect to the parent. This function is a no-op if there is no parent specified. */
  void clearDiffs();

  /** \brief If there is a parent specified for this scene, then the diffs with respect to that parent are applied to a specified planning scene, whatever
      that scene may be. If there is no parent specified, this function is a no-op. */
  void pushDiffs(const PlanningScenePtr &scene);

  /** \brief Make sure that all the data maintained in this
      scene is local. All unmodified data is copied from the
      parent and the pointer to the parent is discarded. */
  void decoupleParent();

  /** \brief Specify a predicate that decides whether states are considered valid or invalid for reasons beyond ones covered by collision checking and constraint evaluation.
      This is useful for setting up problem specific constraints (e.g., stability) */
  void setStateFeasibilityPredicate(const StateFeasibilityFn &fn)
  {
    state_feasibility_ = fn;
  }

  /** \brief Get the predicate that decides whether states are considered valid or invalid for reasons beyond ones covered by collision checking and constraint evaluation. */
  const StateFeasibilityFn& getStateFeasibilityPredicate() const
  {
    return state_feasibility_;
  }

  /** \brief Specify a predicate that decides whether motion segments are considered valid or invalid for reasons beyond ones covered by collision checking and constraint evaluation.  */
  void setMotionFeasibilityPredicate(const MotionFeasibilityFn &fn)
  {
    motion_feasibility_ = fn;
  }

  /** \brief Get the predicate that decides whether motion segments are considered valid or invalid for reasons beyond ones covered by collision checking and constraint evaluation. */
  const MotionFeasibilityFn& getMotionFeasibilityPredicate() const
  {
    return motion_feasibility_;
  }

  /** \brief Check if the current state is in collision (with the environment or self collision) */
  bool isStateColliding(const std::string &group = "", bool verbose = false) const;

  /** \brief Check if a given state is in collision (with the environment or self collision) */
  bool isStateColliding(const moveit_msgs::RobotState &state, const std::string &group = "", bool verbose = false) const;

  /** \brief Check if a given state is in collision (with the environment or self collision) */
  bool isStateColliding(const robot_state::RobotState &state, const std::string &group = "", bool verbose = false) const;

  /** \brief Check if a given state is feasible, in accordance to the feasibility predicate specified by setStateFeasibilityPredicate(). Returns true if no feasibility predicate was specified. */
  bool isStateFeasible(const moveit_msgs::RobotState &state, bool verbose = false) const;

  /** \brief Check if a given state is feasible, in accordance to the feasibility predicate specified by setStateFeasibilityPredicate(). Returns true if no feasibility predicate was specified. */
  bool isStateFeasible(const robot_state::RobotState &state, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const moveit_msgs::RobotState &state, const moveit_msgs::Constraints &constr, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const robot_state::RobotState &state, const moveit_msgs::Constraints &constr, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const moveit_msgs::RobotState &state,  const kinematic_constraints::KinematicConstraintSet &constr, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const robot_state::RobotState &state,  const kinematic_constraints::KinematicConstraintSet &constr, bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions and feasibility */
  bool isStateValid(const moveit_msgs::RobotState &state, const std::string &group = "", bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions and feasibility */
  bool isStateValid(const robot_state::RobotState &state, const std::string &group = "", bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions, feasibility  and whether the user specified validity conditions hold as well */
  bool isStateValid(const moveit_msgs::RobotState &state, const moveit_msgs::Constraints &constr, const std::string &group = "", bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions, feasibility  and whether the user specified validity conditions hold as well */
  bool isStateValid(const robot_state::RobotState &state, const moveit_msgs::Constraints &constr, const std::string &group = "", bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions, feasibility  and whether the user specified validity conditions hold as well */
  bool isStateValid(const robot_state::RobotState &state, const kinematic_constraints::KinematicConstraintSet &constr, const std::string &group = "", bool verbose = false) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance and feasibility) */
  bool isPathValid(const moveit_msgs::RobotState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const std::string &group = "", bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const moveit_msgs::RobotState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const std::string &group = "", bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const moveit_msgs::RobotState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const moveit_msgs::Constraints& goal_constraints,
                   const std::string &group = "", bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const moveit_msgs::RobotState &start_state,
                   const moveit_msgs::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const std::vector<moveit_msgs::Constraints>& goal_constraints,
                   const std::string &group = "", bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const robot_trajectory::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const std::vector<moveit_msgs::Constraints>& goal_constraints,
                   const std::string &group = "", bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the passed in trajectory. */
  bool isPathValid(const robot_trajectory::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const moveit_msgs::Constraints& goal_constraints,
                   const std::string &group = "", bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and constraint satisfaction). */
  bool isPathValid(const robot_trajectory::RobotTrajectory &trajectory,
                   const moveit_msgs::Constraints& path_constraints,
                   const std::string &group = "", bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance and feasibility) */
  bool isPathValid(const robot_trajectory::RobotTrajectory &trajectory,
                   const std::string &group = "", bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  /** \brief Get the top \e max_costs cost sources for a specified trajectory. The resulting costs are stored in \e costs */
  void getCostSources(const robot_trajectory::RobotTrajectory &trajectory, std::size_t max_costs,
                      std::set<collision_detection::CostSource> &costs, double overlap_fraction = 0.9) const;

  /** \brief Get the top \e max_costs cost sources for a specified trajectory, but only for group \e group_name. The resulting costs are stored in \e costs */
  void getCostSources(const robot_trajectory::RobotTrajectory &trajectory, std::size_t max_costs,
                      const std::string &group_name, std::set<collision_detection::CostSource> &costs, double overlap_fraction = 0.9) const;

  /** \brief Get the top \e max_costs cost sources for a specified state. The resulting costs are stored in \e costs */
  void getCostSources(const robot_state::RobotState &state, std::size_t max_costs,
                      std::set<collision_detection::CostSource> &costs) const;

  /** \brief Get the top \e max_costs cost sources for a specified state, but only for group \e group_name. The resulting costs are stored in \e costs */
  void getCostSources(const robot_state::RobotState &state, std::size_t max_costs,
                      const std::string &group_name, std::set<collision_detection::CostSource> &costs) const;

  /** \brief Outputs debug information about the planning scene contents */
  void printKnownObjects(std::ostream& out) const;

  /** \brief Check if a message includes any information about a planning scene, or it is just a default, empty message. */
  static bool isEmpty(const moveit_msgs::PlanningScene &msg);

  /** \brief Check if a message includes any information about a planning scene world, or it is just a default, empty message. */
  static bool isEmpty(const moveit_msgs::PlanningSceneWorld &msg);

  /** \brief Check if a message includes any information about a robot state, or it is just a default, empty message. */
  static bool isEmpty(const moveit_msgs::RobotState &msg);

  /** \brief Clone a planning scene. Even if the scene \e scene depends on a parent, the cloned scene will not. */
  static PlanningScenePtr clone(const PlanningSceneConstPtr &scene);

private:

  /* Private constructor used by the diff() methods. */
  PlanningScene(const PlanningSceneConstPtr &parent);

  /* Initialize the scene.  This should only be called by the constructors.
   * Requires a valid robot_model_ */
  void initialize();

  /* helper function to create a RobotModel from a urdf/srdf. */
  static robot_model::RobotModelPtr createRobotModel(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                                                     const boost::shared_ptr<const srdf::Model> &srdf_model);

  void getPlanningSceneMsgCollisionObject(moveit_msgs::PlanningScene &scene, const std::string &ns) const;
  void getPlanningSceneMsgCollisionObjects(moveit_msgs::PlanningScene &scene) const;
  void getPlanningSceneMsgCollisionMap(moveit_msgs::PlanningScene &scene) const;
  void getPlanningSceneMsgOctomap(moveit_msgs::PlanningScene &scene) const;
  void getPlanningSceneMsgObjectColors(moveit_msgs::PlanningScene &scene_msg) const;

  struct CollisionDetector;
  typedef boost::shared_ptr<CollisionDetector> CollisionDetectorPtr;
  typedef boost::shared_ptr<const CollisionDetector> CollisionDetectorConstPtr;

  /* \brief A set of compatible collision detectors */
  struct CollisionDetector
  {
    collision_detection::CollisionDetectorAllocatorPtr alloc_;
    collision_detection::CollisionRobotPtr             crobot_unpadded_;        // if NULL use parent's
    collision_detection::CollisionRobotConstPtr        crobot_unpadded_const_;
    collision_detection::CollisionRobotPtr             crobot_;                 // if NULL use parent's
    collision_detection::CollisionRobotConstPtr        crobot_const_;

    collision_detection::CollisionWorldPtr             cworld_;                 // never NULL
    collision_detection::CollisionWorldConstPtr        cworld_const_;

    CollisionDetectorConstPtr                          parent_;                 // may be NULL

    const collision_detection::CollisionRobotConstPtr& getCollisionRobot() const
    {
      return crobot_const_ ? crobot_const_ : parent_->getCollisionRobot();
    }
    const collision_detection::CollisionRobotConstPtr& getCollisionRobotUnpadded() const
    {
      return crobot_unpadded_const_ ? crobot_unpadded_const_ : parent_->getCollisionRobotUnpadded();
    }
    void findParent(const PlanningScene& scene);
    void copyPadding(const CollisionDetector& src);
  };
  friend struct CollisionDetector;

  typedef std::map<std::string, CollisionDetectorPtr>::iterator CollisionDetectorIterator;
  typedef std::map<std::string, CollisionDetectorPtr>::const_iterator CollisionDetectorConstIterator;

  void allocateCollisionDetectors();
  void allocateCollisionDetectors(CollisionDetector& detector);



  std::string                                    name_;         // may be empty

  PlanningSceneConstPtr                          parent_;       // Null unless this is a diff scene

  robot_model::RobotModelConstPtr                kmodel_;       // Never null (may point to same model as parent)

  robot_state::RobotStatePtr                     kstate_;       // if NULL use parent's
  robot_state::AttachedBodyCallback              current_state_attached_body_callback_; // called when changes are made to attached bodies

  robot_state::TransformsPtr                     ftf_;          // if NULL use parent's

  collision_detection::WorldPtr                  world_;        // never NULL, never shared with parent/child
  collision_detection::WorldConstPtr             world_const_;  // copy of world_
  collision_detection::WorldDiffPtr              world_diff_;   // NULL unless this is a diff scene
  collision_detection::World::ObserverCallbackFn current_world_object_update_callback_;
  collision_detection::World::ObserverHandle     current_world_object_update_observer_handle_;

  std::map<std::string, CollisionDetectorPtr>    collision_;          // never empty
  CollisionDetectorPtr                           active_collision_;   // copy of one of the entries in collision_.  Never NULL.

  collision_detection::AllowedCollisionMatrixPtr acm_;                // if NULL use parent's

  StateFeasibilityFn                             state_feasibility_;
  MotionFeasibilityFn                            motion_feasibility_;

  boost::scoped_ptr<ObjectColorMap>              object_colors_;

  // a map of object types
  boost::scoped_ptr<ObjectTypeMap>               object_types_;


};

}


#endif
