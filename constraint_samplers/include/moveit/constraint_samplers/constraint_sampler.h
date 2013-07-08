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

#ifndef MOVEIT_CONSTRAINT_SAMPLERS_CONSTRAINT_SAMPLER_
#define MOVEIT_CONSTRAINT_SAMPLERS_CONSTRAINT_SAMPLER_

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <boost/shared_ptr.hpp>
#include <vector>

/**
 * \brief The constraint samplers namespace contains a number of
 * methods for generating samples based on a constraint or set of
 * constraints.
 *
 * It intended for use by any algorithm that requires a
 * constraint-aware sampling strategy.
 */
namespace constraint_samplers
{
/**
 * \brief ConstraintSampler is an abstract base class that allows the
 * sampling of a kinematic state for a particular group of a robot.
 */
class ConstraintSampler
{
public:

  static const unsigned int DEFAULT_MAX_SAMPLING_ATTEMPTS = 2; /**< \brief The default value associated with a sampling request.  By default if a valid sample cannot be produced in this many attempts, it returns with no sample */

  /**
   * \brief Constructor
   *
   * @param [in] scene The planning scene that will be used for constraint checking
   * @param [in] group_name The name of the group that will be sampled.  Will be invalid if no group name is passed in or the joint model group cannot be found in the kinematic model
   */
  ConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name);

  virtual ~ConstraintSampler()
  {
  }

  /**
   * \brief Function for configuring a constraint sampler given a Constraints message.
   *
   * @param [in] constr The constraints from which to construct a sampler
   *
   * @return True if the configuration is successful.  If true, \ref isValid should also true.  If false, \ref isValid should return false
   */
  virtual bool configure(const moveit_msgs::Constraints &constr) = 0;

  /**
   * \brief Gets the group name set in the constructor
   *
   * @return The group name
   */
  const std::string& getGroupName() const
  {
    return getJointModelGroup()->getName();
  }

  /**
   * \brief Gets the joint model group
   *
   *
   * @return The joint model group
   */
  const robot_model::JointModelGroup* getJointModelGroup() const
  {
    return jmg_;
  }

  /**
   * \brief Gets the planning scene
   *
   *
   * @return The planning scene as a const ptr
   */
  const planning_scene::PlanningSceneConstPtr& getPlanningScene() const
  {
    return scene_;
  }

  /**
   * \brief Return the names of the mobile frames whose pose is needed when sample() is called.
   *
   * Mobile frames mean frames other than the reference frame of the
   * kinematic model.  These frames may move when the kinematic state
   * changes.  Frame dependency can help determine an ordering from a
   * set of constraint samplers - for more information see the derived
   * class documentation for \ref UnionConstraintSampler.
   *
   * @return The list of names whose pose is needed
   */
  const std::vector<std::string>& getFrameDependency() const
  {
    return frame_depends_;
  }

  /**
   * \brief Gets the callback used to determine state validity during sampling. The sampler will attempt to satisfy this constraint if possible, but there is no guarantee.
   */
  const robot_state::StateValidityCallbackFn& getStateValidityCallback() const
  {
    return state_validity_callback_;
  }

  /**
   * \brief Sets the callback used to determine the state validity during sampling. The sampler will attempt to satisfy this constraint if possible, but there is no guarantee.
   *
   * @param callback The callback to set
   */
  void setStateValidityCallback(const robot_state::StateValidityCallbackFn &callback)
  {
    state_validity_callback_ = callback;
  }

  /**
   * \brief Samples given the constraints, populating the joint state
   * group.  The value DEFAULT_MAX_SAMPLING_ATTEMPTS will be passed in
   * as the maximum number of attempts to make to take a sample.
   *
   * @param [out] jsg The joint state group into which the values will be placed
   * @param [in] reference_state Reference state that will be used to do transforms or perform other actions
   *
   * @return True if a sample was successfully taken, false otherwise
   */
  bool sample(robot_state::JointStateGroup *jsg,
              const robot_state::RobotState &reference_state)
  {
    return sample(jsg, reference_state, DEFAULT_MAX_SAMPLING_ATTEMPTS);
  }

  /**
   * \brief Project a sample given the constraints, updating the joint state
   * group. The value DEFAULT_MAX_SAMPLING_ATTEMPTS will be passed in
   * as the maximum number of attempts to make to project the sample.
   *
   * @param [out] jsg The joint state group which specifies the state to be projected, according to the constraints
   * @param [in] reference_state Reference state that will be used to do transforms or perform other actions
   *
   * @return True if a sample was successfully projected, false otherwise
   */
  bool project(robot_state::JointStateGroup *jsg,
               const robot_state::RobotState &reference_state)
  {
    return project(jsg, reference_state, DEFAULT_MAX_SAMPLING_ATTEMPTS);
  }

  /**
   * \brief Samples given the constraints, populating the joint state
   * group.  This function allows the parameter max_attempts to be set.
   *
   * @param [out] jsg The joint state group into which the values will be placed
   * @param [in] reference_state Reference state that will be used to do transforms or perform other actions
   * @param [in] max_attempts The maximum number of times to attempt to draw a sample.  If no sample has been drawn in this number of attempts, false will be returned.
   *
   * @return True if a sample was successfully taken, false otherwise
   */
  virtual bool sample(robot_state::JointStateGroup *jsg,
                      const robot_state::RobotState &reference_state,
                      unsigned int max_attempts) = 0;

  /**
   * \brief Project a sample given the constraints, updating the joint state
   * group. This function allows the parameter max_attempts to be set.
   *
   * @param [out] jsg The joint state group which specifies the state to be projected, according to the constraints
   * @param [in] reference_state Reference state that will be used to do transforms or perform other actions
   * @param [in] max_attempts The maximum number of times to attempt to draw a sample.  If no sample has been drawn in this number of attempts, false will be returned.
   *
   * @return True if a sample was successfully projected, false otherwise
   */
  virtual bool project(robot_state::JointStateGroup *jsg,
                       const robot_state::RobotState &reference_state,
                       unsigned int max_attempts) = 0;

  /**
   * \brief Returns whether or not the constraint sampler is valid or not.  To be valid, the joint model group must be available in the kinematic model.
   *
   * @return True if the sampler is valid, and otherwise false.
   */
  bool isValid() const
  {
    return is_valid_;
  }

  /** \brief Check if the sampler is set to verbose mode */
  bool getVerbose() const
  {
    return verbose_;
  }

  /** \brief Enable/disable verbose mode for sampler */
  void setVerbose(bool flag)
  {
    verbose_ = flag;
  }

protected:

  /**
   * \brief Clears all data from the constraint.
   *
   */
  virtual void clear();

  bool                                  is_valid_;  /**< \brief  Holds the value for validity */

  planning_scene::PlanningSceneConstPtr scene_; /**< \brief Holds the planning scene */
  const robot_model::JointModelGroup   *jmg_; /**< \brief Holds the joint model group associated with this constraint */
  std::vector<std::string>              frame_depends_; /**< \brief Holds the set of frames that must exist in the reference state to allow samples to be drawn */
  robot_state::StateValidityCallbackFn  state_validity_callback_; /**< \brief Holds the callback for state validity */
  bool                                  verbose_; /**< \brief True if verbosity is on */
};

typedef boost::shared_ptr<ConstraintSampler> ConstraintSamplerPtr; /**< \brief boost shared_ptr to a ConstraintSampler */
typedef boost::shared_ptr<const ConstraintSampler> ConstraintSamplerConstPtr; /**< \brief boost shared_ptr to a const ConstraintSampler */
}


#endif
