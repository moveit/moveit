/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PLANNING_CONTEXT_LOADER_H
#define PLANNING_CONTEXT_LOADER_H

#include "pilz_trajectory_generation/limits_container.h"

#include <memory>
#include <vector>

#include <moveit/planning_interface/planning_interface.h>

#include "pilz_trajectory_generation/limits_container.h"

namespace pilz {

/**
 * @brief Base class for all PlanningContextLoaders.
 * Since planning_interface::PlanningContext has a non empty ctor classes derived from it can not be plugins.
 * This class serves as base class for wrappers.
 */
class PlanningContextLoader
{
public:
  PlanningContextLoader();
  virtual ~PlanningContextLoader();

  /// Return the algorithm the loader uses
  virtual std::string getAlgorithm() const;

  /**
   * @brief Sets the robot model that can be passed to the planning context
   * @param model The robot model
   * @return false if could not be set
   */
  virtual bool setModel(const moveit::core::RobotModelConstPtr& model);

  /**
   * @brief Sets limits the planner can pass to the contexts
   * @param limits container of limits, no guarantee to contain the limits for all joints of the model
   * @return true if limits could be set
   */
  virtual bool setLimits(const pilz::LimitsContainer& limits);

  /**
   * @brief Return the planning context
   * @param planning_context
   * @param name context name
   * @param group name of the planning group
   * @return true on success, false otherwise
   */
  virtual bool loadContext(planning_interface::PlanningContextPtr& planning_context,
                   const std::string& name,
                   const std::string& group) const = 0;


protected:
  /**
   * @brief Return the planning context of type T
   * @param planning_context
   * @param name context name
   * @param group name of the planning group
   * @return true on success, false otherwise
   */
  template <typename T>
  bool loadContext(planning_interface::PlanningContextPtr& planning_context,
                   const std::string& name,
                   const std::string& group) const;

protected:

  /// Name of the algorithm
  std::string alg_;

  /// True if limits are set
  bool limits_set_;

  /// Limits to be used during planning
  pilz::LimitsContainer limits_;

  /// True if model is set
  bool model_set_;

  /// The robot model
  moveit::core::RobotModelConstPtr model_;
};


typedef boost::shared_ptr<PlanningContextLoader> PlanningContextLoaderPtr;                                                                             \
typedef boost::shared_ptr<const PlanningContextLoader> PlanningContextLoaderConstPtr;


template <typename T>
bool PlanningContextLoader::loadContext(planning_interface::PlanningContextPtr& planning_context,
                                                         const std::string& name,
                                                         const std::string& group) const
{
  if(limits_set_ && model_set_) {
    planning_context.reset(new T(name, group, model_, limits_));
    return true;
  }
  else
  {
    if(!limits_set_)
    {
      ROS_ERROR_STREAM("Limits are not defined. Cannot load planning context. Call setLimits loadContext");
    }
    if(!model_set_)
    {
      ROS_ERROR_STREAM("Robot model was not set");
    }
    return false;
  }
}

} // namespace



#endif // PLANNING_CONTEXT_LOADER_H
