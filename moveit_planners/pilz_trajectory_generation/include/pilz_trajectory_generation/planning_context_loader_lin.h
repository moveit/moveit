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

#ifndef PLANNING_CONTEXT_LOADER_LIN_H
#define PLANNING_CONTEXT_LOADER_LIN_H

#include "pilz_trajectory_generation/planning_context_loader.h"

#include <moveit/planning_interface/planning_interface.h>

namespace pilz {

/**
 * @brief Plugin that can generate instances of PlanningContextLIN.
 * Generates instances of PlanningContextLIN.
 */
class PlanningContextLoaderLIN : public PlanningContextLoader
{
public:
  PlanningContextLoaderLIN();
  virtual ~PlanningContextLoaderLIN();

  /**
   * @brief return a instance of pilz::PlanningContextLIN
   * @param planning_context returned context
   * @param name
   * @param group
   * @return true on success, false otherwise
   */
  virtual bool loadContext(planning_interface::PlanningContextPtr& planning_context,
                           const std::string& name,
                           const std::string& group) const override;
};

typedef boost::shared_ptr<PlanningContextLoaderLIN> PlanningContextLoaderLINPtr;                                                                             \
typedef boost::shared_ptr<const PlanningContextLoaderLIN> PlanningContextLoaderLINConstPtr;

} // namespace

#endif // PLANNING_CONTEXT_LOADER_LIN_H
