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

#ifndef PLANNING_CONTEXT_LOADER_PTP_H
#define PLANNING_CONTEXT_LOADER_PTP_H

#include "pilz_trajectory_generation/planning_context_loader.h"

#include <moveit/planning_interface/planning_interface.h>

namespace pilz {

/**
 * @brief Plugin that can generate instances of PlanningContextPTP.
 * Generates instances of PlanningContextPTP.
 */
class PlanningContextLoaderPTP : public PlanningContextLoader
{
public:
  PlanningContextLoaderPTP();
  virtual ~PlanningContextLoaderPTP();

  /**
   * @brief return a instance of pilz::PlanningContextPTP
   * @param planning_context returned context
   * @param name
   * @param group
   * @return true on success, false otherwise
   */
  virtual bool loadContext(planning_interface::PlanningContextPtr& planning_context,
                           const std::string& name,
                           const std::string& group) const override;
};

typedef boost::shared_ptr<PlanningContextLoaderPTP> PlanningContextLoaderPTPPtr;                                                                             \
typedef boost::shared_ptr<const PlanningContextLoaderPTP> PlanningContextLoaderPTPConstPtr;

} // namespace

#endif // PLANNING_CONTEXT_LOADER_PTP_H
