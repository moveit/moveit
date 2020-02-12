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

#ifndef PLANNING_EXCEPTIONS_H
#define PLANNING_EXCEPTIONS_H

#include <stdexcept>

namespace pilz {

/**
 * @class PlanningException
 * @brief A base class for all pilz_planners exceptions inheriting from std::runtime_exception
 */
class PlanningException: public std::runtime_error
{
  public:
    PlanningException(const std::string error_desc) : std::runtime_error(error_desc) {}
};

/**
 * @class PlanningContextFactoryRegistrationException
 * @brief An exception class thrown when the planner manager is unable to load a factory
 *
 * Loading a PlanningContextFactory can fail if a factory is loaded that
 * would provide a command which was already provided by another factory loaded before.
 */
class ContextLoaderRegistrationException: public PlanningException
{
  public:
    ContextLoaderRegistrationException(const std::string error_desc) : PlanningException(error_desc) {}
};

}

#endif // PLANNING_EXCEPTIONS_H
