/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef EXCEPTION_TYPES_H
#define EXCEPTION_TYPES_H

#include <stdexcept>
#include <string>

namespace pilz_industrial_motion_planner_testutils
{
class TestDataLoaderReadingException : public std::runtime_error
{
public:
  TestDataLoaderReadingException(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};
}

#endif  // EXCEPTION_TYPES_H
