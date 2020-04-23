/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#ifndef CHECKS_H
#define CHECKS_H

#include <gtest/gtest.h>
#include <moveit/robot_state/robot_state.h>

namespace pilz_industrial_motion_planner_testutils
{
::testing::AssertionResult isAtExpectedPosition(const robot_state::RobotState& expected,
                                                const robot_state::RobotState& actual, const double epsilon)
{
  if (expected.getVariableCount() != actual.getVariableCount())
  {
    return ::testing::AssertionFailure() << "Both states have different number of Variables";
  }

  for (size_t i = 0; i < actual.getVariableCount(); ++i)
  {
    // PLEASE NOTE: This comparision only works for reasonably
    // values. That means: Values are not to large, values are
    // reasonably close by each other.
    if (std::fabs(expected.getVariablePosition(i) - actual.getVariablePosition(i)) > epsilon)
    {
      std::stringstream msg;
      msg << expected.getVariableNames().at(i) << " position - expected: " << expected.getVariablePosition(i)
          << " actual: " << actual.getVariablePosition(i);

      return ::testing::AssertionFailure() << msg.str();
    }
  }

  return ::testing::AssertionSuccess();
}
}

#endif  // CENTERAUXILIARY_H