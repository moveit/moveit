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

#include <memory>

#include <gtest/gtest.h>

#include <pilz_trajectory_generation/pilz_command_planner.h>
#include <pilz_trajectory_generation/planning_context_loader_ptp.h>
#include <pilz_trajectory_generation/planning_exceptions.h>

using namespace pilz;

TEST(CommandPlannerTestDirect, ExceptionCoverage)
{
  std::shared_ptr<PlanningException> p_ex {new PlanningException("")};
  std::shared_ptr<ContextLoaderRegistrationException> clr_ex {new ContextLoaderRegistrationException("")};
}

/**
  *  This test uses pilz::CommandPlanner directly and is thus seperated from unittest_pilz_command_planner.cpp
  *  since plugin loading via pluginlib does not allow loading of classes already defined.
  */

/**
 * @brief Check that a exception is thrown if a already loaded PlanningContextLoader is loaded
 *
 * It this point the planning_instance_ has loaded ptp, lin, circ.
 * A additional ptp is loaded which should throw the respective exception.
 */
TEST(CommandPlannerTestDirect, CheckDoubleLoadingException)
{

  /// Registed a found loader
  pilz::CommandPlanner planner;
  pilz::PlanningContextLoaderPtr planning_context_loader(new pilz::PlanningContextLoaderPTP());


  EXPECT_NO_THROW(planner.registerContextLoader(planning_context_loader));


  EXPECT_THROW(
     planner.registerContextLoader(planning_context_loader),
     pilz::ContextLoaderRegistrationException
  );
}


/**
 * @brief Check that getPlanningContext() fails if the underlying ContextLoader fails to load the context.
 */
TEST(CommandPlannerTestDirect, FailOnLoadContext)
{

  pilz::CommandPlanner planner;

  // Mock of failing PlanningContextLoader
  class TestPlanningContextLoader : public pilz::PlanningContextLoader
  {
  public:
    std::string getAlgorithm() const override
    {
      return "Test_Algorithm";
    }

    bool loadContext(planning_interface::PlanningContextPtr& /* planning_context */,
                     const std::string& /* name */,
                     const std::string& /* group */) const override
    {
      // Mock behaviour: Cannot load planning context.
      return false;
    }
  };

  /// Registed a found loader
  pilz::PlanningContextLoaderPtr planning_context_loader(new TestPlanningContextLoader());
  planner.registerContextLoader(planning_context_loader);

  moveit_msgs::MotionPlanRequest req;
  req.planner_id = "Test_Algorithm";

  moveit_msgs::MoveItErrorCodes error_code;
  EXPECT_FALSE(planner.getPlanningContext(nullptr, req, error_code));
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::PLANNING_FAILED, error_code.val);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
