#include "load_test_robot.h"

#include <gtest/gtest.h>

#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>

class TestPathLengthDirectInfSampler : public ompl_interface_testing::LoadTestRobot, public testing::Test
{
public:
  TestPathLengthDirectInfSampler(const std::string& robot_name, const std::string& group_name)
    : LoadTestRobot("panda", "panda_arm")
  {
    ompl_interface::ModelBasedStateSpaceSpecification spec(robot_model_, robot_name_);
    ompl_interface::JointModelStateSpace ss(spec);
    ss.setPlanningVolume(-1, 1, -1, 1, -1, 1);
    ss.setup();

    spec.
  }

protected:
  std::shared_ptr<ompl::base::PathLengthDirectInfSampler> sampler_;
};

// TEST_F(TestPathLengthDirectInfSampler, test)
// {
// }

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
