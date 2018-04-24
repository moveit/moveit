#include <gtest/gtest.h>
#include <jog_arm/compliant_control/compliant_control.h>

using testing::Types;

namespace compliant_control_test
{
TEST(compliantControlTest, constructor)
{
  std::vector<double> stiffness(6, 1.);
  std::vector<double> deadband(6, 10.);
  std::vector<double> end_condition_wrench(6, 0.0);
  double filterCutoff = 10.;
  std::vector<double> haltF(6, 20.0);
  geometry_msgs::WrenchStamped ftData0;
  double highestAllowableForce = 100.;
  double highestAllowableTorque = 100.;
  compliant_control::CompliantControl control(stiffness, deadband, end_condition_wrench, filterCutoff, ftData0,
                                              highestAllowableForce, highestAllowableTorque);
}

TEST(compliantControlTest, setStiffness)
{
  std::vector<double> stiffness(6, 1.);
  std::vector<double> deadband(6, 10.);
  std::vector<double> end_condition_wrench(6, 0.0);
  double filterCutoff = 10.;
  std::vector<double> haltF(6, 20.0);
  geometry_msgs::WrenchStamped ftData0;
  double highestAllowableForce = 100.;
  double highestAllowableTorque = 100.;
  compliant_control::CompliantControl control(stiffness, deadband, end_condition_wrench, filterCutoff, ftData0,
                                              highestAllowableForce, highestAllowableTorque);

  std::vector<double> bin(6, DBL_MAX);
  bin[0] = 12.0;
  bin[1] = 10.0;
  bin[3] = 1e-4;
  bin[4] = -12.0;

  control.setStiffness(bin);

  EXPECT_NEAR(control.stiffness_[0], 12.0, 1e-4);
  EXPECT_NEAR(control.stiffness_[1], 10.0, 1e-4);
  EXPECT_NEAR(control.stiffness_[2], DBL_MAX, 1e-4);
  EXPECT_NEAR(control.stiffness_[3], DBL_MAX, 1e-4);
  EXPECT_NEAR(control.stiffness_[4], -12.0, 1e-4);
  EXPECT_NEAR(control.stiffness_[5], DBL_MAX, 1e-4);
}

TEST(compliantControlTest, setEndCondition)
{
  std::vector<double> stiffness(6, 1.);
  std::vector<double> deadband(6, 10.);
  std::vector<double> end_condition_wrench(6, 0.0);
  double filterCutoff = 10.;
  std::vector<bool> endCondition(6, false);
  geometry_msgs::WrenchStamped ftData0;
  double highestAllowableForce = 100.;
  double highestAllowableTorque = 100.;
  compliant_control::CompliantControl control(stiffness, deadband, end_condition_wrench, filterCutoff, ftData0,
                                              highestAllowableForce, highestAllowableTorque);

  std::vector<double> fCond(6, 0.0);
  fCond[0] = 12.0;
  fCond[1] = 10.0;
  fCond[3] = 1e-4;
  fCond[4] = -12.0;

  control.setEndCondition(fCond);

  EXPECT_NEAR(control.end_condition_wrench_[0], 12.0, 1e-4);
  EXPECT_NEAR(control.end_condition_wrench_[1], 10.0, 1e-4);
  EXPECT_NEAR(control.end_condition_wrench_[2], 0.0, 1e-4);
  EXPECT_NEAR(control.end_condition_wrench_[3], 0.0, 1e-4);
  EXPECT_NEAR(control.end_condition_wrench_[4], -12.0, 1e-4);
  EXPECT_NEAR(control.end_condition_wrench_[5], 0.0, 1e-4);
}

TEST(compliantControlTest, getVelocity)
{
  std::vector<double> stiffness(6, 1.);

  std::vector<double> deadband(6, 1.);

  std::vector<double> end_condition_wrench(6, 12.0);

  double filterCutoff = 10.;

  // Input velocity set #1

  geometry_msgs::WrenchStamped ftData0;
  double highestAllowableForce = 100.;
  double highestAllowableTorque = 100.;
  compliant_control::CompliantControl control(stiffness, deadband, end_condition_wrench, filterCutoff, ftData0,
                                              highestAllowableForce, highestAllowableTorque);

  std::vector<double> vIn(6, 0.), vOut(6, 0.);
  vIn[0] = 1.0;
  vIn[1] = 3.0;
  vIn[2] = 10.0;
  vIn[3] = 0.0;
  vIn[4] = 1e-5;
  vIn[5] = 0.9;

  geometry_msgs::WrenchStamped ftData;
  ftData.wrench.force.x = 10.0;
  ftData.wrench.force.y = 10.0;
  ftData.wrench.force.z = 10.0;
  ftData.wrench.torque.x = 10.0;
  ftData.wrench.torque.y = 10.0;
  ftData.wrench.torque.z = 10.0;

  compliantEnum::exitCondition endcondition = control.getVelocity(vIn, ftData, vOut);

  // Output will not be exactly equal to input/stiffness because of the low-pass
  // filtering
  // and the compliant behavior. Should be close, though.
  EXPECT_TRUE(endcondition == compliantEnum::CONDITION_NOT_MET);
  EXPECT_NEAR(vOut[0], vIn[0], 0.2);
  EXPECT_NEAR(vOut[1], vIn[1], 0.2);
  EXPECT_NEAR(vOut[2], vIn[2], 0.2);
  EXPECT_NEAR(vOut[3], vIn[3], 0.2);
  EXPECT_NEAR(vOut[4], vIn[4], 0.2);
  EXPECT_NEAR(vOut[5], vIn[5], 0.2);

  //  To test std_msgs::String length
  char cmd[74];
  sprintf(cmd, "speedl([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 0.2, 0.1)\n", vOut[0], vOut[1], vOut[2], vOut[3],
          vOut[4], vOut[5]);

  //  One of the controlled velocities has met the force/torque end condition so
  //  velocity in
  //  that direction should be zero.
  ftData.wrench.force.y = 50.0;

  // No other applied wrench
  ftData.wrench.force.x = 0.;
  ftData.wrench.force.z = 0.;
  ftData.wrench.torque.x = 0.;
  ftData.wrench.torque.y = 0.;
  ftData.wrench.torque.z = 0.;

  // Spam this several times to allow the filter to settle
  for (int i = 0; i < 20; i++)
    endcondition = control.getVelocity(vIn, ftData, vOut);

  EXPECT_TRUE(endcondition == compliantEnum::CONDITION_MET);
  EXPECT_NEAR(vOut[0], vIn[0], 0.2);
  EXPECT_EQ(vOut[1], 0.0);
  EXPECT_NEAR(vOut[2], vIn[2], 0.2);
  EXPECT_NEAR(vOut[3], vIn[3], 0.2);
  EXPECT_NEAR(vOut[4], vIn[4], 0.2);
  EXPECT_NEAR(vOut[5], vIn[5], 0.2);

  //  One of the controlled element has exceeded the safety limit
  // so velocity should be set to zero.
  ftData.wrench.force.x = 124.0;
  ftData.wrench.force.y = 0.;
  ftData.wrench.force.z = 0.;
  ftData.wrench.torque.x = 0.;
  ftData.wrench.torque.y = 0.;
  ftData.wrench.torque.z = 0.;

  // Spam this several times to allow the filter to settle
  for (int i = 0; i < 20; i++)
    endcondition = control.getVelocity(vIn, ftData, vOut);

  EXPECT_TRUE(endcondition == compliantEnum::FT_VIOLATION);
  EXPECT_EQ(vOut[0], 0.0);
  EXPECT_EQ(vOut[1], 0.0);
  EXPECT_EQ(vOut[2], 0.0);
  EXPECT_EQ(vOut[3], 0.0);
  EXPECT_EQ(vOut[4], 0.0);
  EXPECT_EQ(vOut[5], 0.0);
}
}