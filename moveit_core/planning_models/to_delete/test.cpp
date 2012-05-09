/*
TEST_F(LoadPlanningModelsPr2, SemanticInit)
{
  boost::shared_ptr<planning_models::KinematicModel> kmodel(new planning_models::KinematicModel(urdf_model_, srdf_model_));  
  planning_models::SemanticModel smodel(kmodel, srdf_model_);
  
  EXPECT_TRUE(smodel.isArm("right_arm"));
  EXPECT_TRUE(smodel.hasEndEffector("right_arm"));
  EXPECT_FALSE(smodel.isEndEffector("right_arm"));
  EXPECT_EQ(smodel.getEndEffector("right_arm"), "r_end_effector");
  EXPECT_EQ(smodel.getTipLink("right_arm"), "r_wrist_roll_link");
  EXPECT_EQ(smodel.getBaseLink("right_arm"), "torso_lift_link");
}
