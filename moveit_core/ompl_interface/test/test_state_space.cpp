/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/** Author Ioan Sucan */

#include "ompl_interface/parameterization/model_based_state_space.h"
#include <ompl/util/Exception.h>
#include <planning_models/conversions.h>
#include <gtest/gtest.h>
#include <fstream>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  
  virtual void SetUp()
  {
    urdf_model_.reset(new urdf::Model());
    srdf_model_.reset(new srdf::Model());
    urdf_ok_ = urdf_model_->initFile("../planning_models/test/urdf/robot.xml");
    srdf_ok_ = srdf_model_->initFile(*urdf_model_, "../planning_models/test/srdf/robot.xml");
    if (urdf_ok_ && srdf_ok_)
      kmodel_.reset(new planning_models::KinematicModel(urdf_model_, srdf_model_));
  };
  
  virtual void TearDown()
  {
  }
  
protected:
  planning_models::KinematicModelPtr kmodel_;
  boost::shared_ptr<urdf::Model>     urdf_model_;
  boost::shared_ptr<srdf::Model>     srdf_model_;
  bool                               urdf_ok_;
  bool                               srdf_ok_;
  
};

TEST_F(LoadPlanningModelsPr2, StateSpace)
{
  ompl_interface::ModelBasedStateSpaceSpecification spec(kmodel_, "whole_body");
  ompl_interface::ModelBasedStateSpace ss(spec);
  ss.setBounds(-1, 1, -1, 1, -1, 1);
  ss.setup();
  std::ofstream fout("ompl_interface_test_state_space_diagram1.dot");
  ss.diagram(fout);
  bool passed = false;
  try
  {
    ss.sanityChecks();
    passed = true;
  }
  catch(ompl::Exception &ex)
  {
    ROS_ERROR_STREAM("Sanity checks did not pass: " << ex.what());
  }
  EXPECT_TRUE(passed);
}

TEST_F(LoadPlanningModelsPr2, StateSpaces)
{
  ompl_interface::ModelBasedStateSpaceSpecification spec1(kmodel_, "right_arm");
  ompl_interface::ModelBasedStateSpace ss1(spec1);
  ss1.setup();

  ompl_interface::ModelBasedStateSpaceSpecification spec2(kmodel_, "left_arm");
  ompl_interface::ModelBasedStateSpace ss2(spec2);
  ss2.setup();

  ompl_interface::ModelBasedStateSpaceSpecification spec3(kmodel_, "whole_body");
  ompl_interface::ModelBasedStateSpace ss3(spec3);
  ss3.setup();

  ompl_interface::ModelBasedStateSpaceSpecification spec4(kmodel_, "arms");
  ompl_interface::ModelBasedStateSpace ss4(spec4);
  ss4.setup();
  
  std::ofstream fout("ompl_interface_test_state_space_diagram2.dot");
  ompl::base::StateSpace::Diagram(fout);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
