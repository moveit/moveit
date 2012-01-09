/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/** \author Ioan Sucan */

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <gtest/gtest.h>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:

    virtual void SetUp()
    {
        urdf_model_.reset(new urdf::Model());
        srdf_model_.reset(new srdf::Model());
        urdf_ok_ = urdf_model_->initFile("test/urdf/robot.xml");
        srdf_ok_ = srdf_model_->initFile(*urdf_model_, "test/srdf/robot.xml");
    };

    virtual void TearDown()
    {
    }

protected:

    boost::shared_ptr<urdf::Model> urdf_model_;
    boost::shared_ptr<srdf::Model> srdf_model_;
    bool                           urdf_ok_;
    bool                           srdf_ok_;

};

TEST_F(LoadPlanningModelsPr2, InitOK)
{
    ASSERT_TRUE(urdf_ok_);
    ASSERT_EQ(urdf_model_->getName(), "pr2_test");

    planning_models::KinematicModelPtr kmodel(new planning_models::KinematicModel(urdf_model_, srdf_model_));
    planning_models::KinematicState ks(kmodel);
    ks.setToRandomValues();
    ks.setToDefaultValues();


    planning_models::Transforms tf(kmodel->getModelFrame());

    btTransform t1;
    t1.setIdentity();
    t1.setOrigin(btVector3(10.0, 1.0, 0.0));
    tf.setTransform(t1, "some_frame_1");

    btTransform t2;
    t2 = btTransform(btQuaternion(btVector3(0.0, 1.0, 0.0), 0.5), btVector3(10.0, 1.0, 0.0));
    tf.setTransform(t2, "some_frame_2");

    btTransform t3;
    t3.setIdentity();
    t3.setOrigin(btVector3(0.0, 1.0, -1.0));
    tf.setTransform(t3, "some_frame_3");


    EXPECT_TRUE(tf.isFixedFrame("some_frame_1"));
    EXPECT_FALSE(tf.isFixedFrame("base_footprint"));
    EXPECT_TRUE(tf.isFixedFrame(kmodel->getModelFrame()));

    btTransform x;
    x.setIdentity();
    tf.transformPose(ks, "some_frame_2", x, x);
    EXPECT_TRUE(x == t2);

    tf.transformPose(ks, kmodel->getModelFrame(), x, x);
    EXPECT_TRUE(x == t2);

    x.setIdentity();
    tf.transformPose(ks, "r_wrist_roll_link", x, x);

    EXPECT_NEAR(x.getOrigin().x(), 0.585315, 1e-4);
    EXPECT_NEAR(x.getOrigin().y(), -0.188, 1e-4);
    EXPECT_NEAR(x.getOrigin().z(), 1.24001, 1e-4);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
