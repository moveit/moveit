/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Dale Koenig
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Dale Koenig */

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h>

namespace occupancy_map_monitor
{
class PointcloudUpdaterTester : public testing::Test
{
public:
  void SetUp() override
  {
    tree_.reset(new OccMapTree(0.1));
    updater_.setTree(tree_);
    updater_.initialize();
  }

protected:
  PointCloudOctomapUpdater updater_;
  OccMapTreePtr tree_;

  // Helper to create pointclouds for easy testing
  // Takes vector of form {x_0, y_0, z_0, x_1, y_1, z_1, ..., x_{n-1}, y_{n-1}, z_{n-1}}
  sensor_msgs::PointCloud2::Ptr createPointcloudFromXYZ(std::vector<double> data)
  {
    if (data.size() % 3)
      return nullptr;

    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2());
    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pcd_modifier.resize(data.size() / 3);
    sensor_msgs::PointCloud2Iterator<float> pt_iter(*cloud, "x");
    for (std::size_t i = 0; i < data.size(); i += 3)
    {
      pt_iter[0] = data[i];
      pt_iter[1] = data[i + 1];
      pt_iter[2] = data[i + 2];
      ++pt_iter;
    }
    return cloud;
  }

  bool isOccupied(double x, double y, double z)
  {
    octomap::OcTreeNode* node = tree_->search(x, y, z);
    if (!node)
      return false;
    return tree_->isNodeOccupied(node);
  }
};

TEST_F(PointcloudUpdaterTester, NonIncrementalUpdate)
{
  EXPECT_FALSE(isOccupied(0.95, 0.0, 0.0));
  EXPECT_FALSE(isOccupied(0.0, 0.95, 0.0));

  sensor_msgs::PointCloud2::Ptr cloud = createPointcloudFromXYZ({ 0.95, 0.0, 0.0 });
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::SNAPSHOT));
  EXPECT_TRUE(isOccupied(0.95, 0.0, 0.0));
  EXPECT_FALSE(isOccupied(0.0, 0.95, 0.0));

  cloud = createPointcloudFromXYZ({ 0.0, 0.95, 0.0 });
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::SNAPSHOT));
  EXPECT_FALSE(isOccupied(0.95, 0.0, 0.0));
  EXPECT_TRUE(isOccupied(0.0, 0.95, 0.0));
}

TEST_F(PointcloudUpdaterTester, IncrementalUpdate)
{
  sensor_msgs::PointCloud2::Ptr cloud = createPointcloudFromXYZ({ 0.95, 0.0, 0.0 });
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::INCREMENTAL));
  cloud = createPointcloudFromXYZ({ 0.0, 0.95, 0.0 });
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::INCREMENTAL));

  EXPECT_TRUE(isOccupied(0.0, 0.95, 0.0));
  EXPECT_TRUE(isOccupied(0.95, 0.0, 0.0));
  EXPECT_FALSE(isOccupied(0.0, 0.0, 0.0));
}

TEST_F(PointcloudUpdaterTester, IncrementalRayTracing)
{
  sensor_msgs::PointCloud2::Ptr cloud = createPointcloudFromXYZ({ 0.55, 0.0, 0.0 });
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::INCREMENTAL));
  EXPECT_TRUE(isOccupied(0.55, 0.0, 0.0));

  cloud = createPointcloudFromXYZ({ 0.95, 0.0, 0.0 });
  // We have to process the pointcloud a few times to get the probability odds low enough for the cell to be marked
  // empty
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::INCREMENTAL));
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::INCREMENTAL));
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::INCREMENTAL));
  EXPECT_TRUE(isOccupied(0.95, 0.0, 0.0));
  EXPECT_FALSE(isOccupied(0.55, 0.0, 0.0));
}

TEST_F(PointcloudUpdaterTester, UpdateWithExclusion)
{
  shapes::ShapeConstPtr exclude_shape = std::make_shared<shapes::Box>(0.5, 0.5, 0.5);
  auto exclude_shape_pose = Eigen::Isometry3d(Eigen::Translation3d(1.0, 0.0, 0.0));
  sensor_msgs::PointCloud2::Ptr cloud = createPointcloudFromXYZ({ 0.95, 0.0, 0.0, 1.95, 0.0, 0.0 });

  // If we exclude the shape, a point in the shape shouldn't be occupied
  auto shape_handle = updater_.excludeShape(exclude_shape, exclude_shape_pose);
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::SNAPSHOT));
  EXPECT_TRUE(isOccupied(1.95, 0.0, 0.0));
  EXPECT_FALSE(isOccupied(0.95, 0.0, 0.0));

  // Reinclude the shape and process again, and the point should be occupied
  updater_.forgetShape(shape_handle);
  ASSERT_TRUE(updater_.processCloud(cloud, Eigen::Isometry3d::Identity(), UpdateMethod::SNAPSHOT));
  EXPECT_TRUE(isOccupied(0.95, 0.0, 0.0));
}
}  // namespace occupancy_map_monitor

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pointcloud_octomap_updater_test");

  return RUN_ALL_TESTS();
}