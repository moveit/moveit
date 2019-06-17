#include <moveit/collision_detection_bullet/tesseract/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <ros/ros.h>
TESSERACT_IGNORE_WARNINGS_POP

#include <moveit/collision_detection_bullet/tesseract/bullet_cast_bvh_manager.h>
#include <moveit/collision_detection_bullet/tesseract/bullet_discrete_bvh_manager.h>
#include <moveit/collision_detection/collision_common.h>

void addCollisionObjects(tesseract::ContinuousContactManagerBase& checker)
{
  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  shapes::ShapePtr static_box(new shapes::Box(1, 1, 1));
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("static_box_link", 0, obj1_shapes, obj1_poses, obj1_types);

  ////////////////////////////
  // Add moving box to checker
  ////////////////////////////
  shapes::ShapePtr moving_box(new shapes::Box(0.25, 0.25, 0.25));
  Eigen::Isometry3d moving_box_pose;

  moving_box_pose.setIdentity();
  moving_box_pose.translation() = Eigen::Vector3d(0.5, -0.5, 0);

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  tesseract::VectorIsometry3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(moving_box);
  obj2_poses.push_back(moving_box_pose);
  obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("moving_box_link", 0, obj2_shapes, obj2_poses, obj2_types);
}

void addCollisionObjectsMesh(tesseract::ContinuousContactManagerBase& checker)
{
  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  shapes::ShapePtr static_box(new shapes::Box(0.3, 0.3, 0.3));
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("static_box_link", 0, obj1_shapes, obj1_poses, obj1_types);

  ////////////////////////////
  // Add moving mesh to checker
  ////////////////////////////

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  tesseract::VectorIsometry3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;

  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  Eigen::Isometry3d s_pose;
  s_pose.setIdentity();

  std::string kinect = "package://moveit_resources/panda_description/meshes/collision/hand.stl";
  shapes::ShapeConstPtr s;
  s.reset(shapes::createMeshFromResource(kinect));
  obj2_shapes.push_back(s);
  obj2_types.push_back(tesseract::CollisionObjectType::ConvexHull);
  obj2_poses.push_back(s_pose);

  checker.addCollisionObject("moving_box_link", 0, obj2_shapes, obj2_poses, obj2_types);
}

void runTest(tesseract::ContinuousContactManagerBase& checker, collision_detection::CollisionResult& result,
    std::vector<collision_detection::Contact>& result_vector, Eigen::Isometry3d& start_pos, Eigen::Isometry3d& end_pos)
{
  //////////////////////////////////////
  // Test when object is inside another
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "moving_box_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  checker.setCollisionObjectsTransform("static_box_link", Eigen::Isometry3d::Identity());

  checker.setCollisionObjectsTransform("moving_box_link", start_pos, end_pos);

  // Perform collision check
  collision_detection::CollisionRequest request;
  //tesseract::ContactResultMap result;
  checker.contactTest(result, tesseract::ContactTestType::CLOSEST, request);

  for (auto const contacts_all : result.contacts)
  {
    for (auto const& contact : contacts_all.second)
    {
      result_vector.push_back(contact);
    }
  }
}

TEST(TesseractCollisionUnit, BulletCastBVHCollisionBoxBoxUnit)
{
  collision_detection::CollisionResult result;
  std::vector<collision_detection::Contact> result_vector;

  Eigen::Isometry3d start_pos, end_pos;
  start_pos.setIdentity();
  start_pos.translation().x() = -1.9;
  end_pos.setIdentity();
  end_pos.translation().x() = 1.9;
  end_pos.translation().y() = 3.8;

  tesseract::tesseract_bullet::BulletCastBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker, result, result_vector, start_pos, end_pos);

  ASSERT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].depth, -0.2475, 0.001);
  EXPECT_NEAR(result_vector[0].cc_time, 0.25, 0.001);
  EXPECT_TRUE(result_vector[0].cc_type == collision_detection::ContinouseCollisionTypes::CCType_Between);

  EXPECT_NEAR(result_vector[0].nearest_points[0][0], -0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], 0.0, 0.001);

  EXPECT_NEAR(result_vector[0].nearest_points[1][0], -1.275, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[1][1], -0.625, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[1][2], 0.0, 0.001);

  EXPECT_NEAR(result_vector[0].cc_nearest_points[0][0], -0.325, 0.001);
  EXPECT_NEAR(result_vector[0].cc_nearest_points[0][1], 0.325, 0.001);
  EXPECT_NEAR(result_vector[0].cc_nearest_points[0][2], 0.0, 0.001);

  EXPECT_NEAR(result_vector[0].cc_nearest_points[1][0], 2.525, 0.001);
  EXPECT_NEAR(result_vector[0].cc_nearest_points[1][1], 3.175, 0.001);
  EXPECT_NEAR(result_vector[0].cc_nearest_points[1][2], 0.0, 0.001);
}

TEST(TesseractCollisionUnit, BulletCastMeshVsBox)
{
  tesseract::tesseract_bullet::BulletCastBVHManager checker;
  addCollisionObjectsMesh(checker);

  Eigen::Isometry3d start_pos, end_pos;
  start_pos.setIdentity();
  start_pos.translation().x() = -1.9;
  end_pos.setIdentity();
  end_pos.translation().x() = 1.9;

  collision_detection::CollisionResult result;
  std::vector<collision_detection::Contact> result_vector;

  runTest(checker, result, result_vector, start_pos, end_pos);

  ASSERT_TRUE(result.collision);
}

TEST(TesseractCollisionUnit, TwoManagers)
{
  tesseract::tesseract_bullet::BulletCastBVHManager checker_continuous;
  tesseract::tesseract_bullet::BulletDiscreteBVHManager checker_discrete;

  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  shapes::ShapePtr static_box(new shapes::Box(.1, .1, .1));
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker_continuous.addCollisionObject("static_box_link", 0, obj1_shapes, obj1_poses, obj1_types);
  checker_discrete.addCollisionObject("static_box_link", 0, obj1_shapes, obj1_poses, obj1_types);

  ////////////////////////
  // Add moving sphere to checker
  ////////////////////////
  shapes::ShapePtr sphere;
  sphere.reset(shapes::createMeshFromResource("package://moveit_resources/clutter/sphere_p25m.stl"));

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  tesseract::VectorIsometry3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(sphere_pose);
  obj2_types.push_back(tesseract::CollisionObjectType::ConvexHull);

  checker_continuous.addCollisionObject("moving_box_link", 0, obj2_shapes, obj2_poses, obj2_types);
  checker_discrete.addCollisionObject("moving_box_link", 0, obj2_shapes, obj2_poses, obj2_types);

  checker_continuous.setActiveCollisionObjects({ "moving_box_link" });
  checker_continuous.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  checker_continuous.setCollisionObjectsTransform("static_box_link", Eigen::Isometry3d::Identity());
  checker_discrete.setCollisionObjectsTransform("static_box_link", Eigen::Isometry3d::Identity());

  Eigen::Isometry3d start_pos, end_pos;
  start_pos.setIdentity();
  start_pos.translation().x() = -1.9;
  end_pos.setIdentity();
  end_pos.translation().x() = 1.9;
  checker_continuous.setCollisionObjectsTransform("moving_box_link", start_pos, end_pos);

  Eigen::Isometry3d start_pos_2;
  start_pos_2.setIdentity();
  start_pos_2.translation().x() = -1.9;
  checker_discrete.setCollisionObjectsTransform("moving_box_link", start_pos_2);

  collision_detection::CollisionResult result;
  collision_detection::CollisionResult result_2;
  collision_detection::CollisionRequest request;

  // Perform collision check
  checker_discrete.contactTest(result_2, tesseract::ContactTestType::CLOSEST, request);
  checker_continuous.contactTest(result, tesseract::ContactTestType::CLOSEST, request);

  ASSERT_TRUE(result.collision);
  ASSERT_FALSE(result_2.collision);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
