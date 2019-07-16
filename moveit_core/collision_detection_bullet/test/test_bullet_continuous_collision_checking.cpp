#include <gtest/gtest.h>
#include <ros/ros.h>

#include <moveit/collision_detection_bullet/bullet_integration/bullet_cast_bvh_manager.h>
#include <moveit/collision_detection_bullet/bullet_integration/bullet_discrete_bvh_manager.h>
#include <moveit/collision_detection/collision_common.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit/collision_detection_bullet/collision_world_bullet.h>
#include <moveit/collision_detection_bullet/collision_robot_bullet.h>
#include <moveit/collision_detection_bullet/bullet_integration/basic_types.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>


/** \brief Brings the panda robot in user defined home position */
inline void setToHome(robot_state::RobotState& panda_state)
{
  panda_state.setToDefaultValues();
  double joint2 = -0.785;
  double joint4 = -2.356;
  double joint6 = 1.571;
  double joint7 = 0.785;
  panda_state.setJointPositions("panda_joint2", &joint2);
  panda_state.setJointPositions("panda_joint4", &joint4);
  panda_state.setJointPositions("panda_joint6", &joint6);
  panda_state.setJointPositions("panda_joint7", &joint7);
  panda_state.update();
}

class BulletCollisionDetectionTester : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    robot_model_ok_ = static_cast<bool>(robot_model_);

    acm_.reset(new collision_detection::AllowedCollisionMatrix(robot_model_->getLinkModelNames(), false));

    acm_->setEntry("panda_link0", "panda_link1", true);
    acm_->setEntry("panda_link1", "panda_link2", true);
    acm_->setEntry("panda_link2", "panda_link3", true);
    acm_->setEntry("panda_link3", "panda_link4", true);
    acm_->setEntry("panda_link4", "panda_link5", true);
    acm_->setEntry("panda_link5", "panda_link6", true);
    acm_->setEntry("panda_link6", "panda_link7", true);
    acm_->setEntry("panda_link7", "panda_hand", true);
    acm_->setEntry("panda_hand", "panda_rightfinger", true);
    acm_->setEntry("panda_hand", "panda_leftfinger", true);
    acm_->setEntry("panda_rightfinger", "panda_leftfinger", true);
    acm_->setEntry("panda_link5", "panda_link7", true);
    acm_->setEntry("panda_link6", "panda_hand", true);

    crobot_.reset(new collision_detection::CollisionRobotBullet(robot_model_));
    cworld_.reset(new collision_detection::CollisionWorldBullet());

    robot_state_.reset(new robot_state::RobotState(robot_model_));

    setToHome(*robot_state_);
  }

  void TearDown() override
  {
  }

protected:
  bool robot_model_ok_;

  robot_model::RobotModelPtr robot_model_;

  collision_detection::CollisionWorldPtr cworld_;
  collision_detection::CollisionRobotPtr crobot_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  robot_state::RobotStatePtr robot_state_;
};

void addCollisionObjects(collision_detection_bullet::BulletCastBVHManager& checker)
{
  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  shapes::ShapePtr static_box(new shapes::Box(1, 1, 1));
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  collision_detection_bullet::AlignedVector<Eigen::Isometry3d> obj1_poses;
  std::vector<collision_detection_bullet::CollisionObjectType> obj1_types;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(collision_detection_bullet::CollisionObjectType::USE_SHAPE_TYPE);

  checker.addCollisionObject("static_box_link", collision_detection::BodyType::WORLD_OBJECT, obj1_shapes, obj1_poses,
                             obj1_types);

  ////////////////////////////
  // Add moving box to checker
  ////////////////////////////
  shapes::ShapePtr moving_box(new shapes::Box(0.2, 0.2, 0.2));
  Eigen::Isometry3d moving_box_pose;

  moving_box_pose.setIdentity();
  moving_box_pose.translation() = Eigen::Vector3d(0.5, -0.5, 0);

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  collision_detection_bullet::AlignedVector<Eigen::Isometry3d> obj2_poses;
  std::vector<collision_detection_bullet::CollisionObjectType> obj2_types;
  obj2_shapes.push_back(moving_box);
  obj2_poses.push_back(moving_box_pose);
  obj2_types.push_back(collision_detection_bullet::CollisionObjectType::USE_SHAPE_TYPE);

  checker.addCollisionObject("moving_box_link", collision_detection::BodyType::WORLD_OBJECT, obj2_shapes, obj2_poses,
                             obj2_types);
}

void addCollisionObjectsMesh(collision_detection_bullet::BulletCastBVHManager& checker)
{
  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  shapes::ShapePtr static_box(new shapes::Box(0.3, 0.3, 0.3));
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  collision_detection_bullet::AlignedVector<Eigen::Isometry3d> obj1_poses;
  std::vector<collision_detection_bullet::CollisionObjectType> obj1_types;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(collision_detection_bullet::CollisionObjectType::USE_SHAPE_TYPE);

  checker.addCollisionObject("static_box_link", collision_detection::BodyType::WORLD_OBJECT, obj1_shapes, obj1_poses,
                             obj1_types);

  ////////////////////////////
  // Add moving mesh to checker
  ////////////////////////////

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  collision_detection_bullet::AlignedVector<Eigen::Isometry3d> obj2_poses;
  std::vector<collision_detection_bullet::CollisionObjectType> obj2_types;

  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(collision_detection_bullet::CollisionObjectType::USE_SHAPE_TYPE);

  Eigen::Isometry3d s_pose;
  s_pose.setIdentity();

  std::string kinect = "package://moveit_resources/panda_description/meshes/collision/hand.stl";
  shapes::ShapeConstPtr s;
  s.reset(shapes::createMeshFromResource(kinect));
  obj2_shapes.push_back(s);
  obj2_types.push_back(collision_detection_bullet::CollisionObjectType::CONVEX_HULL);
  obj2_poses.push_back(s_pose);

  checker.addCollisionObject("moving_box_link", collision_detection::BodyType::WORLD_OBJECT, obj2_shapes, obj2_poses,
                             obj2_types);
}

void runTest(collision_detection_bullet::BulletCastBVHManager& checker, collision_detection::CollisionResult& result,
             std::vector<collision_detection::Contact>& result_vector, Eigen::Isometry3d& start_pos,
             Eigen::Isometry3d& end_pos)
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
  request.contacts = true;
  // collision_detection_bullet::ContactResultMap result;
  checker.contactTest(result, request, nullptr);

  for (const auto& contacts_all : result.contacts)
  {
    for (const auto& contact : contacts_all.second)
    {
      result_vector.push_back(contact);
    }
  }
}

// TODO: Add continuous to continuous collision checking
/** \brief Continuous self collision checks are not supported yet by the bullet integration */
TEST_F(BulletCollisionDetectionTester, DISABLED_ContinuousCollisionSelf)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  robot_state::RobotState state1(robot_model_);
  robot_state::RobotState state2(robot_model_);

  setToHome(state1);
  double joint2 = 0.15;
  double joint4 = -3.0;
  double joint5 = -0.8;
  double joint7 = -0.785;
  state1.setJointPositions("panda_joint2", &joint2);
  state1.setJointPositions("panda_joint4", &joint4);
  state1.setJointPositions("panda_joint5", &joint5);
  state1.setJointPositions("panda_joint7", &joint7);
  state1.update();

  crobot_->checkSelfCollision(req, res, state1, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  setToHome(state2);
  double joint_5_other = 0.8;
  state2.setJointPositions("panda_joint2", &joint2);
  state2.setJointPositions("panda_joint4", &joint4);
  state2.setJointPositions("panda_joint5", &joint_5_other);
  state2.setJointPositions("panda_joint7", &joint7);
  state2.update();

  crobot_->checkSelfCollision(req, res, state2, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  crobot_->checkSelfCollision(req, res, state1, state2, *acm_);
  ROS_INFO_STREAM("Continous to continous collisions are not supported yet, therefore fail here.");
  ASSERT_TRUE(res.collision);
  res.clear();
}

/** \brief Two similar robot poses are used as start and end pose of a continuous collision check. */
TEST_F(BulletCollisionDetectionTester, ContinuousCollisionWorld)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 10;
  collision_detection::CollisionResult res;

  robot_state::RobotState state1(robot_model_);
  robot_state::RobotState state2(robot_model_);

  setToHome(state1);
  state1.update();

  setToHome(state2);
  double joint_2{ 0.05 };
  double joint_4{ -1.6 };
  state2.setJointPositions("panda_joint2", &joint_2);
  state2.setJointPositions("panda_joint4", &joint_4);
  state2.update();

  cworld_->checkRobotCollision(req, res, *crobot_, state1, state2, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  // Adding the box which is not in collision with the individual states but with the casted one.
  shapes::Shape* shape = new shapes::Box(0.1, 0.1, 0.1);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };
  pos.translation().x() = 0.43;
  pos.translation().y() = 0;
  pos.translation().z() = 0.55;
  cworld_->getWorld()->addToObject("box", shape_ptr, pos);

  cworld_->checkRobotCollision(req, res, *crobot_, state1, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  cworld_->checkRobotCollision(req, res, *crobot_, state2, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  cworld_->checkRobotCollision(req, res, *crobot_, state1, state2, *acm_);
  ASSERT_TRUE(res.collision);
  ASSERT_EQ(res.contact_count, 4u);
  res.clear();
}

TEST(ContinuousCollisionUnit, BulletCastBVHCollisionBoxBoxUnit)
{
  collision_detection::CollisionResult result;
  std::vector<collision_detection::Contact> result_vector;

  Eigen::Isometry3d start_pos, end_pos;
  start_pos.setIdentity();
  start_pos.translation().x() = -2;
  end_pos.setIdentity();
  end_pos.translation().x() = 2;

  collision_detection_bullet::BulletCastBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker, result, result_vector, start_pos, end_pos);

  ASSERT_TRUE(result.collision);
  EXPECT_NEAR(result_vector[0].depth, -0.6, 0.001);
  EXPECT_NEAR(result_vector[0].percent_interpolation, 0.6, 0.001);
}

TEST(ContinuousCollisionUnit, BulletCastMeshVsBox)
{
  collision_detection_bullet::BulletCastBVHManager checker;
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

TEST(ContinuousCollisionUnit, TwoManagers)
{
  collision_detection_bullet::BulletCastBVHManager checker_continuous;
  collision_detection_bullet::BulletDiscreteBVHManager checker_discrete;

  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  shapes::ShapePtr static_box(new shapes::Box(.1, .1, .1));
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  collision_detection_bullet::AlignedVector<Eigen::Isometry3d> obj1_poses;
  std::vector<collision_detection_bullet::CollisionObjectType> obj1_types;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(collision_detection_bullet::CollisionObjectType::USE_SHAPE_TYPE);

  checker_continuous.addCollisionObject("static_box_link", collision_detection::BodyType::WORLD_OBJECT, obj1_shapes,
                                        obj1_poses, obj1_types);
  checker_discrete.addCollisionObject("static_box_link", collision_detection::BodyType::WORLD_OBJECT, obj1_shapes,
                                      obj1_poses, obj1_types);

  ////////////////////////
  // Add moving sphere to checker
  ////////////////////////
  shapes::ShapePtr sphere;
  sphere.reset(
      shapes::createMeshFromResource("package://moveit_resources/panda_description/meshes/collision/link0.stl"));

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  collision_detection_bullet::AlignedVector<Eigen::Isometry3d> obj2_poses;
  std::vector<collision_detection_bullet::CollisionObjectType> obj2_types;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(sphere_pose);
  obj2_types.push_back(collision_detection_bullet::CollisionObjectType::CONVEX_HULL);

  checker_continuous.addCollisionObject("moving_box_link", collision_detection::BodyType::WORLD_OBJECT, obj2_shapes,
                                        obj2_poses, obj2_types);
  checker_discrete.addCollisionObject("moving_box_link", collision_detection::BodyType::WORLD_OBJECT, obj2_shapes,
                                      obj2_poses, obj2_types);

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
  checker_discrete.contactTest(result_2, request, nullptr);
  checker_continuous.contactTest(result, request, nullptr);

  ASSERT_TRUE(result.collision);
  ASSERT_FALSE(result_2.collision);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
