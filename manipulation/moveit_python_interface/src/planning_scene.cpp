#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <boost/function.hpp>
#include <boost/python.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

#include <boost/shared_ptr.hpp>
#include <Python.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

namespace bp = boost::python;

namespace moveit_python_interface

{
void addSimpleObjectToPlanningScene(std::string id, std::string frame_id, int type, double pos_x, double pos_y, double pos_z, double or_x, double or_y, double or_z, double or_w, std::vector<double> dimensions)
{
  ros::Publisher object_in_map_pub;
  ros::NodeHandle nh;

  //ros::Publisher object_in_map_pub_;
  object_in_map_pub  = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  ros::Duration(2.0).sleep();// This delay is so critical, otherwise the first published object may not be added in the collision_space by the environment_server

  //add the cylinder into the collision space
  moveit_msgs::CollisionObject cylinder_object;
  cylinder_object.id = id;
  cylinder_object.operation = moveit_msgs::CollisionObject::ADD;
  cylinder_object.header.frame_id = frame_id;
  cylinder_object.header.stamp = ros::Time::now();
  shape_msgs::SolidPrimitive object;
  object.type = type;
  object.dimensions.resize(dimensions.size());
  for (int i=0; i < dimensions.size(); i++)
  {
    object.dimensions[i] = dimensions.at(i);
  }

  //object.dimensions = dimensions;
  cylinder_object.primitives.push_back(object);

  geometry_msgs::Pose pose;
  pose.position.x = pos_x;
  pose.position.y = pos_y;
  pose.position.z = pos_z;
  pose.orientation.x = or_x;
  pose.orientation.y = or_y;
  pose.orientation.z = or_z;
  pose.orientation.w = or_w;
  cylinder_object.primitive_poses.push_back(pose);
  object_in_map_pub.publish(cylinder_object);

  ROS_INFO("Should have published");
}

std::vector<double> doubleFromList(bp::list &values)
{
  int l = bp::len(values);
  std::vector<double> v(l);
  for (int i = 0; i < l ; ++i)
    v[i] = bp::extract<double>(values[i]);
  return v;
}

void addSimpleObjectToPlanningScenePython(std::string id, std::string frame_id, int type, double pos_x, double pos_y, double pos_z, double or_x, double or_y, double or_z, double or_w, bp::list &values)
{
    addSimpleObjectToPlanningScene(id, frame_id, type, pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w, doubleFromList(values));
}

void removeSimpleObjectFromPlanningScene(std::string id, std::string frame_id) 
{
  ros::Publisher object_in_map_pub;
  ros::NodeHandle nh;

  //ros::Publisher object_in_map_pub_;
  object_in_map_pub  = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  ros::Duration(2.0).sleep();

  moveit_msgs::CollisionObject object_to_remove;
  object_to_remove.id = id;
  object_to_remove.operation = moveit_msgs::CollisionObject::REMOVE;
  object_to_remove.header.frame_id = frame_id;
  object_to_remove.header.stamp = ros::Time::now();

  object_in_map_pub.publish(object_to_remove);

  ROS_INFO("Should have published");
}

void attachSimpleObjectToGripper(std::string id, std::string frame_id, int type, std::string link_name, std::vector<std::string> touch_links, double pos_x, double pos_y, double pos_z, double or_x, double or_y, double or_z, double or_w, std::vector<double> dimensions)
{
  ros::Publisher object_in_map_pub;
  ros::NodeHandle nh;

  //ros::Publisher object_in_map_pub_;
  object_in_map_pub  = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::Duration(2.0).sleep();// This delay is so critical, otherwise the first published object may not be added in the collision_space by the environment_server

  //add the cylinder into the collision space
  moveit_msgs::AttachedCollisionObject gripper_object;
  gripper_object.object.id = id;
  gripper_object.link_name = link_name;

  /*
  gripper_object.touch_links.push_back("low_cost_gripper_r_finger_link");
  gripper_object.touch_links.push_back("low_cost_gripper_l_finger_link");
  gripper_object.touch_links.push_back("low_cost_gripper_l_finger_tip_link");
  gripper_object.touch_links.push_back("low_cost_gripper_palm_link");
  gripper_object.touch_links.push_back("low_cost_gripper_r_finger_tip_link");
  gripper_object.touch_links.push_back("wrist_roll_link");
  gripper_object.touch_links.push_back("low_cost_gripper_l_finger_tip_frame");
  gripper_object.touch_links.push_back("low_cost_gripper_motor_screw_link");
  gripper_object.touch_links.push_back("low_cost_gripper_motor_slider_link");
  gripper_object.touch_links.push_back("low_cost_gripper_tool_frame");
  */

  for (int i=0; i < touch_links.size(); i++)
  {
    gripper_object.touch_links.push_back(touch_links.at(i));    
  }

  gripper_object.object.operation = moveit_msgs::CollisionObject::ADD;
  gripper_object.object.header.frame_id = frame_id;
  gripper_object.object.header.stamp = ros::Time::now();
  shape_msgs::SolidPrimitive object;
  object.type = type;
  object.dimensions.resize(dimensions.size());
  for (int i=0; i < dimensions.size(); i++)
  {
    object.dimensions[i] = dimensions.at(i);
  }

  //object.dimensions = dimensions;
  gripper_object.object.primitives.push_back(object);

  geometry_msgs::Pose pose;
  pose.position.x = pos_x;
  pose.position.y = pos_y;
  pose.position.z = pos_z;
  pose.orientation.x = or_x;
  pose.orientation.y = or_y;
  pose.orientation.z = or_z;
  pose.orientation.w = or_w;
  gripper_object.object.primitive_poses.push_back(pose);
  object_in_map_pub.publish(gripper_object);

  ROS_INFO("Should have published");


}

std::vector<std::string> stringFromList(bp::list &values)
{
  int l = bp::len(values);
  std::vector<std::string> v(l);
  for (int i = 0; i < l ; ++i)
    v[i] = bp::extract<std::string>(values[i]);
  return v;
}

void attachSimpleObjectToGripperPython(std::string id, std::string frame_id, int type, std::string link_name, bp::list &touch_links, double pos_x, double pos_y, double pos_z, double or_x, double or_y, double or_z, double or_w, bp::list &values)
{
  attachSimpleObjectToGripper(id, frame_id, type, link_name, stringFromList(touch_links), pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w, doubleFromList(values));
}

void removeSimpleAttachedObject(std::string id, std::string frame_id, std::string link_name)
{
  ros::Publisher object_in_map_pub;
  ros::NodeHandle nh;

  //ros::Publisher object_in_map_pub_;
  object_in_map_pub  = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::Duration(2.0).sleep();

  moveit_msgs::AttachedCollisionObject object_to_remove;
  object_to_remove.link_name = link_name;
  object_to_remove.object.id = id;
  object_to_remove.object.operation = moveit_msgs::CollisionObject::REMOVE;
  object_to_remove.object.header.frame_id = frame_id;
  object_to_remove.object.header.stamp = ros::Time::now();

  object_in_map_pub.publish(object_to_remove);

  ROS_INFO("Should have published");
}


}

BOOST_PYTHON_MODULE(_planning_scene)
{
  using namespace moveit_python_interface;

  bp::def("add_simple_object", addSimpleObjectToPlanningScenePython);
  bp::def("remove_simple_object", removeSimpleObjectFromPlanningScene);
  bp::def("attach_simple_object_to_gripper", attachSimpleObjectToGripperPython);
  bp::def("remove_simple_attached_object", removeSimpleAttachedObject);
}

