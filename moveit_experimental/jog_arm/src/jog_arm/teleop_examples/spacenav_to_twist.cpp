#include "geometry_msgs/TwistStamped.h"
#include "control_msgs/JointJog.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

namespace jog_arm
{
static const int NUM_SPINNERS = 1;
static const int QUEUE_LENGTH = 1;

class SpaceNavToTwist
{
public:
  SpaceNavToTwist() : spinner_(NUM_SPINNERS)
  {
    joy_sub_ = n_.subscribe("spacenav/joy", QUEUE_LENGTH, &SpaceNavToTwist::joyCallback, this);
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("jog_arm_server/delta_jog_cmds", QUEUE_LENGTH);
    joint_delta_pub_ = n_.advertise<control_msgs::JointJog>("jog_arm_server/joint_delta_jog_cmds", QUEUE_LENGTH);

    spinner_.start();
    ros::waitForShutdown();
  };

private:
  // Convert incoming joy commands to TwistStamped commands for jogging.
  // The TwistStamped component goes to jogging, while buttons 0 & 1 control
  // joints directly.
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Cartesian jogging with the axes
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = msg->axes[0];
    twist.twist.linear.y = msg->axes[1];
    twist.twist.linear.z = msg->axes[2];

    twist.twist.angular.x = msg->axes[3];
    twist.twist.angular.y = msg->axes[4];
    twist.twist.angular.z = msg->axes[5];

    // Joint jogging with the buttons
    control_msgs::JointJog joint_deltas;
    // This example is for a UR5.
    joint_deltas.joint_names.push_back("shoulder_pan_joint");

    // Button 0: positive on the wrist joint
    // Button 1: negative on the wrist joint
    joint_deltas.velocities.push_back(msg->buttons[0] - msg->buttons[1]);
    joint_deltas.header.stamp = ros::Time::now();

    twist_pub_.publish(twist);
    joint_delta_pub_.publish(joint_deltas);
  }

  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_, joint_delta_pub_;
  ros::AsyncSpinner spinner_;
};
}  // namespace jog_arm

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spacenav_to_twist");

  jog_arm::SpaceNavToTwist to_twist;

  return 0;
}
