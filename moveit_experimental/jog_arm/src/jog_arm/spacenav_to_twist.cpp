#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

namespace to_twist {
class spaceNavToTwist {
public:
  spaceNavToTwist() : spinner_(2) {
    joy_sub_ =
        n_.subscribe("spacenav/joy", 1, &spaceNavToTwist::joyCallback, this);
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>(
        "jog_arm_server/delta_jog_cmds", 1);

    spinner_.start();
    ros::waitForShutdown();
  };

private:
  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;
  ros::AsyncSpinner spinner_;

  // Convert incoming joy commands to TwistStamped commands for jogging
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
    geometry_msgs::TwistStamped t_s;
    t_s.header.stamp = ros::Time::now();
    t_s.twist.linear.x = msg->axes[0];
    t_s.twist.linear.y = msg->axes[1];
    t_s.twist.linear.z = msg->axes[2];

    t_s.twist.angular.x = msg->axes[3];
    t_s.twist.angular.y = msg->axes[4];
    t_s.twist.angular.z = msg->axes[5];

    twist_pub_.publish(t_s);
  }
};
} // end to_twist namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "spacenav_to_twist");

  to_twist::spaceNavToTwist to_twist;

  return 0;
}