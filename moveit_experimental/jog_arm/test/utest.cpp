
#include <boost/thread/thread.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "utest");
  boost::thread ros_thread(boost::bind(&ros::spin));

  int res = RUN_ALL_TESTS();
  ros_thread.interrupt();
  ros_thread.join();

  return res;
}
