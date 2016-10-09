#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinematic_plugin_tests");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Loading URDF");
    {
        robot_model_loader::RobotModelLoader rml("robot_description", true);
    }
    {
        robot_model_loader::RobotModelLoader rml("robot_description", true);
    }
    return 0;
}
