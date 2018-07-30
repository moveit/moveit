#!/usr/bin/env python

import unittest
import numpy as np
import rospy
import rostest
import os

from moveit_ros_planning_interface._moveit_move_group_interface import MoveGroupInterface


class RobotStateUpdateTest(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    @classmethod
    def setUpClass(self):
        self.group = MoveGroupInterface(self.PLANNING_GROUP, "robot_description", rospy.get_namespace())

    @classmethod
    def tearDown(self):
        pass

    def plan(self, target):
        self.group.set_joint_value_target(target)
        return self.group.compute_plan()

    def test(self):
        current = np.asarray(self.group.get_current_joint_values())
        for i in range(30):
            target = current + np.random.uniform(-0.5, 0.5, size = current.shape)
            # if plan was successfully executed, current state should be reported at target
            if self.group.execute(self.plan(target)):
                 actual = np.asarray(self.group.get_current_joint_values())
                 self.assertTrue(np.allclose(target, actual, atol=1e-4, rtol=0.0))
            # otherwise current state should be still the same
            else:
               actual = np.asarray(self.group.get_current_joint_values())
               self.assertTrue(np.allclose(current, actual, atol=1e-4, rtol=0.0))


if __name__ == '__main__':
    PKGNAME = 'moveit_ros_planning_interface'
    NODENAME = 'moveit_test_robot_state_update'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, RobotStateUpdateTest)

    # suppress cleanup segfault in ROS < Kinetic
    os._exit(0)
