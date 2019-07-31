#!/usr/bin/env python

import unittest
import numpy as np
import rospy
import rostest
import os

from moveit_ros_planning_interface._moveit_moveit_cpp_interface import MoveItCpp
from moveit_msgs.msg import MoveItErrorCodes


class PythonMoveItCppTest(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    @classmethod
    def setUpClass(self):
        self.group = MoveItCpp(self.PLANNING_GROUP, "robot_description", rospy.get_namespace())

    @classmethod
    def tearDown(self):
        pass

    def check_target_setting(self, expect, *args):
        if len(args) == 0:
            args = [expect]
        self.group.set_joint_value_target(*args)
        res = self.group.get_joint_value_target()
        self.assertTrue(np.all(np.asarray(res) == np.asarray(expect)),
                        "Setting failed for %s, values: %s" % (type(args[0]), res))

    def test_target_setting(self):
        n = self.group.get_variable_count()
        self.check_target_setting([0.1] * n)
        self.check_target_setting((0.2,) * n)
        self.check_target_setting(np.zeros(n))
        self.check_target_setting([0.3] * n, {name: 0.3 for name in self.group.get_active_joints()})
        self.check_target_setting([0.5] + [0.3]*(n-1), "joint_1", 0.5)

    def plan(self, target):
        self.group.set_joint_value_target(target)
        return self.group.plan()

    def test_validation(self):
        current = np.asarray(self.group.get_current_joint_values())

        error_code1, plan1, time = self.plan(current + 0.2)
        error_code2, plan2, time = self.plan(current + 0.2)

        # both plans should have succeeded:
        error_code = MoveItErrorCodes()
        error_code.deserialize(error_code1)
        self.assertEqual(error_code.val, MoveItErrorCodes.SUCCESS)
        error_code = MoveItErrorCodes()
        error_code.deserialize(error_code2)
        self.assertEqual(error_code.val, MoveItErrorCodes.SUCCESS)

        # first plan should execute
        self.assertTrue(self.group.execute(plan1))

        # second plan should be invalid now (due to modified start point) and rejected
        self.assertFalse(self.group.execute(plan2))

        # newly planned trajectory should execute again
        error_code3, plan3, time = self.plan(current)
        error_code = MoveItErrorCodes()
        error_code.deserialize(error_code3)
        self.assertEqual(error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertTrue(self.group.execute(plan3))


if __name__ == '__main__':
    PKGNAME = 'moveit_ros_planning_interface'
    NODENAME = 'moveit_test_python_moveit_cpp'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, PythonMoveItCppTest)
