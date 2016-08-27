#!/usr/bin/env python

from __future__ import print_function

import unittest
import rostest
import subprocess
import rospkg
import roslib.packages

PKGNAME = 'moveit_ros_planning_interface'
NODENAME = 'moveit_cleanup_tests'

# As issue #592 is related to a crash during program exit,
# we cannot perform a standard unit test.
# We have to check the return value of the called program.
# As rostest doesn't do this automatically, we do it ourselves
# and call the actual test program here.
class CleanupTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(CleanupTest, self).__init__(*args, **kwargs)
        self._rospack = rospkg.RosPack()

    def test_py(self):
        cmd = roslib.packages.find_node(PKGNAME, "movegroup_interface.py", self._rospack)
        self.assertTrue(subprocess.call(cmd) == 0)

    def test_cpp(self):
        cmd = roslib.packages.find_node(PKGNAME, "test_cleanup", self._rospack)
        self.assertTrue(subprocess.call(cmd) == 0)

if __name__ == '__main__':
    rostest.rosrun(PKGNAME, NODENAME, CleanupTest)
