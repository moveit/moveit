#!/usr/bin/env python
import time

import pytest
import rospy
from geometry_msgs.msg import TwistStamped

from control_msgs.msg import JointJog
from std_msgs.msg import Int8

# Import common Python test utilities
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import util

# The robot starts at a singular position (see config file).
# The jogger should halt and publish a warning.
# Listen for a warning message from the jogger.
# This can be run as part of a pytest, or like a normal ROS executable:
# rosrun moveit_jog_arm test_jog_arm_halt_msg.py

CARTESIAN_JOG_COMMAND_TOPIC = 'jog_server/delta_jog_cmds'

# jog_arm should publish a nonzero warning code here
HALT_TOPIC = 'jog_server/status'

# Check if jogger is initialized with this service
SERVICE_NAME = 'jog_server/change_drift_dimensions'


@pytest.fixture
def node():
    return rospy.init_node('pytest', anonymous=True)


class CartesianJogCmd(object):
    def __init__(self):
        self._pub = rospy.Publisher(
            CARTESIAN_JOG_COMMAND_TOPIC, TwistStamped, queue_size=1
        )

    def send_cmd(self, linear, angular):
        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        ts.twist.linear.x, ts.twist.linear.y, ts.twist.linear.z = linear
        ts.twist.angular.x, ts.twist.angular.y, ts.twist.angular.z = angular
        self._pub.publish(ts)


def test_jog_arm_halt_msg(node):
    assert util.wait_for_jogger_initialization(SERVICE_NAME)

    received = []
    sub = rospy.Subscriber(
        HALT_TOPIC, Int8, lambda msg: received.append(msg)
    )
    cartesian_cmd = CartesianJogCmd()

    # This nonzero command should produce jogging output
    # A subscriber in a different thread fills `received`
    TEST_DURATION = 1
    start_time = rospy.get_rostime()
    received = []
    while (rospy.get_rostime() - start_time).to_sec() < TEST_DURATION:
        cartesian_cmd.send_cmd([1, 1, 1], [0, 0, 1])
        time.sleep(0.1)

    # Check the received messages
    # A non-zero value signifies a warning
    assert len(received) > 3
    assert (received[-1].data != 0) or (received[-2].data != 0) or (received[-3].data != 0)


if __name__ == '__main__':
   node = node()
   test_jog_arm_halt_msg(node)
