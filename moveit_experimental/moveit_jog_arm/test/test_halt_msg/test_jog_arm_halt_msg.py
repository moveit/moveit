#!/usr/bin/env python
import time

import pytest
import rospy
from geometry_msgs.msg import TwistStamped

from control_msgs.msg import JointJog
from std_msgs.msg import Bool

# The robot starts at a singular position (see config file).
# Listen for a halt message from the jogger.
# This can be run as part of a pytest, or like a normal ROS executable:
# rosrun moveit_jog_arm test_jog_arm_integration.py

JOG_ARM_SETTLE_TIME_S = 10
ROS_SETTLE_TIME_S = 10

CARTESIAN_JOG_COMMAND_TOPIC = 'jog_server/delta_jog_cmds'

# jog_arm should publish 'true' here if it halts
HALT_TOPIC = 'jog_server/halted'


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
    sub = rospy.Subscriber(
        HALT_TOPIC, Bool, lambda msg: received.append(msg)
    )
    cartesian_cmd = CartesianJogCmd()
    time.sleep(ROS_SETTLE_TIME_S)  # wait for pub/subs to settle
    time.sleep(JOG_ARM_SETTLE_TIME_S)  # wait for jog_arm server to init

    # This nonzero command should produce jogging output
    # A subscriber in a different thread fills `received`
    TEST_DURATION = 1
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 1])
    received = []
    rospy.sleep(TEST_DURATION)

    # Check the received messages
    assert len(received) > 1
    assert received[-1].data == True


if __name__ == '__main__':
   node = node()
   test_jog_arm_halt_msg(node)
