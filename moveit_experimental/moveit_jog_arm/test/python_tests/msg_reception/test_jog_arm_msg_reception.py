#!/usr/bin/env python
import time

import pytest
import rospy
from geometry_msgs.msg import TwistStamped

from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory

# Import common Python test utilities
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import util

# Test that the jogger publishes controller commands when it receives Cartesian or joint commands.
# This can be run as part of a pytest, or like a normal ROS executable:
# rosrun moveit_jog_arm test_jog_arm_integration.py

JOINT_JOG_COMMAND_TOPIC = 'jog_server/joint_delta_jog_cmds'
CARTESIAN_JOG_COMMAND_TOPIC = 'jog_server/delta_jog_cmds'

COMMAND_OUT_TOPIC = 'jog_server/command'

# Check if jogger is initialized with this service
SERVICE_NAME = 'jog_server/change_drift_dimensions'


@pytest.fixture
def node():
    return rospy.init_node('pytest', anonymous=True)


class JointJogCmd(object):
    def __init__(self):
        self._pub = rospy.Publisher(JOINT_JOG_COMMAND_TOPIC, JointJog, queue_size=1)

    def send_joint_velocity_cmd(self, joint_pos):
        jj = JointJog()
        jj.header.stamp = rospy.Time.now()
        jj.joint_names = ['joint_{}'.format(i) for i in range(len(joint_pos))]
        jj.velocities = list(map(float, joint_pos))
        self._pub.publish(jj)


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


def test_jog_arm_cartesian_command(node):
    # Test sending a cartesian velocity command

    assert util.wait_for_jogger_initialization(SERVICE_NAME)

    received = []
    sub = rospy.Subscriber(
        COMMAND_OUT_TOPIC, JointTrajectory, lambda msg: received.append(msg)
    )
    cartesian_cmd = CartesianJogCmd()

    # Repeated zero-commands should produce no output, other than a few halt messages
    # A subscriber in a different thread fills 'received'
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 0])
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 0])
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 0])
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 0])
    received = []
    rospy.sleep(1)
    assert len(received) <= 4 # 'num_outgoing_halt_msgs_to_publish' in the config file

    # This nonzero command should produce jogging output
    # A subscriber in a different thread fills `received`
    TEST_DURATION = 1
    PUBLISH_PERIOD = 0.01 # 'PUBLISH_PERIOD' from jog_arm config file

    # Send a command to start the jogger
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 1])

    start_time = rospy.get_rostime()
    received = []
    while (rospy.get_rostime() - start_time).to_sec() < TEST_DURATION:
        cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 1])
        time.sleep(0.1)
    # TEST_DURATION/PUBLISH_PERIOD is the expected number of messages in this duration.
    # Allow a small +/- window due to rounding/timing errors
    assert len(received) >= TEST_DURATION/PUBLISH_PERIOD - 20
    assert len(received) <= TEST_DURATION/PUBLISH_PERIOD + 20


def test_jog_arm_joint_command(node):
    # Test sending a joint command

    assert util.wait_for_jogger_initialization(SERVICE_NAME)

    received = []
    sub = rospy.Subscriber(
        COMMAND_OUT_TOPIC, JointTrajectory, lambda msg: received.append(msg)
    )
    joint_cmd = JointJogCmd()

    TEST_DURATION = 1
    PUBLISH_PERIOD = 0.01 # 'PUBLISH_PERIOD' from jog_arm config file
    velocities = [0.1]

    # Send a command to start the jogger
    joint_cmd.send_joint_velocity_cmd(velocities)

    start_time = rospy.get_rostime()
    received = []
    while (rospy.get_rostime() - start_time).to_sec() < TEST_DURATION:
        joint_cmd.send_joint_velocity_cmd(velocities)
        time.sleep(0.1)
    # TEST_DURATION/PUBLISH_PERIOD is the expected number of messages in this duration.
    # Allow a small +/- window due to rounding/timing errors
    assert len(received) >= TEST_DURATION/PUBLISH_PERIOD - 20
    assert len(received) <= TEST_DURATION/PUBLISH_PERIOD + 20


if __name__ == '__main__':
    node = node()
    time.sleep(JOG_ARM_SETTLE_TIME_S)  # wait for jog_arm server to init
    test_jog_arm_cartesian_command(node)
    test_jog_arm_joint_command(node)
