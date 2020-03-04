#!/usr/bin/env python
import time

import pytest
import rospy

from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory

# Test that commands that are too fast are caught and flagged
# This can be run as part of a pytest, or like a normal ROS executable:
# rosrun moveit_jog_arm test_vel_accel_limits.py

JOG_ARM_SETTLE_TIME_S = 10
ROS_SETTLE_TIME_S = 10

JOINT_JOG_COMMAND_TOPIC = 'jog_server/joint_delta_jog_cmds'

COMMAND_OUT_TOPIC = 'jog_server/command'


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


def test_vel_accel_limits(node):
    # Test sending a joint command

    sub = rospy.Subscriber(
        COMMAND_OUT_TOPIC, JointTrajectory, lambda msg: received.append(msg)
    )

    joint_cmd = JointJogCmd()
    time.sleep(ROS_SETTLE_TIME_S)  # wait for pub/subs to settle

    received = []
    TEST_DURATION = 1
    PUBLISH_PERIOD = 0.01 # 'PUBLISH_PERIOD' from jog_arm config file
    velocities = [0.1]

    start_time = rospy.get_rostime()
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
    test_vel_accel_limits(node)
