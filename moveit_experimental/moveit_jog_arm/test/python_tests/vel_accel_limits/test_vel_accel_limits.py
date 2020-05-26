#!/usr/bin/env python
import time

import pytest
import rospy

from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory

# Import common Python test utilities
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import util

# Test that commands that are too fast are caught and flagged
# This can be run as part of a pytest, or like a normal ROS executable:
# rosrun moveit_jog_arm test_vel_accel_limits.py

JOINT_JOG_COMMAND_TOPIC = 'jog_server/joint_delta_jog_cmds'

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


def test_vel_limit(node):
    # Test sending a joint command

    assert util.wait_for_jogger_initialization(SERVICE_NAME)

    received = []
    sub = rospy.Subscriber(
        COMMAND_OUT_TOPIC, JointTrajectory, lambda msg: received.append(msg)
    )

    joint_cmd = JointJogCmd()

    TEST_DURATION = 1
    PUBLISH_PERIOD = 0.01 # 'PUBLISH_PERIOD' from jog_arm config file

    # Panda arm limit, from joint_limits.yaml
    VELOCITY_LIMIT = rospy.get_param("/robot_description_planning/joint_limits/panda_joint1/max_velocity")
    # Send a velocity command that exceeds the limit
    velocities = [10 * VELOCITY_LIMIT]

    # Send a command to start the jogger
    joint_cmd.send_joint_velocity_cmd(velocities)

    start_time = rospy.get_rostime()
    received = []
    while (rospy.get_rostime() - start_time).to_sec() < TEST_DURATION:
        joint_cmd.send_joint_velocity_cmd(velocities)
        time.sleep(0.1)

    # Period of outgoing commands from the jogger, from yaml
    JOGGER_COMMAND_PERIOD = rospy.get_param("/jog_server/publish_period")

    # Should be no velocities greater than the limit
    assert len(received) > 2
    for msg_idx in range(1, len(received)):
        velocity = \
            (received[msg_idx].points[0].positions[0] - received[msg_idx - 1].points[0].positions[0]) / JOGGER_COMMAND_PERIOD
        assert abs(velocity) <= VELOCITY_LIMIT

if __name__ == '__main__':
    node = node()
    test_vel_limit(node)
    # TODO(andyz): add an acceleration limit test (the Panda joint_limits.yaml doesn't define acceleration limits)
