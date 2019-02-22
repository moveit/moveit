# -*- coding: utf-8 -*-
import time

import pytest
import rospy
from geometry_msgs.msg import TwistStamped

from jog_msgs.msg import JogJoint
from trajectory_msgs.msg import JointTrajectory

JOG_ARM_STALE_TIMEOUT_S = 2.0
JOG_ARM_INIT_WAIT_TIME_S = 0.5
ROS_SETTLE_WAIT_TIME_S = 0.5

JOINT_JOG_COMMAND_TOPIC = 'jog_arm_server/joint_delta_jog_cmds'
CARTESIAN_JOG_COMMAND_TOPIC = 'jog_arm_server/delta_jog_cmds'

COMMAND_OUT_TOPIC = 'jog_arm_server/command'


@pytest.fixture
def node():
    return rospy.init_node('pytest', anonymous=True)


class JointJogCmd(object):
    def __init__(self):
        self._pub = rospy.Publisher(JOINT_JOG_COMMAND_TOPIC, JogJoint, queue_size=1)

    def send_cmd(self, joint_pos):
        jj = JogJoint()
        jj.header.stamp = rospy.Time.now()
        jj.joint_names = ['joint_{}'.format(i) for i in range(len(joint_pos))]
        jj.deltas = list(map(float, joint_pos))
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


def test_jog_arm_generates_joint_trajectory_when_joint_jog_command_is_received(node):
    received = []
    sub = rospy.Subscriber(
        COMMAND_OUT_TOPIC, JointTrajectory, lambda x: received.append(x)
    )
    joint_cmd = JointJogCmd()
    cartesian_cmd = CartesianJogCmd()
    time.sleep(ROS_SETTLE_WAIT_TIME_S)  # wait for pub/subs to settle
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 0])
    time.sleep(JOG_ARM_INIT_WAIT_TIME_S)  # wait for jog_arm server to init
    joint_cmd.send_cmd([0, 0, 0, 0, 0, 1])

    # wait until 103 messages received
    # 1s timeout, 0.01s message period, 0.005s message send delay ->
    # 99 command messages and 4 halt messages
    while len(received) < 104:
        rospy.wait_for_message(COMMAND_OUT_TOPIC, JointTrajectory, timeout=JOG_ARM_STALE_TIMEOUT_S)

    # the first message must be a command message
    first_msg = received[0]
    assert first_msg.joint_names == [
        'joint_1',
        'joint_2',
        'joint_3',
        'joint_4',
        'joint_5',
        'joint_6',
    ]
    assert len(first_msg.points) == 1
    tfs = first_msg.points[0].time_from_start
    assert tfs.secs == 0 and tfs.nsecs == 1e7
    assert len(first_msg.points[0].positions) == 6
    assert len(first_msg.points[0].velocities) == 6
    assert first_msg.points[0].velocities[5] > 0.0

    # last message must be a halt message
    last_msg = received[-1]
    assert len(last_msg.points[0].positions) == 6
    assert len(last_msg.points[0].velocities) == 6
    assert last_msg.points[0].velocities[5] == pytest.approx(0.0)
