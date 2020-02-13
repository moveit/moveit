#!/usr/bin/env python
import time

import pytest
import rospy

from moveit_msgs.srv import ChangeDriftDimensions
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

# Send a service call to allow drift in all but the y-dimension.
# In other words, only the y-dimension will be controlled exactly.
# Check that the service returns and the jog node continues to publish commands to the robot.

JOG_ARM_SETTLE_TIME_S = 10
ROS_SETTLE_TIME_S = 10

CARTESIAN_JOG_COMMAND_TOPIC = 'jog_server/delta_jog_cmds'

COMMAND_OUT_TOPIC = 'jog_server/command'


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


def test_drift_dimensions_service(node):
    sub = rospy.Subscriber(
        COMMAND_OUT_TOPIC, JointTrajectory, lambda msg: received.append(msg)
    )
    cartesian_cmd = CartesianJogCmd()
    time.sleep(ROS_SETTLE_TIME_S)  # wait for pub/subs to settle
    time.sleep(JOG_ARM_SETTLE_TIME_S)  # wait for jog_arm server to init

    # Make the service call to allow drift in all dimensions except y-translation
    drift_service = rospy.ServiceProxy('jog_server/change_drift_dimensions', ChangeDriftDimensions)
    # the transform is an identity matrix, not used for now
    drift_response = drift_service(True, False, True, True, True, True, Transform())
    # Check for successful response
    assert drift_response.success == True

    # This nonzero command should produce jogging output
    # A subscriber in a different thread fills `received`
    TEST_DURATION = 1
    cartesian_cmd.send_cmd([0, 0, 0], [0, 0, 1])
    received = []
    rospy.sleep(TEST_DURATION)

    # Ensure some messages have been received
    assert len(received) > 10


if __name__ == '__main__':
   node = node()
   test_drift_dimensions_service(node)
