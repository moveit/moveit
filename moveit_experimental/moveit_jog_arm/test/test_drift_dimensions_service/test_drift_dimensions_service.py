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


def test_drift_dimensions_service(node):
    # Make the service call to allow drift in all dimensions except y-translation
    drift_service = rospy.ServiceProxy('jog_server/change_drift_dimensions', ChangeDriftDimensions)
    # the transform is an identity matrix, not used for now
    drift_response = drift_service(True, False, True, True, True, True, Transform())
    # Check for successful response
    assert drift_response.success == True


if __name__ == '__main__':
   node = node()
   test_drift_dimensions_service(node)
