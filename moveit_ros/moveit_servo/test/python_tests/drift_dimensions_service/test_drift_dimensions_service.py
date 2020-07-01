#!/usr/bin/env python
import time

import pytest
import rospy

from moveit_msgs.srv import ChangeDriftDimensions
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

# Import common Python test utilities
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import util

# Send a service call to allow drift in all but the y-dimension.
# In other words, only the y-dimension will be controlled exactly.
# Check that the service returns and the servo node continues to publish commands to the robot.

CARTESIAN_COMMAND_TOPIC = 'servo_server/delta_twist_cmds'

COMMAND_OUT_TOPIC = 'servo_server/command'

SERVICE_NAME = 'servo_server/change_drift_dimensions'


@pytest.fixture
def node():
    return rospy.init_node('pytest', anonymous=True)


def test_drift_dimensions_service(node):
    assert util.wait_for_servo_initialization()

    # Service to change drift dimensions
    drift_service = rospy.ServiceProxy(SERVICE_NAME, ChangeDriftDimensions)

    # Service call to allow drift in all dimensions except y-translation
    # The transform is an identity matrix, not used for now
    drift_response = drift_service(True, False, True, True, True, True, Transform())
    # Check for successful response
    assert drift_response.success == True


if __name__ == '__main__':
   node = node()
   test_drift_dimensions_service(node)
