import rospy
from std_msgs.msg import Int8

# servo should publish a nonzero warning code here
STATUS_TOPIC = 'servo_server/status'

def wait_for_servo_initialization(timeout=15):
    try:
      rospy.wait_for_message(STATUS_TOPIC, Int8, timeout=timeout)
    except rospy.ROSException as exc:
      rospy.logerr("The servo topic " + STATUS_TOPIC + " is not available: " + str(exc))
      return False

    return True
