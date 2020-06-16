import rospy
from std_msgs.msg import Int8

# jog_arm should publish a nonzero warning code here
STATUS_TOPIC = 'jog_server/status'

def wait_for_jogger_initialization(timeout=15):
    try:
      rospy.wait_for_message(STATUS_TOPIC, Int8, timeout=timeout)
    except rospy.ROSException as exc:
      rospy.logerr("The jogger topic " + STATUS_TOPIC + " is not available: " + str(exc))
      return False

    return True
