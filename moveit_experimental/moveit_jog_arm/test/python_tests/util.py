import rospy

def wait_for_jogger_initialization(service_name):
    try:
      rospy.wait_for_service(service_name, timeout=15)
    except rospy.ServiceException as exc:
      rospy.logerr("The jogger never finished initialization, expected service is not available: " + str(exc))
      return False

    return True
