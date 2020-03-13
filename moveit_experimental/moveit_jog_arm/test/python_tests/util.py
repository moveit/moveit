import rospy

def wait_for_jogger_initialization(service_name):
    try:
      rospy.wait_for_service(service_name, timeout=15)
    except rospy.ROSException as exc:
      rospy.logerr("The jogger service " + service_name + " is not available: " + str(exc))
      return False

    return True
