#!/usr/bin/env python
import rospy
import actionlib
import roslib
roslib.load_manifest('moveit_commander')
from object_recognition_msgs.msg import *

class ObjectRecognition(object):

    def __init__(self):
        self._client = actionlib.SimpleActionClient('recognize_objects', ObjectRecognitionAction)
        self._client.wait_for_server()

    def look_for_objects(self, dimensions = False):

        """
        [x,y,z] of bounding box where robot should look, 
        if not specified then robot looks everywhere within sensor range
        """

        start = rospy.Time.now() # for checking the round trip time.

        goal = ObjectRecognitionGoal()

        if dimensions:
            goal.use_roi = True
            goal.filter_limits = dimensions

        self._client.send_goal(goal)
        self._client.wait_for_result() # wait indefinitely for a result

        #print out the round trip time.
        print "Time for 1 detection:", (rospy.Time.now() - start).to_sec()
        return self._client.get_result() 

