#!/usr/bin/env python

import sys
import argparse
import roslib
import rospy
from threading import Thread

from moveit_commander import ObjectDetector, ObjectBroadcaster

def auto_trigger(detector, wait):
    rospy.loginfo("Auto-triggering object detection every %s seconds" % str(wait))
    r = rospy.Rate(1.0 / wait)
    while not rospy.is_shutdown():
        r.sleep()
        rospy.loginfo("Auto-triggering new object detection...")
        detector.trigger_detection()
        detector.wait_for_detection()
    
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Client that queries the ORK server and prints the output. '
                                     'Start your server and launch that file for testing.')
    parser.add_argument('--auto-trigger', help='Specifies the number of seconds in between calls to triggering object detection',
                        default=None, type=float)
    parser.add_argument('--min-confidence', help='Specifies the minimum confidence for a recognized object to be published to the planning scene',
                        default=0.5, type=float)
    parser.add_argument('--topic', help='Specifies whether recognized objects should be listened for on a topic',
                        default=False, type=bool)
    
    cmd_args = vars(parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:]))
    
    rospy.init_node('recognition_client')
    
    broadcaster = ObjectBroadcaster()
    broadcaster.set_minimum_confidence(cmd_args['min_confidence'])
    detector = ObjectDetector(broadcaster.broadcast)

    if cmd_args['topic'] is True:
        detector.start_continuous_monitor_client() 

    if cmd_args['auto_trigger'] is not None:
        detector.start_action_client()
        thread = Thread(target = auto_trigger, args = (detector, cmd_args['auto_trigger']))
        thread.start()
        rospy.spin()
        thread.join()
    else:
        rospy.spin()
