#!/usr/bin/env python

import argparse
import roslib
import rospy
from threading import Thread

from moveit_commander import ObjectDetector, ObjectBroadcaster

def auto_trigger(detector, wait):
    r = rospy.Rate(1.0 / wait)
    while not rospy.is_shutdown():
        r.sleep(dur)
        detector.trigger_detection()
    
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Client that queries the ORK server and prints the output. '
                                     'Start your server and launch that file for testing.')
    parser.add_argument('--auto_trigger', help='Specifies the number of seconds in between calls to triggering object detection',
                        default=None, type=float)
    
    cmd_args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
    
    rospy.init_node('recognition_client')
    
    broadcaster = ObjectBroadcaster()
    detector = ObjectDetector(broadcaster.broadcast)
    detector.start_continuous_monitor_client() 

    if cmd_args['auto_trigger'] is not None:
        detector.start_action_client()
        print "Auto-triggering object detection every %s seconds" % str(cmd_args['auto_trigger'])
        thread = Thread(target = auto_trigger, args = (detector, cmd_args['auto_trigger']))
        thread.start()
        rospy.spin()
        thread.join()
    else:
        rospy.spin()
