#!/usr/bin/env python

import roslib
roslib.load_manifest('move_group_interface')
import rospy
import sys

from move_group import *

def run(group_name):
    g = MoveGroup(group_name)
    print g.get_end_effector_link()

if __name__=='__main__':
    rospy.init_node('move_group_interface_cmdline', anonymous=True)
    if len(sys.argv) != 2:
        print "Usage: %s <group_name>" % sys.argv[0]
    else:
        run(sys.argv[1])
