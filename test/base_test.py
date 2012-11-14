#!/usr/bin/env python

import roslib
roslib.load_manifest('moveit_commander')
import rospy

from moveit_commander import base

if __name__=='__main__':
    rospy.init_node('move_base_example')
    b = base.Base()
    x,y,theta = b.get_current_pose()

    print 'CURRENT POSE: ' + str(x) + ' ' + str(y) + ' ' + str(theta)
    b.move_to(0.5, 0.5, 0.0, True)
