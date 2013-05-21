#!/usr/bin/env python

import sys
from moveit_commander import Robot, roscpp_initialize, roscpp_shutdown

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    
    pr2 = Robot()
    print "Groups: ", pr2.get_group_names()
    
    print pr2.r_wrist_roll_joint.bounds()

    roscpp_shutdown()
