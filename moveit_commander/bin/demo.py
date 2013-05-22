#!/usr/bin/env python

import sys
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    
    pr2 = RobotCommander()
    print "Groups: ", pr2.get_group_names()
    
    joint = pr2.r_shoulder_pan_joint
    print joint.bounds()
    print joint.value()
    joint.move(joint.value() + 0.1)

    roscpp_shutdown()
