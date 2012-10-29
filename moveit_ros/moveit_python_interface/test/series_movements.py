#!/usr/bin/env python

import roslib
roslib.load_manifest('moveit_python_interface')
import rospy

from moveit_python_interface import base
from moveit_python_interface.arm_mover import ArmMover

if __name__=='__main__':
    rospy.init_node('series_movements_demo')
    b = base.Base()
    x,y,theta = b.get_current_pose()

    arm_mover = ArmMover('arm')
    """
    joints = [1.6731839662, 0.619746633081, -1.55729298232, -0.805061599583, -1.74644450444, -0.367394230879, 0.974680149403]
    js = arm_mover.get_current_joint_values()

    js.position = joints

    reached_goal = arm_mover.move_arm(js)
    if reached_goal:
        print 'Success.'
    else:
        print 'Goal not reached.'
    """
    arm_mover.move_relative(0,0,0.2,0,0,0)

    print 'CURRENT POSE: ' + str(x) + ' ' + str(y) + ' ' + str(theta)
    b.move_to(0.5, 0.0, 0.0, True)


    arm_mover.move_relative(0,-0.1,-0.1,0,0,0)
    rospy.sleep(2.0)

    print "Turn wrist"
    #arm_mover.move_relative(0,0.0,0,0,0,-2)
    js = arm_mover.get_current_joint_values()
    js.position[6] = js.position[6] - 3.0

    reached_goal = arm_mover.move_arm(js)
    if reached_goal:
        print 'Success.'
    else:
        print 'Goal not reached.'



    

