#!/usr/bin/env python

import roslib 
roslib.load_manifest('moveit_commander')
import rospy

from moveit_commander.gripper import Gripper

if __name__=='__main__':
    rospy.init_node("test_gripper", anonymous=True)
    gr = Gripper('right_arm')

    # Open
    # Close
    gr.open()
    
    rospy.sleep(2.0)
 
    gr.close()
