import roslib
roslib.load_manifest('moveit_commander')
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_commander.commander import MoveGroupCommander
# initialize
rospy.init_node('test_arm_control', anonymous=True)
arm_mover = MoveGroupCommander('arm')

# desired joint positions
#msg = JointState()
#joint_position = [-0.817, 0.001, -1.253, -0.892, 60.854, -0.250, 3.338]
#joints = [-0.199792813192, -0.0301778257579, -0.119474613742, 0.113024891124, -0.0663316698686, -0.497162338433, 0.488334857099]
#msg.position = joints

# get a joint state message already configured for this arm
js = arm_mover.get_current_joint_values()
print "Current joints: %s" % (str(js))

arm_mover.set_pose_reference_frame('base_link')
pose = arm_mover.get_current_pose()
print "Current pose: %s" % (str(pose))
# set desired joint positions
#js.position = joints
#print 'Moving to %s' % (str(joints))

# send out the command
#reached_goal = arm_mover.move_to_goal(js)
#if not reached_goal:
#    print arm_mover.get_exceptions()
#reached_goal = arm_mover.go(joints)
#if reached_goal:
#    print 'Success.'
#else: 
#    print 'Goal not reached.'   
