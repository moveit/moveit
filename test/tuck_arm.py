import roslib
roslib.load_manifest('moveit_commander')
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_commander.commander import MoveGroupCommander
from moveit_commander.torso import Torso
from moveit_msgs.srv import ExecuteKnownTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
# initialize
rospy.init_node('tuck_arm', anonymous=True)
torso = Torso("torso_lift_joint/command")
torso.move(0.2)

#joints = [-0.199792813192, -0.0301778257579, -0.119474613742, 0.113024891124, -0.0663316698686, -0.497162338433, 0.488334857099]

arm_mover = MoveGroupCommander('arm')
joints = [1.0337725352659688, -0.2635069358441139, 0.2732018645713051, -1.1433179087498708, -0.0010712502247107196, 1.0083989964733064, -0.3671447970689187]

reached_goal = arm_mover.go(joints)
if reached_goal:
    print 'Success.'
else:
    print 'Goal not reached.'

"""
traj = RobotTrajectory()
traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upper_arm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
point = JointTrajectoryPoint()
point.time_from_start = rospy.Duration(1)
point.positions = []
point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
point.accelerations = [-0.12540648292888865, -0.054677776933911806, 0.12798875357017672, -0.936644722979469, 0.0009360306727148692, 0.9873930696622216, 0.007191206023983183]
traj.points.append(point) 

rospy.wait_for_service('execute_known_path')
    try:
        execute_known_path = rospy.ServiceProxy('execute_known_path', ExecuteKnownTrajectory)
        resp1 = execute_known_path(traj, True)
        print "Service call returned: %d" % resp1.error_code
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
"""

