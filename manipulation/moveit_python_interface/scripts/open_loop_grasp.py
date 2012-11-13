import roslib
roslib.load_manifest('moveit_python_interface')
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_python_interface.arm_mover import ArmMover
from moveit_python_interface.planning_scene_interface import PlanningScene
from moveit_python_interface.gripper import Gripper

import yaml

# initialize


rospy.init_node('test_grasp', anonymous=True)

arm_mover = ArmMover('arm')
gripper = Gripper('right_arm')

PlanningScene.add_simple_object("floor", "base_link", SolidPrimitive.BOX, 0, 0, -0.1, 0.0, 0.0, 0.0, 1.0, [2.0, 2.0, 0.1])
PlanningScene.add_simple_object("box2", "base_link", SolidPrimitive.BOX, 0.5, 0, 0.10, 0.0, 0.0, 0.0, 1.0, [0.3, 0.6, 0.20])


gripper.open()
# desired joint positions
#msg = JointState()
#joint_position = [-0.817, 0.001, -1.253, -0.892, 60.854, -0.250, 3.338]
filename = 'grasping'
stream = open(filename, 'r')
joint_values = yaml.load(stream)

home = joint_values['home']
above = joint_values['above']
grasp = joint_values['grasp']
away = joint_values['away']

#msg.position = joints

# get a joint state message already configured for this arm
js = arm_mover.get_current_joint_values()

# set desired joint positions
#js.position = home
#print 'Moving to %s' % ('Home')

# send out the command
#reached_goal = arm_mover.move_to_goal(js)
#if not reached_goal:
#    print arm_mover.get_exceptions()
#reached_goal = arm_mover.move_arm(js)
#if reached_goal:
#    print 'Success.'
#else: 
#    print 'Goal not reached.'

#rospy.sleep(2.0)

js.position = above

print 'Moving to Above'

reached_goal = arm_mover.move_arm(js)
if reached_goal:
    print 'Success.'
else: 
    print 'Goal not reached.'


#rospy.sleep(2.0)

js.position = grasp

print 'Moving to Grasp'

reached_goal = arm_mover.move_arm(js)
if reached_goal:
    print 'Success.'
else:
    print 'Goal not reached.'

#rospy.sleep(2.0)

try:
    gripper.close()
except:
    pass

rospy.sleep(2.0)

js.position = above

print 'Moving to Above'

reached_goal = arm_mover.move_arm(js)
if reached_goal:
    print 'Success.'
else:
    print 'Goal not reached.'

PlanningScene.attach_simple_object_to_gripper("cup6", "low_cost_gripper_palm_link", SolidPrimitive.CYLINDER, "low_cost_gripper_palm_link", ["low_cost_gripper_r_finger_tip_link", "low_cost_gripper_r_finger_link", "low_cost_gripper_l_finger_tip_link", "low_cost_gripper_l_finger_link", "wrist_roll_link", "low_cost_gripper_palm_link"], 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, [0.1, 0.04])


#PlanningScene.attach_simple_object_to_gripper("cup", SolidPrimitive.CYLINDER, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, [0.08, 0.1])

#rospy.sleep(2.0)

js.position = away

print 'Moving to Away'

reached_goal = arm_mover.move_arm(js)
if reached_goal:
    print 'Success.'
else:
    print 'Goal not reached.'

#rospy.sleep(5.0)

js.position = above

print 'Moving to Above'

reached_goal = arm_mover.move_arm(js)
if reached_goal:
    print 'Success.'
else:
    print 'Goal not reached.'

PlanningScene.remove_simple_attached_object("cup6", "low_cost_gripper_palm_link", "low_cost_gripper_palm_link")
PlanningScene.remove_simple_object("cup6", "low_cost_gripper_palm_link")

#rospy.sleep(2.0)

js.position = grasp

print 'Moving to Grasp'

reached_goal = arm_mover.move_arm(js)
if reached_goal:
    print 'Success.'
else:
    print 'Goal not reached.'

try:
    gripper.open()
except:
    pass

rospy.sleep(2.0)

js.position = above

print 'Moving to Above'

reached_goal = arm_mover.move_arm(js)
if reached_goal:
    print 'Success.'
else:
    print 'Goal not reached.'

#rospy.sleep(2.0)

PlanningScene.remove_simple_object("box2", "base_link")
PlanningScene.remove_simple_object("floor", "base_link")
