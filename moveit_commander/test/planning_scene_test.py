import roslib
roslib.load_manifest('moveit_commander')
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_commander import PlanningScene
from moveit_commander import ArmMover
#from moveit_commander import PlanningScene

# initialize
rospy.init_node('test_arm_control', anonymous=True)
arm_mover = ArmMover("arm")

PlanningScene.add_simple_object("box", "base_link", SolidPrimitive.BOX, 0.5, 0, 0.15, 0.0, 0.0, 0.0, 1.0, [0.3, 0.6, 0.3])
PlanningScene.attach_simple_collision_object("cup6", "low_cost_gripper_palm_link", SolidPrimitive.CYLINDER, "low_cost_gripper_palm_link", ["low_cost_gripper_r_finger_tip_link", "low_cost_gripper_r_finger_link", "low_cost_gripper_l_finger_tip_link", "low_cost_gripper_l_finger_link", "wrist_roll_link", "low_cost_gripper_palm_link"], 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, [0.1, 0.04])


#arm_mover.add_simple_object("/u/selliott/workspace_temp/cob_environments/cob_gazebo_objects/Media/models/milk.dae")
# desired joint positions
#msg = JointState()
#joint_position = [-0.817, 0.001, -1.253, -0.892, 60.854, -0.250, 3.338]
#joints = [-0.199792813192, -0.0301778257579, -0.119474613742, 0.113024891124, -0.0663316698686, -0.497162338433, 0.488334857099]
#joints = [0.0172883820558, -0.118635322825, -0.993054012956, 0.0887487314401, -0.615995126404, -0.110791162283, 0.992815442109]
#msg.position = joints

# get a joint state message already configured for this arm
#js = arm_mover.get_current_joint_values()

# set desired joint positions
#js.position = joints
#print 'Moving to %s' % (str(joints))

# send out the command
#reached_goal = arm_mover.move_to_goal(js)
#if not reached_goal:
#    print arm_mover.get_exceptions()
#reached_goal = arm_mover.move_arm(js)
#if reached_goal:
#    print 'Success.'
#else: 
#    print 'Goal not reached.'   

rospy.sleep(3.0)

PlanningScene.remove_simple_object("box", "base_link")
PlanningScene.remove_simple_attached_object("cup6", "low_cost_gripper_palm_link", "low_cost_gripper_palm_link")
PlanningScene.remove_simple_object("cup6", "low_cost_gripper_palm_link")
