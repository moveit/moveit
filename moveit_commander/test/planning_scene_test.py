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
planning_scene = PlanningScene();

planning_scene.add_box("box", "base_link", [0.5, 0, 0.15], [0.0, 0.0, 0.0, 1.0], 0.3, 0.6, 0.3)
planning_scene.attach_cylinder("cup", "low_cost_gripper_palm_link", "low_cost_gripper_palm_link", ["low_cost_gripper_r_finger_tip_link", "low_cost_gripper_r_finger_link", "low_cost_gripper_l_finger_tip_link", "low_cost_gripper_l_finger_link", "wrist_roll_link", "low_cost_gripper_palm_link"], [0.2, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.1, 0.04)


rospy.sleep(5.0)


print "Detaching object."
planning_scene.remove_simple_attached_object("cup", "low_cost_gripper_palm_link", "low_cost_gripper_palm_link")

rospy.sleep(5.0)
print "Removing object from world."
planning_scene.remove_simple_object("cup", "low_cost_gripper_palm_link")
