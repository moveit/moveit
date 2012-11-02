import roslib
roslib.load_manifest('moveit_python_interface')
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_python_interface.planning_scene_interface import PlanningScene
from moveit_python_interface.arm_mover import ArmMover
#from moveit_python_interface import PlanningScene

# initialize
rospy.init_node('test_arm_control', anonymous=True)
arm_mover = ArmMover("arm")

PlanningScene.add_simple_object("circle", SolidPrimitive.CYLINDER, 0.6, -0.6, 0.5, 0.0, 0.0, 0.0, 1.0, [1, 0.1 ])

#arm_mover.add_simple_object("/u/selliott/workspace_temp/cob_environments/cob_gazebo_objects/Media/models/milk.dae")
# desired joint positions
#msg = JointState()
#joint_position = [-0.817, 0.001, -1.253, -0.892, 60.854, -0.250, 3.338]
#joints = [-0.199792813192, -0.0301778257579, -0.119474613742, 0.113024891124, -0.0663316698686, -0.497162338433, 0.488334857099]
joints = [0.0172883820558, -0.118635322825, -0.993054012956, 0.0887487314401, -0.615995126404, -0.110791162283, 0.992815442109]
#msg.position = joints

# get a joint state message already configured for this arm
js = arm_mover.get_current_joint_values()

# set desired joint positions
js.position = joints
print 'Moving to %s' % (str(joints))

# send out the command
#reached_goal = arm_mover.move_to_goal(js)
#if not reached_goal:
#    print arm_mover.get_exceptions()
reached_goal = arm_mover.move_arm(js)
if reached_goal:
    print 'Success.'
else: 
    print 'Goal not reached.'   

PlanningScene.remove_simple_object("circle")
