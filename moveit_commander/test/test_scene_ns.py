import os
os.environ['ROS_NAMEPSACE'] = 'my_robot'

from geometry_msgs.msg import PoseStamped
import rospy
import moveit_commander

moveit_commander.roscpp_initialize([])
rospy.init_node("testing_scene")
scene = moveit_commander.PlanningSceneInterface()

for i in range(10):
    scene.add_box("box", PoseStamped())
    rospy.sleep(1.0)

