import pymoveit
from pymoveit.planning_scene import PlanningScene
from pymoveit.robot_model import load_robot_model

urdf_path = '/opt/ros/noetic/share/moveit_resources_pr2_description/urdf/robot.xml'
srdf_path = '/opt/ros/noetic/share/moveit_resources_pr2_description/srdf/robot.xml'

urdf_model = pymoveit.urdf.parseURDFFile(urdf_path)
print(urdf_model.getName())

model = load_robot_model(urdf_path, srdf_path)
scene = PlanningScene(model)


