from exception import *
from roscpp_initializer import *
from planning_scene_interface import *
from move_group import *
from robot import *
from interpreter import *

# TODO: repair broken destruction of MoveIt objects in python to remove this workaround
# workaround for core dump whenever exiting Python MoveIt script (https://github.com/ros-planning/moveit_commander/issues/15#issuecomment-34441531)
import atexit, os
atexit.register(lambda : os._exit(0))
