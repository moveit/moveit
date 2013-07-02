cmake_minimum_required(VERSION 2.8.3)
project(moveit_ikfast)

find_package(catkin REQUIRED)

catkin_package()


#############
## Install ##
#############

catkin_python_setup()
install(
  PROGRAMS 
    src/create_ikfast_moveit_plugin.py
    src/round_collada_numbers.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
