cmake_minimum_required(VERSION 2.8.3)
project(moveit_ikfast)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES moveit_ikfast
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(moveit_ikfast
#   src/${PROJECT_NAME}/moveit_ikfast.cpp
# )

## Declare a cpp executable
# add_executable(moveit_ikfast_node src/moveit_ikfast_node.cpp)

## Add dependencies to the executable
# add_dependencies(moveit_ikfast_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
# target_link_libraries(moveit_ikfast_node
#   ${catkin_LIBRARIES}
# )

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


## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS moveit_ikfast moveit_ikfast_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )