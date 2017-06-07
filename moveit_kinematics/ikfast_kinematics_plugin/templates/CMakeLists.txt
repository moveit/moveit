cmake_minimum_required(VERSION 2.8.12)
project(_PACKAGE_NAME_)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(include)

set(IKFAST_LIBRARY_NAME _LIBRARY_NAME_)

find_package(LAPACK REQUIRED)

add_library(${IKFAST_LIBRARY_NAME} src/_ROBOT_NAME___GROUP_NAME__ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${IKFAST_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  _ROBOT_NAME___GROUP_NAME__moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
