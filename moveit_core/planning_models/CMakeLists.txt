cmake_minimum_required(VERSION 2.8)
project(planning_models)

include_directories(include)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS 
  moveit_msgs 
  resource_retriever
  urdfdom
  srdfdom
  geometric_shapes
  shape_tools 
  eigen_stl_containers 
  eigen_conversions
  random_numbers
  console_bridge
  visualization_msgs
  kinematics_base
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS kinematics_base geometric_shapes shape_tools eigen_stl_containers moveit_msgs visualization_msgs random_numbers Eigen
  )

add_library(${PROJECT_NAME}
  src/kinematic_model.cpp
  src/kinematic_model/joint_model.cpp
  src/kinematic_model/fixed_joint_model.cpp
  src/kinematic_model/planar_joint_model.cpp
  src/kinematic_model/floating_joint_model.cpp
  src/kinematic_model/prismatic_joint_model.cpp
  src/kinematic_model/revolute_joint_model.cpp
  src/kinematic_model/link_model.cpp
  src/kinematic_model/joint_model_group.cpp
  src/kinematic_state.cpp
  src/kinematic_state/joint_state.cpp
  src/kinematic_state/link_state.cpp
  src/kinematic_state/attached_body.cpp
  src/kinematic_state/joint_state_group.cpp
  src/transforms.cpp
  src/conversions.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Unit tests
catkin_add_gtest(test_kinematic test/test_kinematic.cpp)
target_link_libraries(test_kinematic ${PROJECT_NAME})

catkin_add_gtest(test_kinematic_complex test/test_kinematic_complex.cpp)
target_link_libraries(test_kinematic_complex ${PROJECT_NAME}})

catkin_add_gtest(test_transforms test/test_transforms.cpp)
target_link_libraries(test_transforms ${PROJECT_NAME})
