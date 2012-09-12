cmake_minimum_required(VERSION 2.8)
project(kinematic_constraints)

include_directories(include)

find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS 
  planning_models
  collision_detection 
  collision_detection_fcl
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS planning_models
  )

add_library(${PROJECT_NAME}
  src/kinematic_constraint.cpp
  src/utils.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_constraints test/test_constraints.cpp)
target_link_libraries(test_constraints ${PROJECT_NAME})
