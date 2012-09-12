cmake_minimum_required(VERSION 2.8)
project(collision_detection)

include_directories(include)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(planning_models REQUIRED)
include_directories(${planning_models_INCLUDE_DIRS})
link_directories(${planning_models_LIBRARY_DIRS})

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS planning_models
  )

add_library(${PROJECT_NAME}
  src/collision_world.cpp 
  src/collision_robot.cpp
  src/collision_matrix.cpp
  src/collision_tools.cpp
  src/allvalid/collision_robot.cpp
  src/allvalid/collision_world.cpp
)

target_link_libraries(${PROJECT_NAME} ${planning_models_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
