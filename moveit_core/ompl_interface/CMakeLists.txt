cmake_minimum_required(VERSION 2.8)
project(ompl_interface)

include_directories(include)

find_package(Boost REQUIRED thread date_time serialization system filesystem)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(OMPL REQUIRED ompl)
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})

find_package(planning_scene REQUIRED)
include_directories(${planning_scene_INCLUDE_DIRS})
link_directories(${planning_scene_LIBRARY_DIRS})

find_package(constraint_samplers REQUIRED)
include_directories(${constraint_samplers_INCLUDE_DIRS})
link_directories(${constraint_samplers_LIBRARY_DIRS})

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  DEPENDS planning_scene constraint_samplers
  )

add_library(${PROJECT_NAME}
  src/ompl_interface.cpp
  src/planning_context_manager.cpp
  src/constraints_library.cpp
  src/model_based_planning_context.cpp
  src/parameterization/model_based_joint_state_space.cpp
  src/parameterization/model_based_state_space.cpp
  src/parameterization/model_based_state_space_factory.cpp
  src/parameterization/joint_space/joint_model_state_space.cpp
  src/parameterization/joint_space/joint_model_state_space_factory.cpp  
  src/parameterization/work_space/pose_model_state_space.cpp  
  src/parameterization/work_space/pose_model_state_space_factory.cpp
  src/detail/threadsafe_state_storage.cpp
  src/detail/state_validity_checker.cpp
  src/detail/projection_evaluators.cpp
  src/detail/goal_union.cpp
  src/detail/constrained_sampler.cpp
  src/detail/constrained_goal_sampler.cpp
  src/detail/ompl_console.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-fopenmp)
endif()

target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${constraint_samplers_LIBRARIES} ${planning_scene_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

catkin_add_gtest(test_state_space test/test_state_space.cpp) 
target_link_libraries(test_state_space ${PROJECT_NAME} ompl)
