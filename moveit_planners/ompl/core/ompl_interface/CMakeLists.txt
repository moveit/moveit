set(MOVEIT_LIB_NAME moveit_ompl_interface)

add_library(moveit_ompl_interface
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
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
target_link_libraries(${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_state_space test/test_state_space.cpp)
target_link_libraries(test_state_space ${MOVEIT_LIB_NAME})
