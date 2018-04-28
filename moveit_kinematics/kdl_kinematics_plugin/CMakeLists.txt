set(MOVEIT_LIB_NAME moveit_kdl_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} src/kdl_kinematics_plugin.cpp
  src/chainiksolver_pos_nr_jl_mimic.cpp
  src/chainiksolver_vel_pinv_mimic.cpp)
<<<<<<< HEAD:moveit_kinematics/kdl_kinematics_plugin/CMakeLists.txt
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
=======
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION 0.7.3)
>>>>>>> upstream/indigo-devel:moveit_kinematics/kdl_kinematics_plugin/CMakeLists.txt

target_link_libraries(${MOVEIT_LIB_NAME} moveit_rdf_loader ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
