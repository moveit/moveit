set(MOVEIT_LIB_NAME moveit_kinematics_base)

add_library(${MOVEIT_LIB_NAME} src/kinematics_base.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # Avoid warnings about deprecated members of KinematicsBase when building KinematicsBase itself.
  # TODO: remove when deperecated variables (tip_frame_, search_discretization_) are finally removed.
  target_compile_options(${MOVEIT_LIB_NAME} PRIVATE -Wno-deprecated-declarations)
endif()

# This line is needed to ensure that messages are done being built before this is built
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
