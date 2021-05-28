set(MOVEIT_LIB_NAME moveit_prbt_manipulator_ikfast_plugin)

add_library(${MOVEIT_LIB_NAME} src/prbt_manipulator_ikfast_plugin.cpp)
target_include_directories(${MOVEIT_LIB_NAME} PRIVATE include/)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${MOVEIT_LIB_NAME} PRIVATE -Wno-unused-variable)

# enable aligned new in gcc7+
if(CMAKE_COMPILER_IS_GNUCXX)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
    target_compile_options(${MOVEIT_LIB_NAME} PRIVATE -faligned-new)
  endif()
endif()

install(TARGETS
  ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
