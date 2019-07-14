set(HEADERS
  include/moveit/handeye_calibration_target/handeye_target_base.h
  include/moveit/handeye_calibration_target/handeye_aruco_target.h
)

#catkin_lint: ignore_once missing_directory
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES
  src/handeye_aruco_target.cpp
)

set(MOVEIT_LIB_NAME moveit_handeye_calibration_target)
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES} ${HEADERS})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})