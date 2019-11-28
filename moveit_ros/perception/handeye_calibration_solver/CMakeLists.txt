# Suppress the warning 'int _import_array()' defined but not used in 'usr/include/python2.7/numpy/__multiarray_api.h'
add_compile_options(-Wno-unused-function)

set(HEADERS
  include/moveit/handeye_calibration_solver/handeye_solver_base.h
  include/moveit/handeye_calibration_solver/handeye_solver_default.h
)

# ---[ Using cmake scripts and modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

find_package(PythonLibs 2.7 REQUIRED)
find_package(NumPy 1.7 REQUIRED)

#catkin_lint: ignore_once missing_directory
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${PYTHON_INCLUDE_DIRS})

# Plugin Source
set(SOURCE_FILES
  src/handeye_solver_default.cpp
)

set(MOVEIT_LIB_NAME moveit_handeye_calibration_solver)
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES} ${HEADERS})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(jsoncpp REQUIRED)
  get_target_property(JSON_INC_PATH jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)

  include_directories(${JSON_INC_PATH})

  catkin_add_gtest(test_handeye_solver test/handeye_solver_test.cpp)
  target_link_libraries(test_handeye_solver ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} jsoncpp_lib)
endif()