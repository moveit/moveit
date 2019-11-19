cmake_minimum_required(VERSION 3.1.3)
project(moveit_commander)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
  include(CodeCoverage)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

install(PROGRAMS bin/${PROJECT_NAME}_cmdline.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_subdirectory(test)
