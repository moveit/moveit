cmake_minimum_required(VERSION 2.8.3)
project(moveit_resources)

find_package(catkin REQUIRED)

catkin_package()

file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test" MOVEIT_TEST_RESOURCES_DIR)

if(WIN32)
    # Correct directory separator for Windows
    string(REPLACE "\\" "\\\\" MOVEIT_TEST_RESOURCES_DIR "${MOVEIT_TEST_RESOURCES_DIR}")
endif(WIN32)
configure_file("include/moveit/test_resources/config.h.in" "${CMAKE_CURRENT_SOURCE_DIR}/include/moveit/test_resources/config.h")

install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
