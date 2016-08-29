cmake_minimum_required(VERSION 2.8.3)
project(stomp_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
)
find_package(Eigen REQUIRED)

add_definitions("-std=c++11")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stomp_core
  CATKIN_DEPENDS roscpp cmake_modules
  DEPENDS eigen
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/stomp.cpp
   src/utils.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  set(UTEST_SRC_FILES test/utest.cpp
      test/stomp_3dof.cpp)
  catkin_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})

endif()
