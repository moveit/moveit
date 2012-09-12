cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Eigen
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

pkg_check_modules(OGRE OGRE)
include_directories(SYSTEM ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  include/moveit_rviz_plugin/planning_display.h
  include/moveit_rviz_plugin/planning_frame.h
  include/moveit_rviz_plugin/planning_scene_render.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES ${headers})

qt4_wrap_ui(UIC_FILES
  src/ui/moveit_rviz_plugin_frame.ui
)

# build is where the UI file outputs (ui_*.h) go.
include_directories(build)

# Plugin Source
set(SOURCE_FILES 
  src/interactive_marker_helpers.cpp
  src/background_processing.cpp
  src/planning_link_updater.cpp
  src/planning_scene_render.cpp
  src/robot_interaction.cpp
  src/planning_frame.cpp
  src/planning_display.cpp
)

include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  rosbuild_add_library_module(${PROJECT_NAME} ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES})
else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES})
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

# Link to boost libraries
rosbuild_link_boost(${PROJECT_NAME} thread)
         
# Link libraries
target_link_libraries(${PROJECT_NAME} ${OGRE_LIBRARIES} ${QT_LIBRARIES})
