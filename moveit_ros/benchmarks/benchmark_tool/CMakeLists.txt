qt4_wrap_cpp(MOC_FILES include/main_window.h include/trajectory.h)
qt4_wrap_ui(UIC_FILES
  src/ui/main_window.ui
  src/ui/run_benchmark_dialog.ui
  src/ui/robot_loader.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(SYSTEM
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS})

set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/job_processing.cpp
  src/ui_utils.cpp
  src/frame_marker.cpp
  src/trajectory.cpp
  src/tab_states_and_goals.cpp
  src/tab_trajectories.cpp
)

add_executable(benchmark_tool ${SOURCES} ${MOC_FILES} ${UIC_FILES})

target_link_libraries(benchmark_tool 
  ${catkin_LIBRARIES} 
  ${OGRE_LIBRARIES} 
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    benchmark_tool
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION include)

