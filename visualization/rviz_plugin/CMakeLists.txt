
# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  include/moveit/rviz_plugin/planning_display.h
  include/moveit/rviz_plugin/planning_frame.h
  include/moveit/rviz_plugin/planning_scene_render.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES ${headers})

qt4_wrap_ui(UIC_FILES
  src/ui/moveit_rviz_plugin_frame.ui
)

# build is where the UI file outputs (ui_*.h) go.
#include_directories(build)

# Plugin Source
set(SOURCE_FILES 
  src/background_processing.cpp
  src/planning_link_updater.cpp
  src/planning_scene_render.cpp
  src/planning_frame.cpp
  src/planning_display.cpp
)

set(MOVEIT_LIB_NAME moveit_rviz_plugin)
add_library(${MOVEIT_LIB_NAME} ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
