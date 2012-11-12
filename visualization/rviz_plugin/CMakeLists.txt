
# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  include/moveit/rviz_plugin/planning_display.h
  include/moveit/rviz_plugin/planning_frame.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES ${headers})

qt4_wrap_ui(UIC_FILES
  src/ui/moveit_rviz_plugin_frame.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES 
  src/background_processing.cpp
  src/planning_link_updater.cpp
  src/planning_scene_render.cpp
  src/planning_frame.cpp
  src/planning_display.cpp
)

set(MOVEIT_LIB_NAME moveit_rviz_plugin)
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES})
target_link_libraries(${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}_core LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})