set(MOVEIT_LIB_NAME ros_controllers_widget)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set(HEADERS
  controller_edit_widget.h
  ros_controllers_widget.h
)

add_library(${MOVEIT_LIB_NAME}
  controller_edit_widget.cpp
  ros_controllers_widget.cpp
  )

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
