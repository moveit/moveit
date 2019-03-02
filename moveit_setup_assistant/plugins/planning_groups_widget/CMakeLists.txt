set(MOVEIT_LIB_NAME planning_groups_widget)

find_package(catkin REQUIRED COMPONENTS
  rviz
  urdf
  rosconsole
  roscpp
  srdfdom
  moveit_core
)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set(HEADERS
  group_edit_widget.h
  kinematic_chain_widget.h
  planning_groups_widget.h
)

add_library(${MOVEIT_LIB_NAME}
  group_edit_widget.cpp
  kinematic_chain_widget.cpp
  planning_groups_widget.cpp
  )

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)

install(
  FILES
    planning_groups_widget.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})

