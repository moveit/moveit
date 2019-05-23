set(MOVEIT_LIB_NAME author_information_widget)

add_library(${MOVEIT_LIB_NAME}
  author_information_widget.cpp
)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
