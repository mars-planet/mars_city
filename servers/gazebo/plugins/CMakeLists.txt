cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIR}
)
link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${SDF_LIBRARY_DIR}
)

add_library(swivel_plugin SHARED SwivelPlugin.cc SwivelPlugin.hh)
target_link_libraries(swivel_plugin ${GAZEBO_libraries} )
# target_link_libraries(swivel_plugin libgazebo gazebo_sensors ${ogre_libraries})
install (TARGETS swivel_plugin DESTINATION ./)

add_library(track_drive_plugin SHARED TrackDrivePlugin.cc TrackDrivePlugin.hh)
target_link_libraries(track_drive_plugin ${GAZEBO_libraries})
