cmake_minimum_required(VERSION 3.0.2)
project(aeroslam_livox_bridge)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

# Optional dependency: only available when the frontier modules are fetched
# (./scripts/setup.sh --with-frontier). The node is skipped otherwise.
find_package(livox_ros_driver QUIET)

catkin_package()

if(livox_ros_driver_FOUND)
  include_directories(${livox_ros_driver_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include)

  add_executable(livox_sim_bridge src/livox_sim_bridge.cpp)
  target_link_libraries(livox_sim_bridge ${catkin_LIBRARIES})
  add_dependencies(livox_sim_bridge ${catkin_EXPORTED_TARGETS})
  if(TARGET livox_ros_driver_generate_messages_cpp)
    add_dependencies(livox_sim_bridge livox_ros_driver_generate_messages_cpp)
  endif()
  install(TARGETS livox_sim_bridge
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(STATUS "aeroslam_livox_bridge: livox_ros_driver not found, "
                 "skipping livox_sim_bridge (see src/frontier/README.md)")
endif()
