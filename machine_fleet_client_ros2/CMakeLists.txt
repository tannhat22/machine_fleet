cmake_minimum_required(VERSION 3.5.0)
project(machine_fleet_client_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)

  set(dependencies
    rclcpp
    geometry_msgs
    machine_fleet_msgs
    machine_fleet
  )

  foreach(dep ${dependencies})
    find_package(${dep} REQUIRED)
  endforeach()

  #=============================================================================

  include_directories(include)
  add_executable(machine_fleet_client_ros2
    src/main.cpp
    src/client_node.cpp
    src/client_node_config.cpp
  )
  ament_target_dependencies(machine_fleet_client_ros2
    ${dependencies}
  )

  #=============================================================================

  install(TARGETS machine_fleet_client_ros2
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()

else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()
