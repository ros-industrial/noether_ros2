cmake_minimum_required(VERSION 3.5)
project(noether_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_AUTOMOC ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(noether_tpp REQUIRED)
find_package(pluginlib)
find_package(rclcpp REQUIRED)
find_package(rviz_common)
find_package(tf2_eigen REQUIRED)
find_package(yaml-cpp REQUIRED)

# Generate the messages
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/LocatedVector.msg"
  "msg/ToolPath.msg"
  "msg/ToolPaths.msg"
  "srv/PlanToolPath.srv"
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
)
rosidl_get_typesupport_target(interfaces_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Library
add_library(${PROJECT_NAME}_conversions SHARED src/conversions.cpp src/located_vector_generators.cpp)
target_include_directories(
  ${PROJECT_NAME}_conversions PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                     "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}_conversions noether::noether_tpp ${interfaces_target})
ament_target_dependencies(${PROJECT_NAME}_conversions tf2_eigen rclcpp)

# Tool path planning server
add_executable(${PROJECT_NAME}_tool_path_planning_server src/tool_path_planning_server.cpp)
target_link_libraries(${PROJECT_NAME}_tool_path_planning_server ${PROJECT_NAME}_conversions yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_tool_path_planning_server rclcpp)

# Located vector Rviz tool
add_library(${PROJECT_NAME}_located_vector_tool SHARED src/rviz_located_vector_tool.h src/rviz_located_vector_tool.cpp ${moc})
target_link_libraries(${PROJECT_NAME}_located_vector_tool ${interfaces_target})
ament_target_dependencies(${PROJECT_NAME}_located_vector_tool rviz_common pluginlib)

if(${ENABLE_TESTING})
  enable_testing()
  add_subdirectory(test)
endif()

# Install the headers
install(DIRECTORY include/ DESTINATION include/)

# Install the library
install(
  TARGETS ${PROJECT_NAME}_conversions ${PROJECT_NAME}_located_vector_tool
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

# Install the executable
install(TARGETS ${PROJECT_NAME}_tool_path_planning_server DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  noether_tpp
  pluginlib
  rosidl_default_runtime
  rviz_common
  tf2_eigen
)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_package()
