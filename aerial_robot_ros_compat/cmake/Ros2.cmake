find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_compile_definitions(${PROJECT_NAME} INTERFACE AERIAL_ROBOT_ROS_VERSION=2)
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp)

# Compile-time check that the ROS1 and ROS2 backings expose the same API.
add_executable(${PROJECT_NAME}_compile_test test/compile_test.cpp)
target_link_libraries(${PROJECT_NAME}_compile_test ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_compile_test
  rclcpp geometry_msgs sensor_msgs std_srvs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
