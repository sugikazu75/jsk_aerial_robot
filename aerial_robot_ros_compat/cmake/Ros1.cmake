add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Compile-time check that the ROS1 and ROS2 backings expose the same API.
add_executable(${PROJECT_NAME}_compile_test test/compile_test.cpp)
target_link_libraries(${PROJECT_NAME}_compile_test ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
