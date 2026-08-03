add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_srvs
  eigen_conversions
  tf
  tf2
  tf_conversions
)

find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Compile-time check that the ROS1 and ROS2 backings expose the same API.
add_executable(${PROJECT_NAME}_compile_test test/compile_test.cpp)
target_link_libraries(${PROJECT_NAME}_compile_test ${catkin_LIBRARIES})

# Checks the hand-written tf2 conversions against the tf1 functions they
# replace. ROS1-only: tf1 is what it compares against.
add_executable(${PROJECT_NAME}_tf_compat_test test/tf_compat_test.cpp)
target_link_libraries(${PROJECT_NAME}_tf_compat_test ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
