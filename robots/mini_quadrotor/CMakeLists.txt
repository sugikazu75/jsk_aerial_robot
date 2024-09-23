cmake_minimum_required(VERSION 3.0.2)
project(mini_quadrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros_control
  roscpp
  tf
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(OpenMP REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model roscpp tf
  )

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(crocoddyl_test src/crocoddyl_test.cpp)
target_link_libraries(crocoddyl_test
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  pinocchio::pinocchio
  crocoddyl::crocoddyl
  )


install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(TARGETS crocoddyl_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# mujoco_model_convert(
#   ${PROJECT_SOURCE_DIR}
#   ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
