cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_dynamics)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(urdfdom_headers REQUIRED)

if(OPENMP_FOUND)
  message(STATUS "OpenMP found.")
  MESSAGE("OpenMP found.")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -DNDEBUG")

option(BUILD_TESTING "Build Test Code" OFF)
option(BUILD_BENCHMARK "Build Benchmark Code" OFF)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_model
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  )

add_library(robot_model src/robot_model.cpp)
target_link_libraries(robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  pinocchio::pinocchio
  )

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_BENCHMARK)
  add_subdirectory(benchmark)
endif()

if(BUILD_TESTING AND CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
