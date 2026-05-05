include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)

add_library(robot_model_test
  fd_test.cpp
  fd_derivative_test.cpp
  id_test.cpp
  tauext_by_thrust_derivative_q_derivative_test.cpp
  tauext_by_thrust_q_derivative_comparison_test.cpp
  tauext_by_thrust_q_derivative_test.cpp
)
target_link_libraries(robot_model_test
  robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  pinocchio::pinocchio
)

add_rostest_gtest(pinocchio_robot_model_gtest
  pinocchio_robot_model.test
  pinocchio_robot_model_gtest.cpp
  )
set_target_properties(pinocchio_robot_model_gtest PROPERTIES EXCLUDE_FROM_ALL FALSE)
target_link_libraries(pinocchio_robot_model_gtest
  robot_model
  robot_model_test
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  pinocchio::pinocchio
  )
