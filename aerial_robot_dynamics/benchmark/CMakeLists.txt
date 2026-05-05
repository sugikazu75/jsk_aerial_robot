find_package(benchmark REQUIRED)

add_executable(tauext_partial_q_derivative_hessian_benchmark
  tauext_partial_q_derivative_hessian_benchmark.cpp
)
target_link_libraries(tauext_partial_q_derivative_hessian_benchmark
  robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  pinocchio::pinocchio
  benchmark::benchmark
)

add_executable(tauext_partial_q_derivative_rnea_benchmark
  tauext_partial_q_derivative_rnea_benchmark.cpp
)
target_link_libraries(tauext_partial_q_derivative_rnea_benchmark
  robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  pinocchio::pinocchio
  benchmark::benchmark
)

add_executable(tauext_partial_q_derivative_numerical_benchmark
  tauext_partial_q_derivative_numerical_benchmark.cpp
)
target_link_libraries(tauext_partial_q_derivative_numerical_benchmark
  robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  pinocchio::pinocchio
  benchmark::benchmark
)
