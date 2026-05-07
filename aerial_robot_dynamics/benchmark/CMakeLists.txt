find_package(benchmark REQUIRED)

add_executable(tauext_partial_q_derivative_hessian_benchmark
  tauext_partial_q_derivative_hessian_benchmark.cpp
)
configure_aerial_robot_dynamics_target(tauext_partial_q_derivative_hessian_benchmark)
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
configure_aerial_robot_dynamics_target(tauext_partial_q_derivative_rnea_benchmark)
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
configure_aerial_robot_dynamics_target(tauext_partial_q_derivative_numerical_benchmark)
target_link_libraries(tauext_partial_q_derivative_numerical_benchmark
  robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  pinocchio::pinocchio
  benchmark::benchmark
)

install(TARGETS
  tauext_partial_q_derivative_hessian_benchmark
  tauext_partial_q_derivative_rnea_benchmark
  tauext_partial_q_derivative_numerical_benchmark
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
