add_rostest(flight_control.test ARGS headless:=true type:=tri)
add_rostest(flight_control.test ARGS headless:=true type:=quad)

# check the analytical gradients of the nlopt
add_rostest_gtest(delta_wrench_allocation_gradient_test
  wrench_allocation_gradient.test
  delta/wrench_allocation_gradient_test.cpp)
target_link_libraries(delta_wrench_allocation_gradient_test
  delta_controller
  ${catkin_LIBRARIES}
  ${NLOPT_LIBRARIES})
add_rostest(wrench_allocation_gradient.test ARGS type:=quad
  DEPENDENCIES delta_wrench_allocation_gradient_test)
