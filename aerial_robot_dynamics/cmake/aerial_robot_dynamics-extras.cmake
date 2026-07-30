# This file is included at the end of the generated aerial_robot_dynamicsConfig.cmake.
#
# include/aerial_robot_dynamics/robot_model.h publicly includes pinocchio and
# OsqpEigen headers, so every dependent package needs them as well.  Those are
# plain CMake packages, hence catkin cannot propagate them by itself: find them
# here and merge them into the variables exported to the dependent packages.
# This is the catkin counterpart of jrl-cmakemodules' add_project_dependency().

include(CMakeFindDependencyMacro)

find_dependency(Eigen3)
find_dependency(pinocchio)
find_dependency(OsqpEigen)

foreach(_target Eigen3::Eigen pinocchio::pinocchio OsqpEigen::OsqpEigen)
  # linking against the imported target propagates its include directories,
  # compile definitions (PINOCCHIO_WITH_*) and compile features
  list(APPEND aerial_robot_dynamics_LIBRARIES ${_target})

  # for dependent packages which only use ${aerial_robot_dynamics_INCLUDE_DIRS}
  get_target_property(_include_dirs ${_target} INTERFACE_INCLUDE_DIRECTORIES)
  foreach(_dir ${_include_dirs})
    if(NOT _dir MATCHES "\\$<")  # a generator expression cannot be resolved here
      list(APPEND aerial_robot_dynamics_INCLUDE_DIRS ${_dir})
    endif()
  endforeach()
  unset(_include_dirs)
endforeach()
unset(_target)
unset(_dir)

list(REMOVE_DUPLICATES aerial_robot_dynamics_LIBRARIES)
list(REMOVE_DUPLICATES aerial_robot_dynamics_INCLUDE_DIRS)
