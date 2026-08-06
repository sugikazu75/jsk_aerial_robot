#!/bin/bash
#
# Build the ROS2 half of the stack on Ubuntu 22.04 / humble.
#
# The counterpart of .travis.sh, which builds the ROS1 half. Kept separate
# rather than branching inside that script: the two builds share no tooling -
# colcon against catkin, vcs against wstool, a different dependency list - and
# the ROS1 script is what the flying robots' CI runs.
#
# It builds and generates the MuJoCo models, as the catkin job does, but does
# not yet fly the robots: the ROS1 hovering test is a rostest driving a rospy
# script, and neither has a ROS2 counterpart yet. Flying under ROS2 is verified
# by hand - see docs/ros2_migration.md for the takeoff check.

set -ex

# The bare ubuntu image has no TERM and tzdata will otherwise stop and ask which
# continent this is, which in CI means hanging until the job times out.
export DEBIAN_FRONTEND=noninteractive

# libglfw3-dev is for mujoco_ros, which declares no rendering dependency of its
# own and picks a backend by what it finds. With none present it falls through
# to OSMesa and then fails to link against it - upstream tests `!OSMesa_FOUND`,
# which is not CMake, so the not-found branch is never taken. GLFW is also the
# backend a developer machine has, so CI builds what has been verified.

apt-get update -qq
apt-get install -y -q --no-install-recommends \
    ca-certificates curl gnupg lsb-release sudo git build-essential tzdata \
    libglfw3-dev

# ROS2 humble
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
apt-get update -qq
apt-get install -y -q ros-humble-ros-base \
    python3-colcon-common-extensions python3-rosdep python3-vcstool

rosdep init
rosdep update --rosdistro humble

source /opt/ros/humble/setup.bash

# Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
cp -r "${CI_SOURCE_PATH}" "src/${REPOSITORY_NAME}"
vcs import src < "src/${REPOSITORY_NAME}/aerial_robot_humble.rosinstall" --recursive

# rosdep resolves the humble-conditioned dependencies out of the format 3
# package.xml files, including the mesh toolchain mujoco_ros_utils needs to
# generate the MuJoCo models. --skip-keys is for what is built from source in
# this workspace rather than installed: nlopt and osqp-eigen from
# aerial_robot_3rdparty, mujoco from its own wrapper.
rosdep install --from-paths src -y -r --ignore-src --rosdistro humble \
    --skip-keys "nlopt osqp osqp-eigen mujoco mujoco_ros_utils"

# mujoco_ros reads MUJOCO_DIR directly instead of going through
# find_package(mujoco), so the wrapper has to be built and located first.
colcon build --packages-select mujoco --base-paths src
export MUJOCO_DIR=$PWD/install/mujoco

# --packages-up-to rather than everything in the workspace, for two reasons.
# It states what the ROS2 build is meant to cover, so adding a package here is a
# deliberate act; and the workspace still contains catkin-only packages that
# colcon would otherwise pick up and fail on - osqp_slsqp in
# aerial_robot_3rdparty, and the ROS1-only extras in mujoco_ros_pkgs.
#
# mujoco_ros's own tests reference the pymujoco_ros target that the same branch
# disables; an inconsistency upstream, not something introduced here.
colcon build --base-paths src --cmake-args -DBUILD_TESTING=OFF --packages-up-to \
    aerial_robot_base \
    aerial_robot_simulation \
    mini_quadrotor \
    hydrus \
    hydrus_xi \
    dragon \
    gimbalrotor \
    mujoco_ros_utils

# Now generate the MuJoCo models, which the catkin job produces as part of its
# build. Under ROS2 it has to be a second step: a robot's xacro says
# $(find <itself>), which resolves through the ament index, so the package has
# to be installed first - it cannot generate its own model during its own build.
#
# The second colcon build then picks the models up and installs them.
# --cmake-force-configure is not optional: whether a model is installed is
# decided at *configure* time, and the first pass decided "no model here".
source install/setup.bash
for robot in mini_quadrotor hydrus hydrus_xi dragon; do
    ros2 run mujoco_ros_utils mujoco_model_generator.py \
        src/${REPOSITORY_NAME}/robots/${robot}/config/mujoco_model.yaml \
        --package-dir ${robot}=src/${REPOSITORY_NAME}/robots/${robot}
done

colcon build --base-paths src --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF --packages-select \
    mini_quadrotor \
    hydrus \
    hydrus_xi \
    dragon

# And assert they arrived. A generation step that quietly produced nothing would
# otherwise look exactly like a pass, and the packages only warn about a missing
# model - which is right for a developer and wrong for CI.
for robot in mini_quadrotor hydrus hydrus_xi dragon; do
    if ! find "install/${robot}/share/${robot}/mujoco" -name robot.xml | grep -q .; then
        echo "no MuJoCo model was installed for ${robot}" >&2
        exit 1
    fi
done

# The compat layer's own checks: the tf conversions against their tf1
# counterparts, and the nested-parameter mapping the whole stack rests on.
source install/setup.bash
install/aerial_robot_ros_compat/lib/aerial_robot_ros_compat/aerial_robot_ros_compat_param_mapping_test \
    --ros-args --params-file \
    install/aerial_robot_ros_compat/share/aerial_robot_ros_compat/test/param_mapping_test.yaml
