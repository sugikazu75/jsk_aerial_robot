#!/usr/bin/env bash

set -eu

dir=$(rospack find mini_quadrotor)
patch "${dir}/mujoco/robot.xml" < "${dir}/config/mujoco_patch/mini_quadrotor.diff"
