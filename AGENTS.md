# Notes for agents working in this repository

This stack flies real robots. The ROS1 build is the one they fly on.

## Before you start

This repository is mid-migration from ROS1 to ROS2, as a hybrid tree: both builds come
out of the same sources. **Read `docs/ros2_migration.md` first.** It records what is
already converted, the two decisions the rest follows from, the ROS2 behaviours that fail
silently rather than failing to compile, and the facts that were expensive to find.

## The hard constraint

ROS1 must keep building at every step:

```
cd /home/leus/ros/ros2_ws && catkin build --no-status     # must say: All 61 packages succeeded
```

A change that breaks ROS1 is a failure regardless of what it does for ROS2. Run this
before declaring anything done, not just the ROS2 build you were working on.

## How to work here

**Iterate against the compiler.** Nearly every real problem in this migration was found
by building, not by reading headers. The ROS2 compiler will tell you things no amount of
reading will.

**Do not trust a clean compile on anything involving coordinate frames, parameters, or
timing.** The failure mode in this codebase is not a crash, it is a robot that flies
subtly wrong. Where behaviour has to match the ROS1 original, check it numerically -
`aerial_robot_ros_compat/test/` has two examples of doing that, one comparing every tf
conversion against its tf1 counterpart and one checking that nested parameters actually
resolve.

**Say what you are unsure about.** "This compiles but I could not verify it against a
running system" is useful. Silence is not.

## Conventions

Follow what `aerial_robot_estimation` and `aerial_robot_model` already do: a `CMakeLists.txt`
that dispatches to `cmake/Ros1.cmake` or `cmake/Ros2.cmake`, a format 3 `package.xml` with
`condition="$ROS_VERSION == 1"` / `== 2`, message includes written out under an explicit
`#if` with the type namespace aliased, and `ros_compat::` in place of the roscpp and boost
spellings. The mapping table is in `docs/ros2_migration.md`.

The ROS2 build may legitimately be narrower than the ROS1 one. The first milestone -
mini_quadrotor hovering in MuJoCo - is met; anything that robot does not launch can still
stay ROS1-only with its dependencies conditioned.

## CI

`ros_test.yml` builds ROS1; `ros2_test.yml` builds ROS2 on Ubuntu 22.04 / humble
through `.ci/humble.sh`. The ROS2 job builds, generates the MuJoCo
models, runs the compat layer's parameter mapping test, and flies mini_quadrotor through
`hovering_check.py` - the same check the ROS1 rostest runs.

## Checking that it still flies

A green build says nothing about flight. The end-to-end check is:

```
ros2 launch mini_quadrotor bringup.launch.py rm:=false sim:=true mujoco:=true headless:=true
ros2 topic pub --once /quadrotor/teleop_command/start   std_msgs/msg/Empty "{}"
ros2 topic pub --once /quadrotor/teleop_command/takeoff std_msgs/msg/Empty "{}"
ros2 topic echo /quadrotor/uav/cog/odom --once --field pose.pose.position
```

It should settle at z = 0.6. Every bug that mattered in the launch work - a node in the
wrong namespace, a node nobody spun, an empty parameter set, a zero velocity in
`ground_truth` - compiled, loaded, logged nothing, and showed up only here.

## Formatting and commits

`pre-commit` runs clang-format, but the existing sources are not clang-format clean.
**Format only files you add.** When a commit touches existing files, use
`git commit --no-verify` and say why - otherwise a small fix arrives as a several-thousand
line reformat that buries it.

Migration work goes on the `ros2` branch (or `PR/ros2` where `ros2` already exists
upstream). **Never push.**
