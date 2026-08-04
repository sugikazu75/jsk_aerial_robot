# ROS2 migration

Working notes for moving this stack to ROS2 while ROS1 keeps flying. Written to be
picked up by whoever - or whatever - continues the work.

## The hard constraint

**ROS1 must build at every step.** Real robots fly on it. After any change:

```
cd /home/leus/ros/ros2_ws && catkin build --no-status     # must say: All 61 packages succeeded
```

A change that breaks ROS1 is a failure regardless of what it does for ROS2. This is
why the migration is a hybrid tree rather than a fork, and why nothing is ever left
half-converted across a commit boundary.

## Where it stands

| Package | ROS1 | ROS2 |
| --- | --- | --- |
| aerial_robot_msgs | yes | yes |
| spinal (interfaces only) | yes | yes |
| aerial_robot_ros_compat | yes | yes |
| kalman_filter | yes | yes |
| aerial_robot_model | yes | yes |
| aerial_robot_estimation | yes | yes |
| mujoco (wrapper), mujoco_ros, mujoco_ros_control | yes | yes |
| aerial_robot_control | yes | yes |
| aerial_robot_base | yes | yes |
| aerial_robot_simulation | yes | yes |
| robots/* | yes | not started |

Next: mini_quadrotor's launch and config. The goal for the first
milestone is mini_quadrotor hovering in MuJoCo under ROS2; Gazebo comes later.

## A caution about "the build passes"

`aerial_robot_simulation` was first delivered as a hardware component that
implemented every `MujocoRosSystemInterface` method, registered with pluginlib,
and built cleanly under both versions - and could not have hovered, because it
had no estimator, no control law, and published no `ground_truth`. It was a
correct shell around nothing.

That is the shape of the risk here. A green build says the interfaces line up;
it says nothing about whether the behaviour came across. When reviewing a
converted package, compare it against what the ROS1 version *does* - which
topics it publishes, which state it drives - not just against whether it
compiles.

## Working with an agent on this

Most of the remaining work is mechanical enough to hand to a coding agent, with one
important caveat.

**An agent in a sandbox cannot run the full ROS1 regression.** `catkin build` needs to
write to `/home/leus/ros/ros2_ws/build/.built_by`, which sandboxed agents are not given.
They can build a single package with `--no-deps`, and they can build the ROS2 side, but
the check that actually matters - that all 61 ROS1 packages still succeed - has to be run
by a human afterwards. Do not accept "ROS1 builds" from an agent that could not run it.

What worked well when porting `aerial_robot_control`: point the agent at this document
and at an already-converted package to copy, name exactly which source files the ROS2
build should cover, and tell it explicitly what to do about code with no ROS2 equivalent
(guard the members, not the whole class) rather than leaving it to guess. Ask it to
report what it decided rather than mechanically translated, and what it could not verify.

Tell it not to commit. Review the working tree yourself.

## Build commands

ROS1 is plain catkin. ROS2 needs a clean environment, a separate build/install tree so
it does not collide with the catkin one, and `MUJOCO_DIR` because `mujoco_ros` reads
that variable directly instead of going through `find_package(mujoco)`:

```
env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  MUJOCO_DIR=/home/leus/ros/ros2_ws/install_ros2/mujoco bash -lc '
  source /opt/ros/humble/setup.bash; cd /home/leus/ros/ros2_ws
  colcon build --packages-select <pkg> --base-paths src \
    --build-base build_ros2 --install-base install_ros2'
```

`mujoco_ros` itself additionally needs `--cmake-args -DBUILD_TESTING=OFF`: its ROS2
tests reference the `pymujoco_ros` target that the same branch disables. That is an
inconsistency in the upstream branch we depend on, not something we introduced.

## The two decisions everything else follows from

### A compatibility layer, not a ROS-agnostic core

`aerial_robot_ros_compat` reproduces the slice of roscpp this stack uses, on top of
rclcpp. Under ROS1 every name in it is a plain alias onto roscpp, so the ROS1 build is
untouched and the shim cannot regress the flying robots.

The alternative - pulling a ROS-free core out and putting thin adapters around it, the
way mujoco_ros_pkgs does - was rejected because `ros::NodeHandle` is used here as a
*namespace handle* (`ros::NodeHandle(nh_, "controller")`) and is threaded through the
virtual `initialize()` of every pluginlib base class. Splitting the core would change
those signatures and break every in-tree and out-of-tree robot package at once.

The namespace is `ros_compat`, deliberately not `ros`: under ROS1 roscpp is still
included and owns that namespace, and shadowing it would leave it ambiguous which
implementation a given call site resolves to.

### The whole stack moved from tf1 to tf2 in one commit

ROS2 has no `tf` package. `tf::Vector3` and `tf2::Vector3` are separate classes rather
than aliases - the same bullet-derived code copied twice - so one will not bind to the
other and there is no working half-converted state. 408 sites across 44 files went at
once.

The datatypes were a rename. The ~70 conversion helpers were rewritten by hand in
`tf_compat.h` rather than delegating to tf2's `toMsg`/`fromMsg`, whose overload sets
differ between distributions in ways that silently pick a different intermediate.

**All 27 helpers are checked against the tf1 function they replace**, over 500
orientations spanning the RPY range, by `aerial_robot_ros_compat_tf_compat_test`, which
builds and runs as part of the ROS1 build. This is not ceremony: a transposed rotation
or a swapped quaternion storage order compiles fine and produces a robot that flies
subtly wrong. Eigen's quaternion constructor takes `w` first while the message stores
`x,y,z,w`, to name the obvious trap.

One deliberate behaviour change: `poseMsgToTf` treats an all-zero quaternion as
identity. tf1 normalised whatever it was handed, which makes that case NaN, and a
default-constructed Pose reaching the estimator should not poison the state.

## Things that fail silently

These are the ones to be careful about, because the compiler will not catch them.

**Parameter namespace mapping.** Every `NodeHandle child(nh_, "estimation");
child.param("mode", ...)` has to come out as the ROS2 parameter `estimation.mode`. If
the joining is wrong, the code compiles, runs, and quietly falls back to defaults for
every parameter - which on a flying robot means default gains rather than an error.
Verified at runtime by `aerial_robot_ros_compat_param_mapping_test`:

```
<install>/lib/aerial_robot_ros_compat/aerial_robot_ros_compat_param_mapping_test \
  --ros-args --params-file <install>/share/aerial_robot_ros_compat/test/param_mapping_test.yaml
```

Re-run it after touching `NodeHandle`.

**Undeclared parameters.** Nodes must be created with
`automatically_declare_parameters_from_overrides(true)` - which `ros_compat::createNode`
does - or every YAML-supplied parameter looks undeclared, `hasParam` returns false, and
the many `if (nh.hasParam(...))` branches take their default path.

**A default-constructed NodeHandle.** roscpp's `ros::NodeHandle nh;` reaches the
process's node and the global parameter server. ROS2 has no global server, so the compat
default binds to the node registered by `setGlobalNode`. Without that, `RobotModel`'s two
default-constructed handles read nothing.

**`_ONCE` logging.** A mechanical rename that drops the `_ONCE` turns a startup message
into one printed every control cycle.

**Blocking service calls.** rclcpp has no synchronous call: spinning a node from inside
its own callback deadlocks, because the executor is already in that callback. roscpp had
no such restriction and `vo.cpp`'s reset does exactly that. `ros_compat::ServiceClient`
gives the call its own node and executor so it blocks only the calling thread, and adds a
timeout roscpp did not have. **This is a reconstruction of the old behaviour, not the same
mechanism - it deserves checking against a running system rather than trusting because it
links.**

## ROS2 facts worth not rediscovering

Found by running the tools rather than reading documentation.

- rosidl parses the bare `time` type but has no IDL mapping for it - generation dies with
  `KeyError: 'time'`. Seven spinal messages hit this; their ROS2 variants live in
  `msg/ros2/` and use `builtin_interfaces/Time`.
- rosidl takes the interface namespace from the immediate parent directory, so
  `msg/ros2/Imu.msg` would register as `spinal/ros2/Imu`. Per-version variants are staged
  into the build tree under a directory actually called `msg/` and passed with rosidl's
  `"<base>:<relpath>"` tuple form.
- camelCase message fields are rejected. Only `SimpleImu.msg` is affected; it is
  firmware-only and left out of the ROS2 build.
- A package that also generates interfaces gets `include/<pkg>` on its consumers' include
  path, so its hand-written headers must install to `include/<pkg>/<pkg>/`. The interface
  target also takes the project name, which is why `aerial_robot_model`'s library is
  `aerial_robot_model_lib` under ROS2.
- Include paths cannot be built from macro arguments: the preprocessor separates tokens
  when forming a header name, so `<pkg/Camel.h>` expands to `<geometry_msgs/ Vector3Stamped.h>`.
  Message includes are written out under an explicit `#if`; only the type namespace is aliased.
- `ROS_VERSION` is an environment variable and invisible to the preprocessor. Version
  selection comes from `-DAERIAL_ROBOT_ROS_VERSION` passed by each package's cmake.
- The ROS2 header name is rosidl's snake_case of the message name - `UInt8` becomes
  `u_int8.hpp`, not `uint8.hpp`. Check the file exists rather than deriving it.
- `tf2_ros/transform_broadcaster.h` does not pull in the LinearMath types the way
  `tf/transform_broadcaster.h` did, and `tfScalar` is a separate alias from the datatypes.
- pluginlib's `.hpp` headers exist under both versions; the `.h` shims are ROS1-only.

## spinal and ros1_bridge

The board package stays ROS1 - it is rosserial-bound - and ROS2 reaches it through
ros1_bridge. Its ROS2 build produces the interfaces only, and **the package keeps the name
`spinal` on both sides**: that makes ros1_bridge pair `spinal/Imu` with `spinal/msg/Imu`
by itself with no `mapping_rules.yaml`, and keeps the `spinal/MotorInfo[] motor_info`
self-references inside the `.msg` files valid unedited.

ros1_bridge has no humble apt package and must be built from source against both
distributions. Not needed until a real flight controller is in the loop - the MuJoCo
simulation does not start the serial bridge.

## aerial_robot_simulation

Not a port; the structure had to change. ROS1 ros_control lets a controller be
templated on the hardware interface type and receive an arbitrary C++ object:
`MujocoAttitudeController::init` received a `StateEstimate*`, and the controller
wrote back into the hardware through `spinal_interface_->onGround(...)`. ROS2
ros2_control passes only **named doubles** between hardware and controller, so
neither direction survives.

**The attitude estimator and the flight control core are therefore folded into
the ros2_control hardware component** (`AerialRobotMujocoSystem`, with
`AerialRobotSpinal` holding both). `onGround` then needs no interface at all.
The alternative - flattening the estimator's inputs and outputs to named
doubles and keeping a separate controller plugin - needs artificial interfaces
to express what is really shared state.

`read()` takes the `fc` site pose and the acc/gyro/mag sensors from MuJoCo,
drives the estimator, and publishes `ground_truth` and `mocap/pose`.
**Publishing `ground_truth` is not optional**: mini_quadrotor brings up with
`sim_estimate_mode` = GROUND_TRUTH, so that topic is the only pose its estimator
has. A hardware component that builds and loads but does not publish it will
start cleanly and never fly.

`write()` steps the controller and applies its rotor forces. The command
interfaces stay exported, but in the spinal configuration the controller is the
authority, as it is on the board.

The firmware sources are **not** rewritten. They are shared with the flight
controller and the generated `ros_lib` headers must stay byte-identical.
`spinal_ros2_shim.h` supplies what their `-DSIMULATION` path expects under ROS2:
the `ros::` types backed by the compat layer, and the messages hoisted from
`spinal::msg` back into `spinal`. That header injects into `ros::`, which is
what the rest of this migration refuses to do - the difference is that it only
happens under ROS2, where roscpp does not exist and there is nothing to be
ambiguous with.

`mujoco_ros_control` hands the component a `LifecycleNode`, an unrelated type to
the `rclcpp::Node` the compat `NodeHandle` is built on. The flight controller
gets its own node carrying the lifecycle node's name, namespace and **parameter
overrides**, so its parameters resolve from the same config. If the compat layer
ever needs to accept lifecycle nodes generally, that is the call site to
generalise from.

## Environment

Both `/opt/ros/one` (ROS-O) and `/opt/ros/humble` are installed. The workspace is a
catkin_tools workspace; the ROS2 build lives in `build_ros2/` and `install_ros2/`.

Extra apt packages installed for humble beyond the base desktop: `ros2-control`,
`ros2-controllers`, `camera-info-manager`, `image-transport`, `joint-state-publisher`,
`xacro`, `py-binding-tools`, `geographic-msgs`, `geodesy`, `joy`.

`nlopt` is **not** in apt and will need a source build before `hydrus_xi` and `dragon`
can be ported.

## Known rough edges

`aerial_robot_estimation` exports its targets but its own headers do not arrive on
consumers' interface include directories, so `aerial_robot_control`'s ROS2 cmake has to
add `${aerial_robot_estimation_INCLUDE_DIRS}` explicitly. Worth fixing in
`aerial_robot_estimation` itself rather than repeating the workaround in every downstream
package.

GPS waypoint navigation is ROS1-only. `flight_navigation.cpp` guards the paths that use
`geodesy::toMsg` and `sensor_plugin::Gps::wgs84ToNedLocalFrame`; under ROS2 the mode logs a
warning and disables itself. Fine for the MuJoCo milestone, which has no GPS, but it is a
real gap for outdoor flight later.

Dynamic gain tuning is ROS1-only. The PID and LQI `dynamic_reconfigure` servers and
callbacks are guarded; the controllers themselves are in the ROS2 build. Hovering does not
need them, but tuning on ROS2 will need parameter callbacks first.

## Still open

- The ROS1 YAML config files need a `<node>: ros__parameters:` wrapper to load as ROS2
  parameter files. Belongs with the mini_quadrotor launch conversion.
- `dynamic_reconfigure` (9 sites) and `nodelet` (3 sites) have no ROS2 equivalent. In
  `aerial_robot_estimation` they happened to sit entirely inside code the first milestone
  does not need; in `aerial_robot_control` they are the gain-tuning paths and need
  replacing with parameter callbacks eventually.
- ROS2 CI alongside the existing ROS1 hovering tests in `.github/workflows/ros_test.yml`.
