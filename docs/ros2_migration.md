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
| robots/mini_quadrotor | yes | yes |
| robots/hydrus | yes | yes |
| robots/gimbalrotor | yes | builds, cannot fly |
| other robots/* | yes | not started |

**mini_quadrotor and hydrus both hover in MuJoCo under ROS2.**

```
ros2 launch mini_quadrotor bringup.launch.py rm:=false sim:=true mujoco:=true headless:=true
ros2 topic pub --once /quadrotor/teleop_command/start   std_msgs/msg/Empty "{}"
ros2 topic pub --once /quadrotor/teleop_command/takeoff std_msgs/msg/Empty "{}"
```

Measured after takeoff: `uav/cog/odom` z = 0.6000 against a 0.6 m target, xy held
to about 2 cm, attitude angles at 1e-5 rad, and the four rotors at 2.61/2.71 N -
10.6 N in total, which is the 1.084 kg model's weight.

hydrus is the same, with `ros2 launch hydrus bringup.launch.py real_machine:=false
simulation:=true mujoco:=true`: it reaches its quad shape, climbs, and the
navigator prints `Hover!!!` with z = 0.600 against 0.6 and xy within 3 cm of the
takeoff point. Note that `ground_truth` is the **fc site** and `uav/cog/odom` is
the **cog**; on hydrus those are tens of centimetres apart, so the two disagreeing
is geometry, not error.

Gazebo under ROS2 comes later; `mujoco:=true` is currently the only simulation
backend, and the launch says so rather than starting something that cannot work.

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

**Whole numbers in the config files.** rclcpp is strictly typed where roscpp's
parameter server was not. `limit_sum: 3` in FlightControl.yaml becomes an
*integer* parameter, and `rclcpp::Node::get_parameter(name, double&)` **throws**
on it - uncaught, so the node dies. FlightControl.yaml alone has eight of them.
`ros_compat::NodeHandle::getParam` therefore widens int to double the way roscpp
did, and accepts a double for an integer parameter only when it is exactly
integral. `bool` stays strict: a numeric parameter quietly becoming `true` is
how a robot ends up in the wrong mode. Covered by
`aerial_robot_ros_compat_param_mapping_test`.

**Clock sources on `ros_compat::Time`.** `rclcpp::Time` carries a clock source
and throws when two with different sources are compared or subtracted. Its own
constructors default to `RCL_SYSTEM_TIME`, while anything from a node or a
message header is `RCL_ROS_TIME` - so `ros_compat::Time prev_;` or
`ros_compat::Time(0)`, of which this stack has several, throws the first time it
meets a real stamp. The state estimator's publish throttle did exactly that, one
control cycle after the robot announced it was ready for takeoff. The compat
`Time` now declares its constructors explicitly, defaulting to `RCL_ROS_TIME`;
inheriting them with `using` and adding overloads does not work, because an
inherited constructor with defaulted trailing parameters is a separate signature
and every addition comes out ambiguous rather than preferred.

**pluginlib library paths.** ROS1 wants `<library path="lib/lib<name>">`, ROS2
wants `<library path="<name>">`. A ROS1 path under ROS2 fails at *runtime* with
"Could not find library corresponding to plugin", and the stack's loaders log
the failure and carry on with a null pointer, so the crash arrives somewhere
else entirely. Each package ships a `*.ros2.xml` variant and points its
`cmake/Ros2.cmake` at it.

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
**Publishing `ground_truth` is not optional, and neither is its twist**:
mini_quadrotor brings up with `sim_estimate_mode` = GROUND_TRUTH, and the mocap
sensor plugin's `groundTruthCallback` takes velocity *straight out of*
`twist.twist.linear` - it does not differentiate the pose. A component that
publishes pose only does not degrade the velocity estimate, it pins it at zero;
the position gains then fly against a dead D term and the robot walks away
across the floor while holding perfect attitude. That is what it did.

The linear velocity is world frame, the angular velocity is the **fc frame**
(it is the gyro reading, as the ROS1 comment shouts), and publication is
throttled to `simulation/ground_truth_pub_rate`. This is the contract the ROS1
*Gazebo* hardware sim publishes to. The ROS1 *MuJoCo* one advertises
`ground_truth` and never publishes on it - its estimator runs off `mocap/pose`
instead - which is why the gap could not have been found by matching the MuJoCo
original.

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
gets its own node carrying the lifecycle node's parameter overrides, so its
parameters resolve from the same config. If the compat layer ever needs to
accept lifecycle nodes generally, that is the call site to generalise from.

Three things about that node were wrong in a way no build could show, and all
three are worth remembering because the next hardware component will meet them:

- **Its namespace.** mujoco_ros creates plugin nodes at
  `/<mujoco server>/<plugin>`, which is not where the robot is. Taking the
  lifecycle node's namespace put the simulated board's `four_axes/command`,
  `rpy/gain` and `motor_info` under `/mujoco_server`, where
  aerial_robot_control never looked. The namespace now comes from
  mujoco_ros_control's own `namespace` parameter - the same one it uses to find
  `robot_state_publisher` and to place the controller manager, so the two cannot
  disagree.
- **Nothing spun it.** mujoco_ros_control adds its own node and the controller
  manager to the environment's executor; it knows nothing about ours. The
  component now gives the node its own executor and thread. Under ROS1 this
  never came up: the hardware sim's NodeHandle belonged to a node the simulator
  was already spinning.
- **`NodeOptions::parameter_overrides()` is not the parameters.** It holds only
  what was set programmatically on the options object. Everything from a
  `--params-file`, which is how the whole simulation is configured, arrives
  through the node's *arguments* and never appears there. Copying the options
  across handed the flight controller an empty parameter set, with every
  `getParam` silently taking its default. Use
  `get_node_parameters_interface()->get_parameter_overrides()`, which is the
  resolved set.

## Environment

Both `/opt/ros/one` (ROS-O) and `/opt/ros/humble` are installed. The workspace is a
catkin_tools workspace; the ROS2 build lives in `build_ros2/` and `install_ros2/`.

Extra apt packages installed for humble beyond the base desktop: `ros2-control`,
`ros2-controllers`, `camera-info-manager`, `image-transport`, `joint-state-publisher`,
`xacro`, `py-binding-tools`, `geographic-msgs`, `geodesy`, `joy`.

`nlopt` is not in apt, but it does not need to be: `aerial_robot_3rdparty` builds it
through `ExternalProject`. Porting that package is what `hydrus_xi` and `dragon` need.

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

## Launch and config under ROS2

There is no global parameter server, so the ROS1 pattern - load a pile of yaml
into a *namespace* and let every node in it read what it wants - has no direct
translation. Each node needs its own copy, and a ROS2 parameter file has to name
the node it is for.

**The ROS1 yaml files were not wrapped or duplicated.** `bringup.launch.py`
merges them at launch time, in the order roslaunch would have applied them and
with the same deep merge `rosparam` did for stacked files, then writes one
parameter file under a `/**: ros__parameters:` header. Checking in wrapped
copies would have meant two sets of gains that can drift apart silently, and the
robots fly on the ROS1 ones. A plain `dict.update()` would have been wrong too:
three of the files carry a top-level `sensor_plugin:` mapping for different
sensors, and the last one loaded would be the only one left.

The split follows who reads what, which is not how ROS1 arranged it:

- Everything the base node reads - model, motors, battery, control, estimation,
  navigation, plus the per-sensor noise config `sensors.launch.xml` used to load
  - goes into that one merged file.
- The `simulation:` block belongs to the **hardware component**, not the base
  node, and reaches it through mujoco_ros_control's plugin node.
  `mujoco.launch.py` stacks `aerial_robot_simulation/config/Mujoco.yaml` and the
  robot's `Simulation.yaml` exactly as the two rosparam loads did.
- mujoco_ros reads its plugin list from a parameter file keyed by node name, and
  parameter files cannot carry substitutions, so `mujoco.launch.py` renders one
  at launch. That is the same job `<rosparam subst_value="true">` did.

**The merging lives in `aerial_robot_ros_compat.launch_config`**, a python
module the compat package installs alongside its headers - the launch-side half
of the same job. Robots import `write_parameter_file`; `mujoco.launch.py` uses
`load_merged` with a `subtree` to lift just the `simulation:` block out.

**Servo joints are held by the hardware component**, not by a controller. Under
ROS1 `servo_bridge` - the interface the stack reaches its servos through on both
the real machine and in simulation - loaded an
`effort_controllers/JointPositionController` per joint through the controller
manager and published the initial angle to it. ROS2 has no such controller to
load, so `AerialRobotMujocoSystem` runs the PID itself, from the very same
`simulation:` block of `Servo.yaml`, and accepts new targets on the `<group>_ctrl`
topic servo_bridge listened to. It also publishes `joint_states`, which
`joint_state_controller` used to: without it a transformable robot's kinematics
stay at the pose it launched in and its LQI gains are computed for a shape it is
not in. **A real-machine servo_bridge - converting joint commands into servo
units for rosserial - is a separate job and still to do.**

`hydrus` starts with its joints at zero, which is a straight line and not a
stabilisable pose; the LQI generator says so twice while the arms drive to 1.57
rad, and then stops. That transient is expected.

**No flight controller is spawned either.** In the spinal configuration the flight control
core lives inside the hardware component, and mujoco_ros_control activates the
component itself, so `read()`/`write()` run with nothing loaded.
`joint_state_controller` has no counterpart to spawn either - the component
exports thrust/effort interfaces and `joint_state_broadcaster` wants position -
so rotor frames come from `rotor_tf_publisher`, as they do in the ROS1 sim.

The MuJoCo model needs one thing at install time. `mujoco/robot.xml` is
generated by the ROS1 build and pulls in the shared world through a path
relative to the *source* tree, which does not survive installation.
`mini_quadrotor/cmake/Ros2.cmake` resolves that path while the source layout is
still real and installs a self-contained pair: `robot.xml` with a bare
`<include file="world.xml"/>`, and the world beside it.

## gimbalrotor

Builds under ROS2 - the robot model, controller and navigator plugins all load
and initialise under `aerial_robot_base_node` with the robot's own config - and
**cannot fly there**. `bringup.launch.py` brings up the base node and the
description, and raises if asked to simulate. Three things are missing, none of
them launch work:

- **No MuJoCo model.** This robot only ever simulated in Gazebo; it has no
  `config/mujoco_model.yaml` and no generated `mujoco/`.
- **No gimbal servos in the simulation.** `AerialRobotMujocoSystem` handles
  rotor actuators only. A vectored robot needs the servo joints exported as
  ros2_control interfaces and driven from `gimbals_ctrl`.
- **No `servo_bridge`.** The ROS1 launch always starts it, and it is outside
  aerial_robot_model's ROS2 build.

The conversion itself was routine - the compat spellings, message includes under
an `#if`, and a `*.ros2.xml` per plugin description. `gimbalrotor_robot_model`
needed nothing at all: it is KDL and pluginlib, with no ROS in it.

## Still open

- Gazebo has no ROS2 bringup. `bringup.launch.py` raises rather than starting
  half of one.
- `simple_demo.py` and the real-machine sensor drivers (spinal serial bridge,
  mocap, livox, fast_lio) are ROS1-only, so `rm:=true` under ROS2 brings up the
  base node and the model and nothing else. The sensor *config* is still loaded,
  because the estimator reads it either way.
- The rviz config is a ROS1 one (`rviz/Displays` and friends), so
  `aerial_robot_model.launch.py` starts rviz2 without it. Bringup defaults to
  `headless:=true`, as the ROS1 one does, so this is not on the hovering path.
- `robot_state_publisher` lost `tf_prefix`; `frame_prefix` replaces it and wants
  the trailing slash. The stack's own nodes still take a `tf_prefix` parameter
  through `ros_compat::resolveFrame`, so both spellings appear in the launch.
- `dynamic_reconfigure` (9 sites) and `nodelet` (3 sites) have no ROS2 equivalent. In
  `aerial_robot_estimation` they happened to sit entirely inside code the first milestone
  does not need; in `aerial_robot_control` they are the gain-tuning paths and need
  replacing with parameter callbacks eventually.
- ROS2 CI alongside the existing ROS1 hovering tests in `.github/workflows/ros_test.yml`.
