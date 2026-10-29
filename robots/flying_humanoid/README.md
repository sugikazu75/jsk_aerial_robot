# Flying Humanoid

A biped with two gimbal rotors on its body. It flies with the gimbalrotor controller, and walks by replaying a
trajectory planned by trajectory optimization ([crocoddyl](https://github.com/loco-3d/crocoddyl)), with the rotors
supporting a part of its weight.

## Setup

The walk needs the following in addition to the usual setup of jsk_aerial_robot (the flight does not).

| package | from | |
|---|---|---|
| crocoddyl | `aerial_robot_*.rosinstall` of the repository root | a fork with the thrust-rate models (`PR/thrust_rate/exact_gradient` of sugikazu75/crocoddyl) |
| pinocchio (v4) | apt: `ros-${ROS_DISTRO}-pinocchio` | |
| hpipm-cpp, hpipm, blasfeo | another workspace, e.g. `~/ros/ocp_ws` | for the HPIPM-SQP solver (see [Solvers](#solvers)) |

Extend the workspace with hpipm-cpp, and build in Release (without optimization, the planning becomes much slower).
`BUILD_WITH_MULTITHREADS` lets crocoddyl use `num_threads` of the planner.

```bash
cd ~/ros/flying_humanoid_ws
catkin config --extend ~/ros/ocp_ws/devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_MULTITHREADS=ON \
  -DBUILD_BENCHMARK=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
catkin build flying_humanoid
```

If crocoddyl is built without hpipm-cpp, set `solver_type: 0` and `touchdown_constraint: false` in
`config/WalkPlanner.yaml`.

## Fly

```bash
roslaunch flying_humanoid bringup.launch real_machine:=False simulation:=True headless:=False
rosrun aerial_robot_base keyboard_command.py   # in another terminal: r to arm, t to take off, l to land
```

## Walk

```bash
roslaunch flying_humanoid bringup.launch real_machine:=False simulation:=True headless:=False walk:=True
python3 $(rospack find flying_humanoid)/scripts/walk_interface.py   # in another terminal: an IPython shell with `robot`
```

```python
robot.start()
robot.takeoff()     # the rotors are on (in the takeoff state of the navigator) while walking
robot.plan(4)       # plan a walk of 4 steps from the current state, and wait until the controller loads it
robot.walk()        # replay it
robot.plan(2)       # the next walk: plan again just before walking
robot.walk()
robot.walk(2)       # the same as robot.plan(2) and robot.walk()
```

`FlyingHumanoidInterface` in `scripts/walk_interface.py` can be used in your own script as well.

### How it works

```
walk_interface.py ──walk/plan_request (Int32: steps)──> walk_planner
                                                          plans from uav/baselink/odom and joint_states,
                                                          saves /tmp/flying_humanoid_walk/walk_<N>steps_<time>.csv
walk controller   <──walk/trajectory (String: CSV)───────┘
(aerial_robot_base)  loads it unless walking, ──walk/trajectory_loaded──> walk_interface.py
                  <──walk/start (Bool)──────────── walk_interface.py
                     replays it: joints_ctrl (legs), gimbals_ctrl (gimbals),
                     four_axes/command (thrusts), desire_coordinate (attitude)
```

- `walk_planner` (`src/walk_planner*.cpp`, `config/WalkPlanner.yaml`) is a node started by `bringup.launch walk:=True`.
- The walk controller (`src/walk_controller.cpp`, `config/WalkControl.yaml`) is an `aerial_robot_control` plugin,
  loaded by `aerial_robot_base` instead of the gimbalrotor controller. Before `walk/start`, it holds the first
  sample of the trajectory. The thrusts are fed forward from the trajectory, plus a PD feedback on the attitude of the
  baselink. When the robot is armed (`start`) without any trajectory, the controller requests a standing one (0 steps):
  a single sample of the current state with the static thrusts by a QP, without optimization. So the thrusts are sent
  after the takeoff before planning any walk.

## Walk planner

The walk is planned relative to the state measured when it is requested: it goes along the heading (yaw) of the
robot, and each foot keeps its measured height (it may not be zero because of the offset of the motion capture).
Plan just before walking. `use_measured_state: false` plans from `initial_configuration` at the origin instead,
which is handy to try the planner alone.

The plan is: stand, move the weight onto the left foot, steps alternating from the right foot (each foot moves by
`step_length`), bring the other foot next to the last swing foot, and stand. The thrusts are a part of the state
and the control input is their rate (thrust-rate mode), so the planned thrusts change smoothly.

Main parameters in `config/WalkPlanner.yaml`:

| parameter | |
|---|---|
| `step_length`, `step_height` | [m] |
| `time_step`, `step_knots`, `support_knots` | [s], knots of a swing phase and of a double support phase |
| `lateral_shift_scale` | how much the CoM moves toward the support foot (0: stays at the center) |
| `solver_type` | 0: FDDP, 1: box-FDDP, 3: HPIPM-SQP |
| `touchdown_constraint` | the swing foot lands exactly on its target with zero velocity (equality constraint) |
| `touchdown_velocity_weight` | the same as a cost |
| `terminal_posture_constraint` | the joint angles return to those at the start (equality constraint on the terminal node) |
| `terminal_thrust_weight` | the thrusts return to the static ones at the start (cost on the terminal node, see below) |
| `*_weight` | cost weights |

Without `terminal_thrust_weight`, the thrusts end unbalanced (e.g. 2.2 N / 3.9 N against 4.8 N / 4.8 N at the start):
in the double support, the contact forces take any distribution of the thrusts. The thrusts cannot be constrained,
since the constraints of the crocoddyl fork see only q and v. They are in a cost model built on the state with the
thrusts, which however cannot hold the costs needing the multibody state (CoM, wrench cone, centroidal momentum), so
the terminal node has only the state costs.

### Solvers

Which equality constraints each solver can handle:

| `solver_type` | equality constraints on the terminal node | ... on the other nodes |
|---|---|---|
| 0: FDDP | yes | no |
| 1: box-FDDP | no | no |
| 3: HPIPM-SQP | yes | yes |

- FDDP and box-FDDP leave the equality constraints on the nodes other than the terminal one violated, but still
  count the violation in their stopping criterion. With `touchdown_constraint: true` they never converge.
- With a terminal equality constraint, box-FDDP stops at the first iteration and returns the initial guess.
- box-FDDP is the only one which keeps the joint torques in their limits; the planned torques have been well
  within the limits (about 0.7 Nm against 1.5 Nm) anyway.

So use HPIPM-SQP with the default constraints, or FDDP with `touchdown_constraint: false`.
A walk of 2 to 4 steps converges in about 20 to 30 iterations, in about 1 s.

### Diagnosis

With `diagnosis: true` (a private parameter of `walk_planner`), next to the trajectory CSV it saves

- `*_solver.csv`: the solver quantities of each iteration (cost, stopping criterion, regularization, step length,
  dynamics gap `ffeas`, equality constraint violation `hfeas`, expected and actual improvement, ...)
- `*_nodes.csv`: for each node, the weighted cost of each term, the violation of each equality constraint, the
  dynamics gap, and the size of the control

and prints the error of the analytic derivatives against finite differences at some nodes.
