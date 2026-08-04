# ROS2 counterpart of mujoco.launch.
#
# The ROS1 file put the MuJoCo plugin list on the parameter server inline, with
# `<rosparam subst_value="true">`, and spawned ros_control controllers next to
# it. Under ROS2 both halves change shape:
#
#   - mujoco_ros reads its plugin list from a *parameter file* passed to
#     mujoco_node, keyed by node name (/mujoco_server, and /mujoco_server/<plugin>
#     for each plugin's own node). Parameter files cannot carry substitutions, so
#     the robot namespace has to be baked in; this launch renders the file at
#     launch time, which is the same thing `subst_value="true"` did.
#
#   - No controller is spawned. In the spinal configuration the flight control
#     core lives inside the hardware component (see docs/ros2_migration.md), and
#     mujoco_ros_control activates the component itself, so read()/write() run
#     with no controller loaded. joint_state_controller has no ROS2 counterpart
#     to spawn either: the component exports thrust/effort interfaces only, and
#     joint_state_broadcaster wants position. Rotor frames come from
#     aerial_robot_model's rotor_tf_publisher, as they do in the ROS1 sim.
#
# mujoco_ros_sensors is not in the plugin list. The ROS1 launch loaded it, but
# the hardware component reads acc/gyro/mag straight out of mjData by sensor
# name, so nothing here consumes the topics it would publish - and the package
# has no ROS2 build in this workspace.

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import yaml

MUJOCO_SERVER_NODE = '/mujoco_server'


def deep_merge(base, override):
    """Merge `override` into `base` the way rosparam merged stacked yaml files."""
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def load_simulation_params(context):
    """The `simulation:` block the hardware component reads, from the ROS1 yaml.

    Under ROS1 these arrived on the parameter server from two stacked rosparam
    loads - this package's Mujoco.yaml and the robot's own Simulation.yaml - and
    the hardware sim read them through a `simulation` child NodeHandle. Same two
    files, same stacking order, merged here because a ROS2 parameter file has to
    be addressed to a node and these belong to mujoco_ros_control's.
    """
    params = {}
    paths = [os.path.join(get_package_share_directory('aerial_robot_simulation'), 'config', 'Mujoco.yaml'),
             LaunchConfiguration('simulation_config').perform(context)]
    for path in paths:
        if not path:
            continue
        with open(path, 'r') as stream:
            deep_merge(params, (yaml.safe_load(stream) or {}).get('simulation') or {})
    return params


def render_plugin_config(context):
    """Build the mujoco_ros parameter file for this robot and write it out."""
    robot_ns = LaunchConfiguration('robot_ns').perform(context).strip('/')
    control_period = float(LaunchConfiguration('control_period').perform(context))
    overrides_path = LaunchConfiguration('plugin_config_overrides').perform(context)

    control_node = '{}/mujoco_ros_control'.format(MUJOCO_SERVER_NODE)
    visualizer_node = '{}/thrust_visualizer'.format(MUJOCO_SERVER_NODE)
    controller_manager_node = '/{}/controller_manager'.format(robot_ns) if robot_ns \
        else '/controller_manager'

    config = {
        MUJOCO_SERVER_NODE: {
            'ros__parameters': {
                'MujocoPlugins.names': ['mujoco_ros_control', 'thrust_visualizer'],
                'MujocoPlugins.mujoco_ros_control.type':
                    'mujoco_ros_control/MujocoRosControlPlugin',
                'MujocoPlugins.thrust_visualizer.type':
                    'aerial_robot_simulation/ThrustVisualizerPlugin',
            },
        },
        control_node: {
            'ros__parameters': {
                # Read back by AerialRobotMujocoSystem::initSim to decide where
                # the simulated flight controller's node and its ground_truth /
                # mocap topics go, as well as by mujoco_ros_control itself to
                # find robot_state_publisher and place the controller manager.
                'namespace': robot_ns,
                'robot_description_node': 'robot_state_publisher',
                'robot_description': 'robot_description',
                # Mocap and ground-truth noise, read by the hardware component
                # through its `simulation` child handle.
                'simulation': load_simulation_params(context),
            },
        },
        visualizer_node: {
            'ros__parameters': {
                'min_force': 0.001,
                'rgba': [0.0, 0.8, 1.0, 0.9],
            },
        },
        controller_manager_node: {
            'ros__parameters': {
                # mujoco_ros_control refuses to load without this one.
                'update_rate': int(round(1.0 / control_period)),
                'use_sim_time': True,
            },
        },
    }

    if overrides_path:
        with open(overrides_path, 'r') as stream:
            deep_merge(config, yaml.safe_load(stream) or {})

    handle, path = tempfile.mkstemp(prefix='mujoco_plugins_', suffix='.yaml')
    with os.fdopen(handle, 'w') as stream:
        yaml.safe_dump(config, stream, default_flow_style=False)
    return path


def launch_setup(context, *args, **kwargs):
    mujoco_ros_share = get_package_share_directory('mujoco_ros')
    headless = LaunchConfiguration('headless').perform(context)

    return [
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(mujoco_ros_share, 'launch', 'launch_server.launch.xml')),
            launch_arguments={
                'use_sim_time': 'true',
                'unpause': 'true',
                'headless': headless,
                'no_render': LaunchConfiguration('no_render').perform(context),
                'modelfile': LaunchConfiguration('mujoco_model').perform(context),
                'mujoco_threads': LaunchConfiguration('mujoco_threads').perform(context),
                'mujoco_plugin_config': render_plugin_config(context),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value=''),
        DeclareLaunchArgument('mujoco_model'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('no_render', default_value=LaunchConfiguration('headless')),
        DeclareLaunchArgument('mujoco_threads', default_value='1'),
        # 1 kHz, as the ROS1 launch's hardware/control_period asked for.
        DeclareLaunchArgument('control_period', default_value='0.001'),
        # The robot's own Simulation.yaml, stacked on this package's Mujoco.yaml
        # exactly as the two rosparam loads did under ROS1.
        DeclareLaunchArgument('simulation_config', default_value=''),
        # A ROS2 parameter file merged over the generated one, for anything a
        # robot needs to say about the simulation that is not covered above.
        DeclareLaunchArgument('plugin_config_overrides', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
