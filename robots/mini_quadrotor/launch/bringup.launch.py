# ROS2 counterpart of bringup.launch.
#
#   ros2 launch mini_quadrotor bringup.launch.py rm:=false sim:=true mujoco:=true
#
# The arguments keep their ROS1 names and defaults so the same invocation means
# the same thing under both versions.
#
# The one structural change is where the parameters go. ROS1 loaded these yaml
# files into a *namespace* on the global parameter server and every node in that
# namespace could read them; ROS2 has no global server, so each node needs its
# own copy and a parameter file must name the node it is for. The yaml files are
# therefore merged - in the order roslaunch would have applied them, and with
# the same deep merge rosparam did for stacked files - and written out under a
# `/**: ros__parameters:` header for the one node that reads them.
#
# Merging at launch rather than checking in wrapped copies is deliberate. Two
# sets of gains that can drift apart silently is exactly the failure this
# migration cannot afford: the robots fly on the ROS1 files, so those stay the
# only copy.
#
# Not carried across, and why:
#   - Gazebo. There is no ROS2 gazebo bringup for this stack yet; `mujoco:=true`
#     is the only simulation backend, and asking for the other one says so.
#   - simple_demo.py, and the sensor drivers that sensors.launch.xml starts for
#     a real machine (spinal serial bridge, mocap, livox, fast_lio). All ROS1.
#     The sensor *config* those drivers are configured by is still loaded here,
#     as sensors.launch.xml did, because the estimator reads it either way.
#   - spawn_x/y/z/yaw. Those were gazebo spawn arguments; the MuJoCo model
#     carries its own initial pose.

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro
import yaml

# EGOMOTION_ESTIMATE = 0, EXPERIMENT_ESTIMATE = 1, GROUND_TRUTH = 2
DEFAULT_ESTIMATE_MODE = '0'
DEFAULT_SIM_ESTIMATE_MODE = '2'


def as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError("launch argument '{}' is not a boolean: {}".format(name, value))


def deep_merge(base, override):
    """Merge `override` into `base`, the way rosparam merged stacked yaml files.

    A plain dict.update() would be wrong: three of these files have a top-level
    `sensor_plugin:` mapping holding different sensors, and the last one loaded
    would be the only one left.
    """
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def write_parameter_file(config_files, overrides):
    """Merge the ROS1 yaml files and write one ROS2 parameter file."""
    merged = {}
    for path in config_files:
        with open(path, 'r') as stream:
            deep_merge(merged, yaml.safe_load(stream) or {})
    deep_merge(merged, overrides)

    handle, path = tempfile.mkstemp(prefix='mini_quadrotor_params_', suffix='.yaml')
    with os.fdopen(handle, 'w') as stream:
        yaml.safe_dump({'/**': {'ros__parameters': merged}}, stream,
                       default_flow_style=False)
    return path


def launch_setup(context, *args, **kwargs):
    real_machine = as_bool(context, 'real_machine')
    simulation = as_bool(context, 'simulation')
    mujoco = as_bool(context, 'mujoco')
    headless = as_bool(context, 'headless')
    direct_model = as_bool(context, 'direct_model')
    demo = as_bool(context, 'demo')

    robot_ns = 'quadrotor' + LaunchConfiguration('robot_id').perform(context)
    config_dir = LaunchConfiguration('config_dir').perform(context)
    mini_quadrotor_share = get_package_share_directory('mini_quadrotor')
    base_share = get_package_share_directory('aerial_robot_base')

    if simulation and not real_machine and not mujoco:
        raise RuntimeError(
            'Gazebo has no ROS2 bringup in this stack yet. Pass mujoco:=true to '
            'simulate, or use the ROS1 bringup.launch for Gazebo.')
    if demo:
        raise RuntimeError(
            'simple_demo.py is rospy-only and has no ROS2 build. Pass demo:=false '
            'and drive the robot over its topics.')

    # The order roslaunch applied them in: the robot's own config from the
    # <group>, then the sensor config from the sensors.launch.xml include, which
    # comes after it in the document and so wins where they overlap.
    config_files = [os.path.join(config_dir, name) for name in (
        'RobotModel.yaml',
        'MotorInfo.yaml',
        'Battery.yaml',
        'FlightControl.yaml',
        'StateEstimation.yaml',
    )]
    if simulation and not real_machine:
        config_files.append(os.path.join(config_dir, 'Simulation.yaml'))
    config_files.append(os.path.join(config_dir, 'NavigationConfig.yaml'))
    config_files += [os.path.join(base_share, 'config', 'sensors', name) for name in (
        os.path.join('imu', 'spinal.yaml'),
        'mocap.yaml',
        os.path.join('lio', 'livox_mid360.yaml'),
    )]

    if direct_model:
        robot_model = LaunchConfiguration('direct_model_name').perform(context)
    elif simulation:
        robot_model = os.path.join(mini_quadrotor_share, 'urdf', 'robot.mujoco.xacro')
    else:
        robot_model = os.path.join(mini_quadrotor_share, 'urdf', 'robot.urdf.xacro')

    estimate_mode = LaunchConfiguration(
        'sim_estimate_mode' if simulation else 'estimate_mode').perform(context)

    parameter_file = write_parameter_file(config_files, {
        'estimation': {'mode': int(estimate_mode)},
        'uav_model': 0,
        # Private parameters of the base node under ROS1. The compat layer's
        # private handle differs from the public one only for topic names, so
        # under ROS2 they are ordinary parameters of the same node.
        'tf_prefix': robot_ns,
        'param_verbose': False,
        'main_rate': 40.0,
        # aerial_robot_model reads the description off its own node; there is no
        # namespace-wide robot_description to share.
        'robot_description': xacro.process_file(
            robot_model, mappings={'robot_name': robot_ns}).toprettyxml(indent='  '),
        'use_sim_time': simulation,
    })

    actions = [
        Node(
            package='aerial_robot_base',
            executable='aerial_robot_base_node',
            name='aerial_robot_base_node',
            namespace=robot_ns,
            output='screen',
            parameters=[parameter_file],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('aerial_robot_model'),
                'launch', 'aerial_robot_model.launch.py')),
            launch_arguments={
                'robot_model': robot_model,
                'robot_ns': robot_ns,
                'headless': str(headless).lower(),
                # Both simulation and the real machine get their joint state
                # from the robot, so the GUI publisher stays off and
                # rotor_tf_publisher supplies the rotor frames.
                'need_joint_state': str(not (simulation or real_machine)).lower(),
                'rviz_init_pose': os.path.join(config_dir, 'RvizInit.yaml'),
                'use_sim_time': str(simulation).lower(),
            }.items(),
        ),
    ]

    if simulation and not real_machine:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('aerial_robot_simulation'),
                'launch', 'mujoco.launch.py')),
            launch_arguments={
                'robot_ns': robot_ns,
                'headless': str(headless).lower(),
                'mujoco_model': os.path.join(
                    mini_quadrotor_share, 'mujoco', 'robot.xml'),
                # The `simulation:` half of Simulation.yaml belongs to the
                # hardware component, not the base node, so it goes here.
                'simulation_config': os.path.join(config_dir, 'Simulation.yaml'),
            }.items(),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rm', default_value='true'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('real_machine', default_value=LaunchConfiguration('rm')),
        DeclareLaunchArgument('simulation', default_value=LaunchConfiguration('sim')),
        DeclareLaunchArgument('estimate_mode', default_value=DEFAULT_ESTIMATE_MODE),
        DeclareLaunchArgument('sim_estimate_mode', default_value=DEFAULT_SIM_ESTIMATE_MODE),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('direct_model', default_value='false'),
        DeclareLaunchArgument('direct_model_name', default_value=''),
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('config_dir', default_value=os.path.join(
            get_package_share_directory('mini_quadrotor'), 'config')),
        DeclareLaunchArgument('mujoco', default_value='false'),
        DeclareLaunchArgument('demo', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
