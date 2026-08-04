# ROS2 counterpart of bringup.launch.
#
#   ros2 launch dragon bringup.launch.py rm:=false sim:=true mujoco:=true
#
# The arguments keep their ROS1 names and defaults so the same invocation means
# the same thing under both versions. Parameters are merged from the ROS1 yaml
# at launch; aerial_robot_ros_compat.launch_config explains why.
#
# Not carried across, and why:
#   - Gazebo, and gazebo_quick_init_pose.py with it. `mujoco:=true` is the only
#     simulation backend under ROS2.
#   - simple_demo.py and the real-machine sensor drivers (spinal serial bridge,
#     mocap, livox, fast_lio). All ROS1. The sensor *config* is still loaded, as
#     sensors.launch.xml did, because the estimator reads it either way.
#   - servo_bridge. The hardware component holds the joints and gimbals at the
#     angles Servo.yaml asks for; a real-machine servo_bridge is still to do.

import os

from aerial_robot_ros_compat.launch_config import as_bool, write_parameter_file
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

# EGOMOTION_ESTIMATE = 0, EXPERIMENT_ESTIMATE = 1, GROUND_TRUTH = 2
DEFAULT_ESTIMATE_MODE = '1'
DEFAULT_SIM_ESTIMATE_MODE = '2'

# DRAGON = 32 (0x20): dragon type, with the full/over actuated property
UAV_MODEL_DRAGON = 32


def launch_setup(context, *args, **kwargs):
    real_machine = as_bool(context, 'real_machine')
    simulation = as_bool(context, 'simulation')
    mujoco = as_bool(context, 'mujoco')
    headless = as_bool(context, 'headless')
    direct_model = as_bool(context, 'direct_model')
    demo = as_bool(context, 'demo')
    new_model = as_bool(context, 'new_model')
    battery = as_bool(context, 'battery')
    full_vectoring_mode = as_bool(context, 'full_vectoring_mode')

    model_name = 'v1_5_202601' if new_model else 'vim4_202311'
    suffix = '' if battery else '_no_bat'
    link_num = LaunchConfiguration('link_num').perform(context)
    robot_ns = 'dragon' + LaunchConfiguration('robot_id').perform(context)
    takeoff_height = float(LaunchConfiguration('takeoff_height').perform(context))
    share = get_package_share_directory('dragon')
    base_share = get_package_share_directory('aerial_robot_base')
    config_dir = os.path.join(share, 'config', link_num, model_name)

    if simulation and not real_machine and not mujoco:
        raise RuntimeError(
            'Gazebo has no ROS2 bringup in this stack yet. Pass mujoco:=true to '
            'simulate, or use the ROS1 bringup.launch for Gazebo.')
    if demo:
        raise RuntimeError(
            'simple_demo.py is rospy-only and has no ROS2 build. Pass demo:=false '
            'and drive the robot over its topics.')

    config_files = [
        os.path.join(config_dir, 'model',
                     'FullVectoringRobotModel.yaml' if full_vectoring_mode
                     else 'HydrusLikeRobotModel.yaml'),
        os.path.join(config_dir, 'MotorInfo.yaml'),
        os.path.join(config_dir, 'Servo.yaml'),
        os.path.join(config_dir, 'Battery.yaml'),
        os.path.join(config_dir, 'StateEstimation.yaml'),
    ]
    if simulation and not real_machine:
        config_files.append(os.path.join(config_dir, 'Simulation.yaml'))
    config_files += [
        os.path.join(config_dir, 'control',
                     'FullVectoringControlConfig.yaml' if full_vectoring_mode
                     else 'LQIGimbalControlConfig.yaml'),
        os.path.join(config_dir, 'NavigationConfig.yaml'),
    ]
    # What the sensors.launch.xml include contributed, after the <group>.
    config_files += [
        os.path.join(share, 'config', 'sensors', 'imu', 'spinal.yaml'),
        os.path.join(base_share, 'config', 'sensors', 'mocap.yaml'),
        os.path.join(base_share, 'config', 'sensors', 'lio', 'livox_mid360.yaml'),
    ]

    overrides = {
        'estimation': {'mode': int(LaunchConfiguration(
            'sim_estimate_mode' if simulation else 'estimate_mode').perform(context))},
        'uav_model': UAV_MODEL_DRAGON,
        'flight_navigation_plugin_name': 'aerial_robot_navigation/dragon_navigation',
        'tf_prefix': robot_ns,
        'param_verbose': False,
        'main_rate': 40.0,
        'use_sim_time': simulation,
    }
    if simulation and not real_machine:
        # The ROS1 launch turns this off in simulation.
        overrides.setdefault('controller', {})['rotor_interfere_compensate'] = False
    if takeoff_height > 0:
        overrides.setdefault('navigation', {})['takeoff_height'] = takeoff_height

    description_mode = 'mujoco' if simulation else 'urdf'
    if direct_model:
        robot_model = LaunchConfiguration('direct_model_name').perform(context)
    else:
        robot_model = os.path.join(
            share, 'robots', link_num,
            '{}{}.{}.xacro'.format(model_name, suffix, description_mode))
    overrides['robot_description'] = xacro.process_file(
        robot_model, mappings={'robot_name': robot_ns}).toprettyxml(indent='  ')

    parameter_file = write_parameter_file(config_files, overrides=overrides)

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
                    share, 'mujoco', link_num, model_name + suffix, 'robot.xml'),
                'simulation_config': os.path.join(config_dir, 'Simulation.yaml'),
                'servo_config': os.path.join(config_dir, 'Servo.yaml'),
            }.items(),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rm', default_value='true'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('real_machine', default_value=LaunchConfiguration('rm')),
        DeclareLaunchArgument('simulation', default_value=LaunchConfiguration('sim')),
        DeclareLaunchArgument('full_vectoring_mode', default_value='true'),
        DeclareLaunchArgument('new_model', default_value='true'),
        DeclareLaunchArgument('link_num', default_value='quad'),
        DeclareLaunchArgument('battery', default_value='true'),
        DeclareLaunchArgument('estimate_mode', default_value=DEFAULT_ESTIMATE_MODE),
        DeclareLaunchArgument('sim_estimate_mode', default_value=DEFAULT_SIM_ESTIMATE_MODE),
        DeclareLaunchArgument('takeoff_height', default_value='0'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('direct_model', default_value='false'),
        DeclareLaunchArgument('direct_model_name', default_value=''),
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('mujoco', default_value='false'),
        DeclareLaunchArgument('demo', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
