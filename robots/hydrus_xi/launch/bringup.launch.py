# ROS2 counterpart of bringup.launch.
#
#   ros2 launch hydrus_xi bringup.launch.py \
#       real_machine:=false simulation:=true mujoco:=true headless:=true
#
# The arguments keep their ROS1 names and defaults so the same invocation means
# the same thing under both versions. Parameters are merged from the ROS1 yaml
# at launch; aerial_robot_ros_compat.launch_config explains why.
#
# Not carried across, and why:
#   - Gazebo. `mujoco:=true` is the only simulation backend under ROS2.
#   - simple_demo.py and the real-machine sensor drivers (spinal serial bridge,
#     zed, leddar_one, mocap). All ROS1. The sensor *config* is still loaded,
#     as sensors.launch.xml did, because the estimator reads it either way.
#   - init_gimbal_angles.py, which published each gimbal's
#     `simulation/init_value` onto gimbals_ctrl once the robot was up. The
#     hardware component now starts the gimbals at those angles itself, from the
#     same Servo.yaml, so there is nothing left for it to do.

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
DEFAULT_ESTIMATE_MODE = '0'
DEFAULT_SIM_ESTIMATE_MODE = '2'


def launch_setup(context, *args, **kwargs):
    real_machine = as_bool(context, 'real_machine')
    simulation = as_bool(context, 'simulation')
    mujoco = as_bool(context, 'mujoco')
    headless = as_bool(context, 'headless')
    direct_model = as_bool(context, 'direct_model')
    demo = as_bool(context, 'demo')

    robot_type = LaunchConfiguration('type').perform(context)
    onboards_model = LaunchConfiguration('onboards_model').perform(context)
    robot_ns = 'hydrus_xi' + LaunchConfiguration('robot_id').perform(context)
    config_dir = LaunchConfiguration('config_dir').perform(context)
    share = get_package_share_directory('hydrus_xi')
    base_share = get_package_share_directory('aerial_robot_base')

    if simulation and not real_machine and not mujoco:
        raise RuntimeError(
            'Gazebo has no ROS2 bringup in this stack yet. Pass mujoco:=true to '
            'simulate, or use the ROS1 bringup.launch for Gazebo.')
    if demo:
        raise RuntimeError(
            'simple_demo.py is rospy-only and has no ROS2 build. Pass demo:=false '
            'and drive the robot over its topics.')

    type_dir = os.path.join(config_dir, robot_type)
    config_files = [
        os.path.join(type_dir, 'RobotModel.yaml'),
        os.path.join(config_dir, 'motor_info', 'MN4010KV475_tmotor40A_15inch.yaml'),
        os.path.join(type_dir, 'Battery.yaml'),
        os.path.join(type_dir, 'Servo.yaml'),
        LaunchConfiguration('sensor_config_file').perform(context),
    ]
    if simulation:
        config_files.append(os.path.join(type_dir, 'Simulation.yaml'))
    config_files += [
        os.path.join(type_dir, 'FlightControl.yaml'),
        os.path.join(type_dir, 'NavigationConfig.yaml'),
    ]
    # What the sensors.launch.xml include contributed, after the <group>.
    config_files += [
        os.path.join(share, 'config', 'sensors', 'imu', 'spinal.yaml'),
        os.path.join(share, 'config', 'sensors', 'vo', 'zed_mini.yaml'),
        os.path.join(base_share, 'config', 'sensors', 'altimeter', 'leddar_one.yaml'),
        os.path.join(base_share, 'config', 'sensors', 'mocap.yaml'),
    ]

    # 'mujoco' rather than the ROS1 'gazebo': that description carries a
    # <gazebo> plugin tag naming gazebo_ros_control, and ros2_control reads a
    # <ros2_control> block instead.
    description_mode = 'mujoco' if simulation else 'urdf'
    if direct_model:
        robot_model = LaunchConfiguration('direct_model_name').perform(context)
    else:
        robot_model = os.path.join(share, 'robots', robot_type,
                                   '{}.{}.xacro'.format(onboards_model, description_mode))

    estimate_mode = LaunchConfiguration(
        'sim_estimate_mode' if simulation else 'estimate_mode').perform(context)

    parameter_file = write_parameter_file(config_files, overrides={
        'estimation': {'mode': int(estimate_mode)},
        'tf_prefix': robot_ns,
        'param_verbose': False,
        'main_rate': 40.0,
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
                'need_joint_state': str(not (simulation or real_machine)).lower(),
                'rviz_init_pose': os.path.join(type_dir, 'RvizInit.yaml'),
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
                    share, 'mujoco', robot_type, onboards_model, 'robot.xml'),
                'simulation_config': os.path.join(type_dir, 'Simulation.yaml'),
                # The gimbals and joints; the hardware component holds them at
                # the angles Servo.yaml asks for.
                'servo_config': os.path.join(type_dir, 'Servo.yaml'),
            }.items(),
        ))

    return actions


def generate_launch_description():
    share = get_package_share_directory('hydrus_xi')
    return LaunchDescription([
        DeclareLaunchArgument('real_machine', default_value='true'),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('type', default_value='hex_branch'),
        DeclareLaunchArgument('onboards_model', default_value='xavier201811'),
        DeclareLaunchArgument('estimate_mode', default_value=DEFAULT_ESTIMATE_MODE),
        DeclareLaunchArgument('sim_estimate_mode', default_value=DEFAULT_SIM_ESTIMATE_MODE),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('direct_model', default_value='false'),
        DeclareLaunchArgument('direct_model_name', default_value=''),
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('mujoco', default_value='false'),
        DeclareLaunchArgument('demo', default_value='false'),
        DeclareLaunchArgument('config_dir', default_value=os.path.join(share, 'config')),
        DeclareLaunchArgument('sensor_config_file', default_value=[
            os.path.join(share, 'config'), '/', LaunchConfiguration('type'),
            '/egomotion_estimation/', LaunchConfiguration('onboards_model'), '.yaml']),
        OpaqueFunction(function=launch_setup),
    ])
