# ROS2 counterpart of bringup.launch.
#
#   ros2 launch gimbalrotor bringup.launch.py airframe:=quad
#
# WHAT THIS DOES AND DOES NOT DO. gimbalrotor cannot fly under ROS2 yet, and
# this launch does not pretend otherwise. It brings up the base node with the
# robot's model, controller and navigator plugins and its config, and the
# description - which is enough to check that the whole ROS2 conversion of this
# package loads and initialises. It does not bring up a simulator or a real
# machine, because neither exists here yet:
#
#   - Simulation. gimbalrotor only ever simulated in Gazebo, and it ships no
#     MuJoCo model. Porting it needs a mujoco_model.yaml *and* gimbal servo
#     support in aerial_robot_simulation's hardware component, which today
#     handles rotors only.
#   - Real machine. The spinal serial bridge and mocap that sensors.launch.xml
#     starts are ROS1. Their *config* is still loaded here, as that file did,
#     because the estimator reads it either way.
#   - servo_bridge, which the ROS1 launch always starts and the gimbals need. It
#     is left out of aerial_robot_model's ROS2 build.
#
# See docs/ros2_migration.md.

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


def launch_setup(context, *args, **kwargs):
    real_machine = as_bool(context, 'real_machine')
    simulation = as_bool(context, 'simulation')
    headless = as_bool(context, 'headless')
    direct_model = as_bool(context, 'direct_model')

    airframe = LaunchConfiguration('airframe').perform(context)
    robot_ns = 'gimbalrotor' + LaunchConfiguration('robot_id').perform(context)
    config_dir = LaunchConfiguration('config_dir').perform(context)
    gimbalrotor_share = get_package_share_directory('gimbalrotor')
    base_share = get_package_share_directory('aerial_robot_base')

    if simulation and not real_machine:
        raise RuntimeError(
            'gimbalrotor has no ROS2 simulation. It simulates in Gazebo, which is '
            'not ported, and it ships no MuJoCo model. Use the ROS1 bringup.launch '
            'to simulate this robot.')

    # The order roslaunch applied them in, ending with the sensor config that
    # the sensors.launch.xml include contributed after the <group>.
    config_files = [
        os.path.join(config_dir, 'RobotModel.yaml'),
        os.path.join(config_dir, 'MotorInfo.yaml'),
        os.path.join(config_dir, airframe, 'Servo.yaml'),
        os.path.join(config_dir, 'Battery.yaml'),
        os.path.join(config_dir, airframe,
                     'GimbalrotorControl_sim.yaml' if simulation else 'GimbalrotorControl.yaml'),
        os.path.join(config_dir, 'StateEstimation.yaml'),
    ]
    if simulation and not real_machine:
        config_files.append(os.path.join(config_dir, 'Simulation.yaml'))
    config_files += [
        os.path.join(config_dir, 'NavigationConfig.yaml'),
        os.path.join(config_dir, 'sensors', 'imu', 'spinal.yaml'),
        os.path.join(base_share, 'config', 'sensors', 'mocap.yaml'),
    ]

    if direct_model:
        robot_model = LaunchConfiguration('direct_model_name').perform(context)
    else:
        robot_model = os.path.join(gimbalrotor_share, 'robots', airframe, 'gimbalrotor.urdf.xacro')

    estimate_mode = LaunchConfiguration(
        'sim_estimate_mode' if simulation else 'estimate_mode').perform(context)

    parameter_file = write_parameter_file(config_files, overrides={
        'estimation': {'mode': int(estimate_mode)},
        # Set by the launch under ROS1 too: this robot always uses its own
        # navigator rather than the default BaseNavigator.
        'flight_navigation_plugin_name': 'aerial_robot_navigation/gimbalrotor_navigation',
        # Private parameters of the base node under ROS1; ordinary parameters of
        # the same node here.
        'tf_prefix': robot_ns,
        'param_verbose': False,
        'main_rate': 40.0,
        'robot_description': xacro.process_file(
            robot_model, mappings={'robot_name': robot_ns}).toprettyxml(indent='  '),
        'use_sim_time': simulation,
    })

    return [
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
        DeclareLaunchArgument('airframe', default_value='quad'),
        DeclareLaunchArgument('config_dir', default_value=os.path.join(
            get_package_share_directory('gimbalrotor'), 'config')),
        OpaqueFunction(function=launch_setup),
    ])
