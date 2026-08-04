# ROS2 counterpart of aerial_robot_model.launch.
#
# Differences from the ROS1 file, all forced by ROS2 rather than chosen:
#
#   - robot_description is a parameter of each node that needs it, not a shared
#     one on a global server. It is therefore set on robot_state_publisher and
#     on rotor_tf_publisher separately, and bringup sets its own copy on the
#     base node. The xacro is processed here rather than passed as a Command
#     substitution so a failure is reported once, with a traceback.
#   - tf_prefix is gone from robot_state_publisher; `frame_prefix` replaces it
#     and wants the trailing slash that tf::resolve used to add. The stack's own
#     nodes keep taking a tf_prefix parameter (ros_compat::resolveFrame), so the
#     two spellings sit side by side below.
#   - rviz is not started with the ROS1 `rviz_config` file: those configs name
#     rviz/* display classes that rviz2 does not have. Pass a converted config
#     explicitly if you want one.

import os

import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml


def parse_model_options(model_options):
    """Turn the ROS1 `model_options` string of `key:=value` pairs into a dict."""
    mappings = {}
    for option in model_options.split():
        if ':=' not in option:
            raise RuntimeError(
                "model_options entry '{}' is not of the form key:=value".format(option))
        key, value = option.split(':=', 1)
        mappings[key] = value
    return mappings


def process_robot_model(robot_model, mappings):
    doc = xacro.process_file(robot_model, mappings=mappings)
    return doc.toprettyxml(indent='  ')


def load_zeros(rviz_init_pose):
    """The `zeros` mapping the ROS1 launch loaded as a bare rosparam file.

    joint_state_publisher takes them as `zeros.<joint>`; the file itself has no
    `ros__parameters` header, so it cannot be handed over as a parameter file.
    """
    if not rviz_init_pose:
        return {}
    with open(rviz_init_pose, 'r') as stream:
        content = yaml.safe_load(stream) or {}
    return {'zeros.{}'.format(joint): float(value)
            for joint, value in (content.get('zeros') or {}).items()}


def launch_setup(context, *args, **kwargs):
    robot_ns = LaunchConfiguration('robot_ns').perform(context)
    robot_model = LaunchConfiguration('robot_model').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    rviz_init_pose = LaunchConfiguration('rviz_init_pose').perform(context)
    model_options = parse_model_options(
        LaunchConfiguration('model_options').perform(context))
    # Spelled out rather than left as a bare LaunchConfiguration: an untyped
    # substitution reaches the node as the string "true", which is not a bool
    # and leaves the node on wall time in a simulation that publishes /clock.
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    need_joint_state = LaunchConfiguration('need_joint_state')

    # The ROS1 launch names the robot only for the node that publishes the
    # description; the joint_state_publisher_gui copy is processed without it.
    description = process_robot_model(robot_model, dict(model_options, robot_name=robot_ns))
    bare_description = process_robot_model(robot_model, model_options)

    common = {'use_sim_time': use_sim_time}

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_ns,
            output='screen',
            parameters=[dict(common,
                             robot_description=description,
                             frame_prefix='{}/'.format(robot_ns) if robot_ns else '')],
        ),
        # Neither simulation nor real-machine bringup starts this - both pass
        # need_joint_state:=false - so it is carried across but unexercised.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=robot_ns,
            condition=IfCondition(need_joint_state),
            parameters=[dict(common, robot_description=bare_description,
                             **load_zeros(rviz_init_pose))],
        ),
        Node(
            package='aerial_robot_model',
            executable='rotor_tf_publisher',
            name='rotor_tf_publisher',
            namespace=robot_ns,
            condition=UnlessCondition(need_joint_state),
            output='screen',
            parameters=[dict(common, robot_description=description, tf_prefix=robot_ns)],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_ns,
            respawn=True,
            condition=UnlessCondition(LaunchConfiguration('headless')),
            arguments=(['-d', rviz_config] if rviz_config else []),
            parameters=[common],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model'),
        DeclareLaunchArgument('robot_ns'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('need_joint_state', default_value='true'),
        DeclareLaunchArgument('model_options', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('rviz_init_pose', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
