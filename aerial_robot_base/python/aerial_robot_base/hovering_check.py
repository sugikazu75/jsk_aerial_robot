#!/usr/bin/env python3
"""ROS2 counterpart of src/aerial_robot_base/hovering_check.py.

Arm, take off, fly to a waypoint, land - and exit non-zero if any of that
fails. The ROS1 version is a rostest wrapped in unittest; there is no rostest
here, so this is a plain node whose exit status is the result. Run it against a
robot that is already up:

    ros2 launch mini_quadrotor bringup.launch.py rm:=false sim:=true mujoco:=true
    ros2 run aerial_robot_base hovering_check.py --ros-args \\
        -r __ns:=/quadrotor -p use_sim_time:=true \\
        -p waypoint.waypoints:="[0.0, 0.0, 1.6]"

`waypoint.waypoints` is *flat*: ROS2 parameters have no nested-array type, so
the ROS1 list of lists becomes one list plus `waypoint.stride` - 3 for
positions, 4 for position and yaw. Guessing the stride from the length would be
ambiguous (twelve numbers are four triples or three quadruples), and guessing
wrong flies the robot somewhere else without complaining.
"""

import sys

import rclpy

from aerial_robot_base.robot_interface import RobotInterface
from aerial_robot_base.state_machine import Arm, Land, StateMachine, Start, Takeoff, WayPoint


class HoverMotion(object):
    def __init__(self, robot, waypoints):
        self.robot = robot

        waypoint = WayPoint(robot)
        if waypoints:
            waypoint.waypoints = waypoints

        # Nothing else publishes /task_start when this runs as a test, so the
        # trigger the ROS1 rostest file supplied is pressed here instead.
        start = Start(robot)
        start.task_start = True

        self.sm = StateMachine()
        self.sm.add('Start', start)
        self.sm.add('Arm', Arm(robot))
        self.sm.add('Takeoff', Takeoff(robot))
        self.sm.add('WayPoint', waypoint)
        self.sm.add('Land', Land(robot))

    def startMotion(self):
        return self.sm.execute(self.robot.logger) == 'succeeded'


def read_waypoints(node):
    if not node.has_parameter('waypoint.waypoints'):
        node.declare_parameter('waypoint.waypoints', [0.0, 0.0, 1.6])
    if not node.has_parameter('waypoint.stride'):
        node.declare_parameter('waypoint.stride', 3)

    flat = list(node.get_parameter('waypoint.waypoints').value or [])
    stride = node.get_parameter('waypoint.stride').value

    if stride not in (3, 4):
        raise ValueError('waypoint.stride must be 3 (position) or 4 (position and yaw)')
    if len(flat) % stride != 0:
        raise ValueError(
            'waypoint.waypoints has {} elements, which is not a multiple of the stride {}'.format(
                len(flat), stride))

    return [flat[i:i + stride] for i in range(0, len(flat), stride)]


def main(args=None):
    rclpy.init(args=args)

    robot = RobotInterface(node_name='hovering_check')
    node = robot.node

    try:
        waypoints = read_waypoints(node)
    except ValueError as error:
        node.get_logger().error(str(error))
        robot.shutdown()
        rclpy.try_shutdown()
        return 1

    node.get_logger().info('start check hovering, waypoints: {}'.format(waypoints))

    ok = HoverMotion(robot, waypoints).startMotion()

    node.get_logger().info('hovering check: {}'.format('succeeded' if ok else 'FAILED'))
    robot.shutdown()
    rclpy.try_shutdown()

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
