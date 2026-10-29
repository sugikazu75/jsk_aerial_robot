#!/usr/bin/env python
"""Walk the flying humanoid with the trajectories planned by walk_planner.

Launch bringup.launch with walk:=True, then

  rosrun flying_humanoid walk_interface.py 4      # plan a walk of 4 steps, take off and walk

or use FlyingHumanoidInterface from your own script:

  robot = FlyingHumanoidInterface()
  robot.plan(4)           # trajectory optimization (takes a while)
  robot.start(); robot.takeoff()
  robot.walk()            # replay the planned trajectory
"""

import argparse

import rospy
from aerial_robot_base.robot_interface import RobotInterface
from std_msgs.msg import Bool, Int32, String


class FlyingHumanoidInterface(RobotInterface):
    def __init__(self, robot_ns="flying_humanoid", init_node=False):
        super(FlyingHumanoidInterface, self).__init__(robot_ns=robot_ns, init_node=init_node)

        self.plan_request_pub = rospy.Publisher(self.robot_ns + "/walk/plan_request", Int32, queue_size=1)
        self.walk_start_pub = rospy.Publisher(self.robot_ns + "/walk/start", Bool, queue_size=1)
        self.loaded_trajectory = None
        rospy.Subscriber(self.robot_ns + "/walk/trajectory_loaded", String, self.trajectoryLoadedCallback)
        rospy.sleep(0.5)  # wait for the connections

    def trajectoryLoadedCallback(self, msg):
        self.loaded_trajectory = msg.data

    def plan(self, num_steps, timeout=600.0):
        """Plan a walk and wait until the walk controller loads it. Return the path of the trajectory CSV."""
        previous = self.loaded_trajectory
        self.plan_request_pub.publish(Int32(data=num_steps))
        rospy.loginfo("planning a walk of %d steps...", num_steps)

        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while self.loaded_trajectory == previous:
            if rospy.is_shutdown() or rospy.Time.now() > deadline:
                rospy.logerr("the walk trajectory was not loaded in %.0f s", timeout)
                return None
            rospy.sleep(0.1)

        rospy.loginfo("loaded %s", self.loaded_trajectory)
        return self.loaded_trajectory

    def walk(self):
        """Replay the loaded trajectory. The robot must have taken off."""
        self.walk_start_pub.publish(Bool(data=True))

    def stop(self):
        """Stop and rewind the trajectory."""
        self.walk_start_pub.publish(Bool(data=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("num_steps", type=int)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("walk_interface")
    robot = FlyingHumanoidInterface()

    if robot.plan(args.num_steps) is None:
        exit(1)

    if robot.getFlightState() != robot.TAKEOFF_STATE:
        robot.start()
        robot.takeoff()
        rospy.sleep(3.0)  # let the thrusts rise to the standing ones

    robot.walk()
    rospy.sleep(1.0)  # keep the node alive until the message is delivered
