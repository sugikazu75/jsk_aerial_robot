#!/usr/bin/env python
"""Walk the flying humanoid with the trajectories planned by walk_planner.

Launch bringup.launch with walk:=True, then run this script, which opens an IPython shell with `robot`:

  python3 $(rospack find flying_humanoid)/scripts/walk_interface.py
  >>> robot.start()
  >>> robot.takeoff()     # the rotors are on while walking
  >>> robot.plan(4)       # plan a walk of 4 steps from the current state, and wait until it is loaded
  >>> robot.walk()        # replay the planned trajectory
  >>> robot.plan(2)       # the next walk: plan again just before walking
  >>> robot.walk()
  >>> robot.walk(2)       # the same as robot.plan(2) and robot.walk()
"""

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

        # wait for this plan, not for another one (e.g. the standing one requested by the controller on start)
        def loaded():
            path = self.loaded_trajectory
            return path is not None and path != previous and "_%dsteps_" % num_steps in path

        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not loaded():
            if rospy.is_shutdown() or rospy.Time.now() > deadline:
                rospy.logerr("the walk trajectory was not loaded in %.0f s", timeout)
                return None
            rospy.sleep(0.1)

        rospy.loginfo("loaded %s", self.loaded_trajectory)
        return self.loaded_trajectory

    def walk(self, num_steps=None):
        """Replay the loaded trajectory. The robot must have taken off.

        With num_steps, plan a walk of num_steps from the current state first, then replay it.
        Return False if the planned trajectory was not loaded.
        """
        if num_steps is not None and self.plan(num_steps) is None:
            return False
        self.walk_start_pub.publish(Bool(data=True))
        return True

    def stop(self):
        """Stop and rewind the trajectory."""
        self.walk_start_pub.publish(Bool(data=False))


if __name__ == "__main__":
    from IPython import embed

    rospy.init_node("walk_interface")
    robot = FlyingHumanoidInterface()
    embed()
