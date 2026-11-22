#!/usr/bin/env python3
"""Run the perching SMACH only through the deformation flight, then stop before hook approach.

  rosrun gimbalrotor check_pre_hook_motion.py __ns:=gimbalrotor _takeoff:=true

The stop time is the first contact time minus approach_window in perching_task.yaml. The last
target remains held by navigation after this node exits; land with the usual flight controls.
No contact is activated, and the hook approach is not played.
"""

import rospy
import smach_ros

from perching_task import Task, build


def main():
    rospy.init_node("check_pre_hook_motion")
    # The first flight test uses the freshly planned path; the optimised example in
    # follow_trajectory.yaml may still describe an older bar position.
    if not rospy.has_param("~trajectory_file"):
        rospy.set_param(
            "~trajectory_file", "package://gimbalrotor/config/quad_with_dual_arm/trajectories/perching_motion.json"
        )
    task = Task()
    machine = build(task, stop_before_hook=True)
    viewer = smach_ros.IntrospectionServer("check_pre_hook_motion", machine, "/CHECK_PRE_HOOK_MOTION")
    viewer.start()
    try:
        outcome = machine.execute()
        rospy.loginfo("pre-hook motion check: %s", outcome)
    finally:
        viewer.stop()


if __name__ == "__main__":
    main()
