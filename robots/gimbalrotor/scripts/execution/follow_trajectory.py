#!/usr/bin/env python3
"""Play a trajectory as CoG, attitude and arm-joint targets, without task decisions.

  rosrun gimbalrotor follow_trajectory.py __ns:=gimbalrotor [_end_frame:=267 ...]

Parameters are in config/quad_with_dual_arm/follow_trajectory.yaml (~config to choose
another file). ~end_frame plays through that frame (-1: all). For a contact approach
with convergence checks, contact switching and operator decisions, use perching_task.py.
Thrust and gimbal values in the file are not sent; the flight controller allocates them.
"""

import rospy

from gimbalrotor.trajectory_follower import setup, take_off


def main():
    rospy.init_node("follow_trajectory")
    p, targets, player = setup()
    last = targets.last_frame(p["end_frame"])
    end_time, speed = targets.time_of(last), float(p["speed"])
    rospy.loginfo(
        "%d frames of %.3f s, playing 0..%d (%.1f s at speed %.2f); servos %s",
        len(targets),
        targets.dt,
        last,
        end_time / speed,
        speed,
        targets.servo_names,
    )

    if not take_off(p, player):
        return
    if not player.move_to(0.0, float(p["approach_speed"]), float(p["joint_speed"])):
        return
    if not player.hold(0.0, float(p["settle_time"])):
        return
    if not player.play(0.0, end_time, speed):
        return
    player.send(targets.still(end_time))
    rospy.loginfo("done: holding frame %d", last)


if __name__ == "__main__":
    main()
