#!/usr/bin/env python3
"""A fixed pose of the robot for figures (launch/display.launch): one frame of a planned trajectory.

Make the configuration from a frame of a trajectory (config/quad_with_dual_arm/trajectories/*.json):

  rosrun gimbalrotor display_pose.py --trajectory <json> --time <s> [-o <yaml>]

  root    [x, y, z, qx, qy, qz, qw] of the root link in the world (the task frame of the trajectory placed)
  thrust  [N] per rotor
  zeros   {joint: position}: the initial positions of joint_state_publisher(_gui), that keep the pose
          and can be moved from there
  contacts {name: {link, xyz, quaternion, mode, force}}: the contact frames of the trajectory and the
          force [N] on each in its frame (zero if inactive in the frame)

Publish it (the node of display.launch, the configuration loaded as its private parameters):

  /tf_static              ~world -> ~root_frame (default world -> root)
  four_axes/command       spinal/FourAxisCommand with base_thrust ~thrust, for
                          aerial_robot_control/thrust_visualizer.py
  /tf_static              <link> -> <name> per contact of ~contacts
  ~contact                WrenchStamped per support contact (the hook on the bar), in its contact frame
  ~task_contact           the same for the task contacts ("prescribed": the rear hand on the wall), apart
                          so that RViz can draw them in another colour (as trajectory_playback.py)
"""

import argparse
import json
import sys

import yaml


def make_config(trajectory, time):
    with open(trajectory) as f:
        data = json.load(f)
    index = min(max(int(round(time / data["dt"])), 0), len(data["frames"]) - 1)
    frame = data["frames"][index]
    contacts = sorted(name for name, force in frame.get("contacts", {}).items() if any(force))
    config = {
        "root": [float(v) for v in frame["root"]],
        "thrust": [float(v) for v in frame["thrust"]],
        "zeros": {name: float(v) for name, v in zip(data["joint_names"], frame["joints"])},
        "contacts": {
            name: {
                "link": entry["link"],
                "xyz": [float(v) for v in entry["xyz"]],
                "quaternion": [float(v) for v in entry["quaternion"]],
                "mode": entry.get("mode", "support"),
                "force": [float(v) for v in frame.get("contacts", {}).get(name, [0.0, 0.0, 0.0])[:3]],
            }
            for name, entry in data.get("contacts", {}).items()
        },
    }
    header = "# frame %d (t = %.2f s) of %s; contacts: %s\n# made by: rosrun gimbalrotor display_pose.py --trajectory <json> --time <s>\n" % (
        index, index * data["dt"], trajectory.split("/")[-1], ", ".join(contacts) or "none")
    return header + yaml.safe_dump(config, default_flow_style=None, sort_keys=False)


def publish():
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped, WrenchStamped
    from spinal.msg import FourAxisCommand

    rospy.init_node("display_pose")
    root = rospy.get_param("~root", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    thrust = rospy.get_param("~thrust", [])
    contacts = rospy.get_param("~contacts", {})

    def transform(parent, child, xyz, quaternion):
        message = TransformStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = parent
        message.child_frame_id = child
        t, r = message.transform.translation, message.transform.rotation
        t.x, t.y, t.z = xyz
        r.x, r.y, r.z, r.w = quaternion
        return message

    transforms = [transform(rospy.get_param("~world", "world"), rospy.get_param("~root_frame", "root"), root[:3], root[3:7])]
    transforms += [transform(c["link"], name, c["xyz"], c["quaternion"]) for name, c in contacts.items()]
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)

    thrust_pub = rospy.Publisher("four_axes/command", FourAxisCommand, queue_size=1) if thrust else None
    command = FourAxisCommand(base_thrust=[float(v) for v in thrust])
    contact_pubs = {
        mode: rospy.Publisher(topic, WrenchStamped, queue_size=10)
        for mode, topic in (("support", "~contact"), ("prescribed", "~task_contact"))
    }
    rate = rospy.Rate(rospy.get_param("~rate", 10.0))
    while not rospy.is_shutdown():
        if thrust_pub:
            thrust_pub.publish(command)
        for name, c in contacts.items():
            wrench = WrenchStamped()
            wrench.header.stamp = rospy.Time.now()
            wrench.header.frame_id = name
            f = wrench.wrench.force
            f.x, f.y, f.z = c["force"]
            contact_pubs[c.get("mode", "support")].publish(wrench)
        rate.sleep()


if __name__ == "__main__":
    import rospy
    argv = rospy.myargv(sys.argv)[1:]
    if argv:
        parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
        parser.add_argument("--trajectory", required=True)
        parser.add_argument("--time", type=float, required=True, help="[s] of the frame")
        parser.add_argument("-o", "--output", help="the configuration (default: stdout)")
        args = parser.parse_args(argv)
        text = make_config(args.trajectory, args.time)
        if args.output:
            with open(args.output, "w") as f:
                f.write(text)
        else:
            sys.stdout.write(text)
    else:
        publish()
