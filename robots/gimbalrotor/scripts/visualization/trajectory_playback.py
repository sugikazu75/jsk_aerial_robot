#!/usr/bin/env python3
"""Play a trajectory file back for RViz: one of plan_perching_motion.py, or its optimisation by
aerial_to_solver (the same format).

The file holds, every dt, the root pose, the joint angles, the thrust and the force of each active
contact. This publishes, at the frame times (scaled by ~speed):

  joint_states          the posture, gimbals included, and the joint torques of the frame
                        ("effort", if the file has them). The movable joints the file does not
                        have are sent at their "fixed_joints" angle, 0 if not given (the rotors),
                        with no torque
  tf                    ~world_frame -> root link; (static) the contact frames
  ~thrust               WrenchStamped per rotor, in its thrust frame
  ~thrust_value/<thrust frame>
                        Float32 per rotor: its thrust [N] (e.g. for the jsk_rviz_plugins gauges)
  ~contact              WrenchStamped per support contact ("support": carries what the statics or
                        the optimisation gives it), in its contact frame; zero while inactive, or
                        RViz would keep showing the last one
  ~task_contact         the same for the task contacts ("prescribed": the force the task asks for),
                        apart so that RViz can draw them in another colour
  ~path                 latched Marker: the root path, coloured by the set of active contacts
  ~phase                Marker: the active contacts and the total thrust

With ~loop the playback starts over after ~pause seconds at the end.
"""

import json
from xml.etree import ElementTree

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker

CONTACT_COLORS = [(0.1, 0.4, 0.9), (0.9, 0.2, 0.2), (0.5, 0.1, 0.7), (0.9, 0.6, 0.1), (0.1, 0.6, 0.3)]


class TrajectoryPlayback:
    def __init__(self):
        with open(rospy.get_param("~trajectory_file")) as f:
            data = json.load(f)
        self.frames = data["frames"]
        self.dt = float(data["dt"])
        self.joint_names = data["joint_names"]
        self.thrust_frames = data["thrust_frames"]
        self.contacts = data.get("contacts", {})
        self.world = rospy.get_param("~world_frame", "world")
        self.prefix = str(rospy.get_param("~tf_prefix", "")).strip("/")
        self.speed = float(rospy.get_param("~speed", 1.0))
        self.loop = bool(rospy.get_param("~loop", True))
        self.pause = float(rospy.get_param("~pause", 2.0))

        tree = ElementTree.fromstring(rospy.get_param("robot_description"))
        links = {link.get("name") for link in tree.findall("link")}
        children = {joint.find("child").get("link") for joint in tree.findall("joint")}
        self.root_frame = self._tf(sorted(links - children)[0])
        movable = {
            j.get("name") for j in tree.findall("joint") if j.get("type") in ("revolute", "continuous", "prismatic")
        }
        self.extra_joints = sorted(movable - set(self.joint_names))
        fixed = data.get("fixed_joints", {})  # the joints the file does not move, at their angles
        self.extra_positions = [float(fixed.get(name, 0.0)) for name in self.extra_joints]
        # the sets of active contacts in order of appearance, for the colours of the path
        self.states = []
        for frame in self.frames:
            state = tuple(sorted(frame["contacts"]))
            if state not in self.states:
                self.states.append(state)

        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.thrust_pub = rospy.Publisher("~thrust", WrenchStamped, queue_size=30)
        self.thrust_value_pubs = [
            rospy.Publisher("~thrust_value/%s" % name, Float32, queue_size=1) for name in self.thrust_frames
        ]
        self.contact_pubs = {
            mode: rospy.Publisher(topic, WrenchStamped, queue_size=30)
            for mode, topic in (("support", "~contact"), ("prescribed", "~task_contact"))
        }
        self.path_pub = rospy.Publisher("~path", Marker, queue_size=1, latch=True)
        self.phase_pub = rospy.Publisher("~phase", Marker, queue_size=1)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._send_contact_frames()
        self._send_path()
        rospy.loginfo(
            "%d frames, %.1f s, contact states %s",
            len(self.frames),
            self._duration(),
            [list(s) or ["none"] for s in self.states],
        )
        self.start = rospy.Time.now()
        rospy.Timer(rospy.Duration(1.0 / float(rospy.get_param("~rate", 30.0))), self._cycle)

    def _tf(self, frame):
        return "%s/%s" % (self.prefix, frame) if self.prefix else frame

    def _duration(self):
        return (len(self.frames) - 1) * self.dt

    def _color(self, frame):
        r, g, b = CONTACT_COLORS[self.states.index(tuple(sorted(frame["contacts"]))) % len(CONTACT_COLORS)]
        return ColorRGBA(r, g, b, 1.0)

    def _send_contact_frames(self):
        transforms = []
        for name, entry in self.contacts.items():
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = self._tf(entry["link"])
            transform.child_frame_id = self._tf(name)
            t, r = transform.transform.translation, transform.transform.rotation
            t.x, t.y, t.z = entry["xyz"]
            r.x, r.y, r.z, r.w = entry["quaternion"]
            transforms.append(transform)
        if transforms:
            self.static_broadcaster.sendTransform(transforms)

    def _send_path(self):
        marker = Marker()
        marker.header.frame_id = self.world
        marker.ns, marker.id, marker.type = "path", 0, Marker.LINE_STRIP
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        for frame in self.frames:
            x, y, z = frame["root"][:3]
            marker.points.append(Point(x, y, z))
            marker.colors.append(self._color(frame))
        self.path_pub.publish(marker)

    def _index_at(self, elapsed):
        if self.loop:
            elapsed %= self._duration() + self.pause
        return min(int(elapsed / self.dt + 0.5), len(self.frames) - 1)

    def _wrench(self, stamp, frame, wrench):
        """wrench: [fx, fy, fz] or [fx, fy, fz, tx, ty, tz] (a surface contact)."""
        message = WrenchStamped()
        message.header.stamp = stamp
        message.header.frame_id = self._tf(frame)
        wrench = list(wrench) + [0.0] * (6 - len(wrench))
        f, t = message.wrench.force, message.wrench.torque
        f.x, f.y, f.z, t.x, t.y, t.z = (float(v) for v in wrench)
        return message

    def _cycle(self, _event):
        stamp = rospy.Time.now()
        index = self._index_at((stamp - self.start).to_sec() * self.speed)
        frame = self.frames[index]

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world
        transform.child_frame_id = self.root_frame
        t, r = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = frame["root"][:3]
        r.x, r.y, r.z, r.w = frame["root"][3:7]
        self.broadcaster.sendTransform(transform)

        message = JointState()
        message.header.stamp = stamp
        message.name = list(self.joint_names) + self.extra_joints
        message.position = list(frame["joints"]) + self.extra_positions
        if "effort" in frame:
            message.effort = list(frame["effort"]) + [0.0] * len(self.extra_joints)
        self.joint_pub.publish(message)

        for name, value, publisher in zip(self.thrust_frames, frame["thrust"], self.thrust_value_pubs):
            self.thrust_pub.publish(self._wrench(stamp, name, [0.0, 0.0, value]))
            publisher.publish(Float32(float(value)))
        for name in self.contacts:
            publisher = self.contact_pubs[self.contacts[name].get("mode", "support")]
            publisher.publish(self._wrench(stamp, name, frame["contacts"].get(name, [0.0] * 6)))

        marker = Marker()
        marker.header.frame_id = self.world
        marker.header.stamp = stamp
        marker.ns, marker.id, marker.type = "phase", 0, Marker.TEXT_VIEW_FACING
        marker.pose.position.x, marker.pose.position.y = frame["root"][:2]
        marker.pose.position.z = frame["root"][2] + 0.45
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.06
        marker.color = ColorRGBA(0.1, 0.1, 0.1, 1.0)
        marker.text = "%s  %.1f s\nthrust %.1f N" % (
            " + ".join(sorted(frame["contacts"])) or "flight",
            index * self.dt,
            sum(frame["thrust"]),
        )
        self.phase_pub.publish(marker)


def main():
    rospy.init_node("trajectory_playback")
    TrajectoryPlayback()
    rospy.spin()


if __name__ == "__main__":
    main()
