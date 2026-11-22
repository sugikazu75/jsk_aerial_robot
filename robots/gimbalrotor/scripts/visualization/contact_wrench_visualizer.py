#!/usr/bin/env python3
"""The contact forces of the whole-body controller, for RViz (config/rviz_config, started by
bringup.launch). The thrust of the rotors: aerial_robot_control/scripts/thrust_visualizer.py.

  rosrun gimbalrotor contact_wrench_visualizer.py __ns:=gimbalrotor

  whole_body/contacts/<name>/force
                          WrenchStamped of the controller: the force of the QP in the contact frame
                          (frame_id the bare contact name, not a TF frame)
  ~contact                the same, in the TF frame <tf_prefix>/<name>
  /tf_static              <tf_prefix>/<link> -> <tf_prefix>/<name> per contact, from the parameters of
                          the controller (controller/whole_body/contacts: link, xyz, quaternion)

  ~tf_prefix              of the contact frames (default: the namespace)
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, WrenchStamped


class ContactWrenchVisualizer:
    def __init__(self):
        prefix = rospy.get_param("~tf_prefix", rospy.get_namespace().strip("/"))
        tf_name = lambda name: prefix + "/" + name if prefix else name  # noqa: E731
        contacts = rospy.get_param("controller/whole_body/contacts", {})
        self.contact_pub = rospy.Publisher("~contact", WrenchStamped, queue_size=2 * max(len(contacts), 1))
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        transforms = []
        self.contact_subs = []
        for name, contact in sorted(contacts.items()):
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = tf_name(contact["link"])
            transform.child_frame_id = tf_name(name)
            t, r = transform.transform.translation, transform.transform.rotation
            t.x, t.y, t.z = (float(v) for v in contact.get("xyz", [0.0, 0.0, 0.0]))
            r.x, r.y, r.z, r.w = (float(v) for v in contact.get("quaternion", [0.0, 0.0, 0.0, 1.0]))
            transforms.append(transform)
            self.contact_subs.append(
                rospy.Subscriber(
                    "whole_body/contacts/%s/force" % name,
                    WrenchStamped,
                    self.contact_callback,
                    tf_name(name),
                    queue_size=1,
                )
            )
        if transforms:
            self.static_tf.sendTransform(transforms)
            rospy.loginfo("contacts %s", sorted(contacts))
        else:
            rospy.loginfo("no contact in %s", rospy.resolve_name("controller/whole_body/contacts"))

    def contact_callback(self, msg, frame):
        msg.header.frame_id = frame
        self.contact_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("contact_wrench_visualizer")
    ContactWrenchVisualizer()
    rospy.spin()
