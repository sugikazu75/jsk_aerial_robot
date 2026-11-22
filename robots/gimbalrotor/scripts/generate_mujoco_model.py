#!/usr/bin/env python3
"""Generate this package's MuJoCo model and apply its joint attributes locally."""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import rospkg
import yaml


def main(config_path):
    with open(config_path) as stream:
        config = yaml.safe_load(stream)
    subprocess.run(["rosrun", "mujoco_ros_utils", "mujoco_model_generator.py", config_path], check=True)
    packages = rospkg.RosPack()
    for package in config["package"]:
        model_config = config[package]
        filenames = model_config["filename"]
        attributes = model_config.get("joint_attributes", [{} for _ in filenames])
        if len(attributes) != len(filenames) or len(model_config["input"]) != len(filenames):
            raise ValueError("input, filename and joint_attributes must have the same length")
        for filename, joint_attributes in zip(filenames, attributes):
            path = os.path.join(packages.get_path(package), "mujoco", filename, "robot.xml")
            tree = ET.parse(path)
            found = set()
            for joint in tree.iter("joint"):
                name = joint.get("name")
                if name in joint_attributes:
                    for key, value in joint_attributes[name].items():
                        joint.set(key, str(value))
                    found.add(name)
            missing = set(joint_attributes) - found
            if missing:
                raise ValueError("%s: missing MuJoCo joints %s" % (path, sorted(missing)))
            tree.write(path, encoding="unicode", xml_declaration=True)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: generate_mujoco_model.py CONFIG.yaml")
    main(sys.argv[1])
