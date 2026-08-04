"""Loading the stack's ROS1 yaml config from a ROS2 launch file.

ROS1 loaded a pile of yaml into a *namespace* on the global parameter server and
let every node in that namespace read what it wanted. ROS2 has no global server:
each node needs its own copy, and a parameter file has to name the node it is
for. The robots' config files are therefore merged here, at launch, rather than
being checked in a second time with a `ros__parameters` header.

That is a deliberate choice. The real robots fly on the ROS1 files, and two sets
of gains that can drift apart silently is the failure this migration cannot
afford. There is one copy of every parameter, and it is the one ROS1 reads.
"""

import os
import tempfile

import yaml


def deep_merge(base, override):
    """Merge `override` into `base` the way rosparam merged stacked yaml files.

    A plain dict.update() would be wrong: several of these files carry the same
    top-level key - `sensor_plugin:`, holding a different sensor in each - and
    the last file loaded would be the only one left.
    """
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def load_merged(paths, subtree=None):
    """Deep-merge a list of ROS1 yaml files, optionally taking one subtree.

    Paths that are empty are skipped, so a caller can pass a launch argument
    through without testing it first. `subtree` picks a single top-level key out
    of each file - `simulation`, say - for the cases where the block belongs to
    a different node than the rest of the file.
    """
    merged = {}
    for path in paths:
        if not path:
            continue
        with open(path, 'r') as stream:
            content = yaml.safe_load(stream) or {}
        if subtree is not None:
            content = content.get(subtree) or {}
        deep_merge(merged, content)
    return merged


def write_parameter_file(config_files, overrides=None, node='/**'):
    """Merge ROS1 config into one ROS2 parameter file and return its path.

    `overrides` is applied last: it is what the launch file itself decides, as
    the `<param>` tags inside a ROS1 `<node>` did. `node` is the key the
    parameters are filed under; the `/**` default matches whichever node the
    file is handed to, which is what these single-consumer files want.
    """
    merged = load_merged(config_files)
    if overrides:
        deep_merge(merged, overrides)

    handle, path = tempfile.mkstemp(prefix='aerial_robot_params_', suffix='.yaml')
    with os.fdopen(handle, 'w') as stream:
        yaml.safe_dump({node: {'ros__parameters': merged}}, stream, default_flow_style=False)
    return path


def as_bool(context, name):
    """A launch argument as a real bool, rejecting anything ambiguous.

    launch's own conditions accept a wider set of spellings, but a launch file
    that branches on Python truthiness would treat the string "false" as true.
    """
    from launch.substitutions import LaunchConfiguration

    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError("launch argument '{}' is not a boolean: {}".format(name, value))
