#!/usr/bin/env python

import rosbag
import sys
import os


args = sys.argv

assert len(args)>=2, "you must specify the rosbag by assigning the argument."

filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))


bag = rosbag.Bag(filename)

for i, (topic, msg, t) in enumerate(bag.read_messages(topics=['/rosout', 'rosout'])):

    level = "INFO"
    color = '30m'
    if msg.level == 2:
        level = "INFO"
        color = '30m'
    if msg.level == 4:
        level = "WARN"
        color = '33m'

    if msg.level == 6 or msg.level == 8:
        level = "WARN"
        color = '31m'

    print("\033[{} [{}] [{}.{}]: {} \033[0m".format(color, level, msg.header.stamp.secs, msg.header.stamp.nsecs, msg.msg))

bag.close()
