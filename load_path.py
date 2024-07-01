#!/usr/bin/env python
# load_path.py
"""A simple script to load path from a csv.

It supports full trajectories and plain xy data.
"""
######################
# Imports & Globals
######################

import argparse
import csv

from autopsy.core import Core
from autopsy.node import Node


from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


PARSER = argparse.ArgumentParser(
    description = """
Load path from a csv file and publish it.
"""
)

PARSER.add_argument(
    "input_file",

    help = "file to load",
    type = argparse.FileType("r"),
)

PARSER.add_argument(
    "-frame_id", "--frame_id",

    help = "frame id of the Path, default 'map'",
    default = "map",
    type = str,
)


######################
# Main
######################

if __name__ == "__main__":
    args, other = PARSER.parse_known_args()

    Core.init(args = other)

    # Create node
    n = Node("load_path")

    pub = n.Publisher(
        "path", Path, queue_size = 1, latch = True
    )

    # Create data
    msg = Path()
    msg.header.frame_id = args.frame_id

    reader = csv.DictReader(args.input_file)
    n.loginfo("Reading from '%s'." % args.input_file.name)

    for row in reader:
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id

        pose.pose.position.x = float(row["x_m"])
        pose.pose.position.y = float(row["y_m"])

        msg.poses.append(pose)

    pub.publish(msg)

    n.loginfo("Published path with %d poses." % len(msg.poses))
