#!/usr/bin/env python3
"""Extracts the end-effector (gripper/tool0) trajectory from a rosbag2 bag's
/tf + /tf_static topics into a CSV - reads the bag directly via rosbag2_py
(same non-replay pattern as bag_to_video.py), feeds every recorded
transform into an offline tf2_ros.Buffer, then looks up base_frame ->
ee_frame once per /tf message.

Uses the bag's own recorded /tf chain (i.e. whatever robot_state_publisher
actually computed from the real joint motion), not ik_solver.py's
approximate model - correct for both sim and real-robot bags without
needing to know which one produced it, and unaffected by ik_solver.py's
real-gripper-specific TCP offset (which has no meaning for a bag recorded
in Gazebo, where there's no physical gripper attached at all).

Usage:
    python3 bag_to_ee_trajectory.py <bag_path> \
        [--base-frame robot_base_link] [--ee-frame robot_tool0] \
        [--output out.csv]

Defaults match this project's Gazebo sim (tf_prefix="robot_"). For a bag
recorded on the real robot, pass --base-frame base_link --ee-frame tool0
(no prefix there).
"""
import argparse
import csv
import os

import rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
import tf2_ros


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="Path to the rosbag2 directory (contains metadata.yaml).")
    parser.add_argument("--base-frame", default="robot_base_link",
                         help="Reference frame (default matches the sim's tf_prefix='robot_').")
    parser.add_argument("--ee-frame", default="robot_tool0",
                         help="End-effector frame to track (default: robot_tool0).")
    parser.add_argument("--output", default=None,
                         help="Output .csv path (default: <bag_name>_ee_trajectory.csv next to the bag).")
    args = parser.parse_args()

    bag_path = args.bag_path.rstrip("/")
    output_path = args.output or os.path.join(
        os.path.dirname(bag_path) or ".", os.path.basename(bag_path) + "_ee_trajectory.csv"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/tf_static"]))

    # Long cache time so early-recorded static transforms (published once,
    # at the very start) stay valid for lookups made much later in the bag
    # - this buffer is fed directly from bag messages, not a live listener.
    buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3600))

    rows = []
    first_stamp = None
    skipped = 0

    while reader.has_next():
        topic, data, _bag_timestamp = reader.read_next()
        msg = deserialize_message(data, TFMessage)
        if not msg.transforms:
            continue

        for transform in msg.transforms:
            if topic == "/tf_static":
                buffer.set_transform_static(transform, "bag")
            else:
                buffer.set_transform(transform, "bag")

        if topic == "/tf_static":
            continue  # static transforms aren't motion samples

        stamp = msg.transforms[0].header.stamp
        time = Time.from_msg(stamp)
        if first_stamp is None:
            first_stamp = time

        try:
            tf = buffer.lookup_transform(args.base_frame, args.ee_frame, time)
        except Exception:
            skipped += 1
            continue

        t = tf.transform.translation
        r = tf.transform.rotation
        t_sec = (time - first_stamp).nanoseconds / 1e9
        rows.append([t_sec, t.x, t.y, t.z, r.x, r.y, r.z, r.w])

    if not rows:
        print(f"bag_to_ee_trajectory: no valid '{args.base_frame}' -> '{args.ee_frame}' "
              f"lookups found in {bag_path} - check --base-frame/--ee-frame match the bag's "
              f"tf_prefix (skipped {skipped} attempts)")
        return

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_sec", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(rows)

    print(f"bag_to_ee_trajectory: done - {len(rows)} samples ({skipped} skipped) written to {output_path}")


if __name__ == "__main__":
    main()
