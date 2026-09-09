#!/usr/bin/env python3
"""Extract a sensor_msgs/Image topic from a rosbag2 bag into an MP4 - no
need to replay the bag through ROS/Gazebo, reads the bag file directly via
rosbag2_py. Uses cv_bridge/cv2 the same way camera_sensor.py already does
elsewhere in this project.

Usage:
    python3 bag_to_video.py <bag_path> [--topic /robot/camera/image]
                             [--output out.mp4] [--fps 10]
"""
import argparse
import os

import cv2
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="Path to the rosbag2 directory (contains metadata.yaml).")
    parser.add_argument("--topic", default="/robot/camera/image", help="Image topic to extract.")
    parser.add_argument("--output", default=None, help="Output .mp4 path (default: <bag_name>.mp4 next to the bag).")
    parser.add_argument("--fps", type=float, default=10.0,
                         help="Output video frame rate (default 10, matching this project's camera update_rate).")
    args = parser.parse_args()

    bag_path = args.bag_path.rstrip("/")
    output_path = args.output or os.path.join(
        os.path.dirname(bag_path) or ".", os.path.basename(bag_path) + ".mp4"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))

    bridge = CvBridge()
    writer = None
    frame_count = 0

    while reader.has_next():
        topic, data, _timestamp = reader.read_next()
        msg = deserialize_message(data, Image)
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if writer is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(output_path, fourcc, args.fps, (width, height))
            print(f"bag_to_video: writing {width}x{height} @ {args.fps}fps -> {output_path}")

        writer.write(frame)
        frame_count += 1

    if writer is not None:
        writer.release()

    if frame_count == 0:
        print(f"bag_to_video: no messages found on '{args.topic}' in {bag_path} - nothing written")
    else:
        print(f"bag_to_video: done - {frame_count} frames written to {output_path}")


if __name__ == "__main__":
    main()
