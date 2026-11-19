#!/usr/bin/env python3
"""Runs scene1 (ball_sort_demo.launch.py) N times, randomizing red_ball/
green_ball/red_basket/green_basket positions each run, and records each run
to its own scene1_run{i}/ directory: the rosbag (db3 + yaml), a video
restored from the camera images, and the end-effector trajectory csv.

Each run: sample a layout -> template it into a per-run copy of
worlds/ball_sort_demo.sdf -> launch the scene (headless by default) with
that world file and the matching basket_x/basket_y -> record
/robot/camera/image + /tf + /tf_static via `ros2 bag record` straight into
scene1_run{i}/ -> wait for the mission's own SUCCESS/FAILED log line ->
stop the bag and the scene cleanly -> post-process the bag into
video.mp4 + ee_trajectory.csv via this project's existing bag_to_video.py /
bag_to_ee_trajectory.py (both read the bag file directly, no need for a
live ROS graph). Shared machinery (layout sampling, process lifecycle, the
fast-abort-on-process-death fix) lives in scene_sweep_lib.py, reused by
run_scene2_sweep.py.

Usage (after sourcing /opt/ros/humble/setup.bash and install/setup.bash):
    python3 run_scene1_sweep.py [--num-runs 10] [--start-index 1]
                                 [--output-dir scene1_runs] [--gui] [--seed 42]
"""
import argparse
from pathlib import Path

import scene_sweep_lib as lib

WORLD_TEMPLATE_PATH = lib.REPO_ROOT / "ros2_ws/src/box_pick_place_demo/worlds/ball_sort_demo.sdf"

# Exact pose lines in worlds/ball_sort_demo.sdf - matched verbatim and
# substituted per run. If that file's default positions ever change, these
# must be updated to match (a mismatch here means the substitution below
# silently does nothing and every run uses the file's default layout).
POSE_TOKENS = {
    "red_ball": ("<pose>0.40 -0.08 0.775 0 0 0</pose>", "{x:.4f} {y:.4f} 0.775 0 0 0"),
    "green_ball": ("<pose>0.40 0.08 0.775 0 0 0</pose>", "{x:.4f} {y:.4f} 0.775 0 0 0"),
    "red_basket": ("<pose>0.65 -0.08 0 0 0 0</pose>", "{x:.4f} {y:.4f} 0 0 0 0"),
    "green_basket": ("<pose>0.65 0.08 0 0 0 0</pose>", "{x:.4f} {y:.4f} 0 0 0 0"),
}

OBJECTS = [
    ("green_basket", lib.BASKET_CLEARANCE_M),
    ("red_basket", lib.BASKET_CLEARANCE_M),
    ("green_ball", lib.BALL_CLEARANCE_M),
    ("red_ball", lib.BALL_CLEARANCE_M),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-runs", type=int, default=10)
    parser.add_argument("--start-index", type=int, default=1,
                         help="First run number to use (e.g. 6 to continue a sweep after runs 1-5).")
    parser.add_argument("--output-dir", type=Path, default=lib.REPO_ROOT / "scene1_runs")
    parser.add_argument("--gui", action="store_true", help="Show the Gazebo GUI (default: headless).")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--mission-timeout", type=float, default=180.0,
                         help="Max seconds to wait for a single run's mission to finish.")
    args = parser.parse_args()

    lib.run_sweep(
        args, run_prefix="scene1_run", launch_file="ball_sort_demo.launch.py",
        world_template_path=WORLD_TEMPLATE_PATH, pose_tokens=POSE_TOKENS, objects=OBJECTS,
        basket_object="green_basket", log_tag="run_scene1_sweep",
    )


if __name__ == "__main__":
    main()
