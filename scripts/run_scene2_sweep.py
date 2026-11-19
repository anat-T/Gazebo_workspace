#!/usr/bin/env python3
"""Runs scene2 (ball_sort_demo2.launch.py) N times, randomizing red_ball/
green_ball/gray_basket positions each run, and records each run to its own
scene2_run{i}/ directory: the rosbag (db3 + yaml), a video restored from
the camera images, and the end-effector trajectory csv.

Identical procedure to run_scene1_sweep.py (see its docstring), just for
scene2's world (one plain gray basket instead of a red/green pair) - see
scene_sweep_lib.py for the shared implementation.

Usage (after sourcing /opt/ros/humble/setup.bash and install/setup.bash):
    python3 run_scene2_sweep.py [--num-runs 10] [--start-index 1]
                                 [--output-dir scene2_runs] [--gui] [--seed 42]
"""
import argparse
from pathlib import Path

import scene_sweep_lib as lib

WORLD_TEMPLATE_PATH = lib.REPO_ROOT / "ros2_ws/src/box_pick_place_demo/worlds/ball_sort_demo2.sdf"

# Exact pose lines in worlds/ball_sort_demo2.sdf - matched verbatim and
# substituted per run. If that file's default positions ever change, these
# must be updated to match (a mismatch here means the substitution below
# silently does nothing and every run uses the file's default layout).
POSE_TOKENS = {
    "red_ball": ("<pose>0.40 -0.08 0.775 0 0 0</pose>", "{x:.4f} {y:.4f} 0.775 0 0 0"),
    "green_ball": ("<pose>0.40 0.08 0.775 0 0 0</pose>", "{x:.4f} {y:.4f} 0.775 0 0 0"),
    "gray_basket": ("<pose>0.65 0 0 0 0 0</pose>", "{x:.4f} {y:.4f} 0 0 0 0"),
}

OBJECTS = [
    ("gray_basket", lib.BASKET_CLEARANCE_M),
    ("green_ball", lib.BALL_CLEARANCE_M),
    ("red_ball", lib.BALL_CLEARANCE_M),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-runs", type=int, default=10)
    parser.add_argument("--start-index", type=int, default=1,
                         help="First run number to use (e.g. 6 to continue a sweep after runs 1-5).")
    parser.add_argument("--output-dir", type=Path, default=lib.REPO_ROOT / "scene2_runs")
    parser.add_argument("--gui", action="store_true", help="Show the Gazebo GUI (default: headless).")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--mission-timeout", type=float, default=180.0,
                         help="Max seconds to wait for a single run's mission to finish.")
    args = parser.parse_args()

    lib.run_sweep(
        args, run_prefix="scene2_run", launch_file="ball_sort_demo2.launch.py",
        world_template_path=WORLD_TEMPLATE_PATH, pose_tokens=POSE_TOKENS, objects=OBJECTS,
        basket_object="gray_basket", log_tag="run_scene2_sweep",
    )


if __name__ == "__main__":
    main()
