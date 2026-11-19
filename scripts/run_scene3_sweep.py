#!/usr/bin/env python3
"""Runs scene3 (ball_sort_demo3.launch.py) N times, randomizing
ball_a/ball_b/basket_a/basket_b positions each run, and records each run to
its own scene3_run{i}/ directory: the rosbag (db3 + yaml), a video restored
from the camera images, and the end-effector trajectory csv.

Same procedure as run_scene1_sweep.py/run_scene2_sweep.py (see their
docstrings) - see scene_sweep_lib.py for the shared implementation - with
one scene3-specific twist: the gripper's DetachableJoint is hardwired to
ball_a at spawn time (see ball_sort_demo3.launch.py's own comment on this),
so ball_a MUST stay the "right" ball (the smaller y) or the mission would
resolve ball_b as the pick target from live poses while the gripper can
only ever physically attach to ball_a - a guaranteed failure that has
nothing to do with IK/reachability. After sampling, this script swaps
ball_a/ball_b's positions if needed to restore that ordering (harmless -
both balls are the same size/shape, so swapping preserves every clearance
constraint the sampler already satisfied).

Usage (after sourcing /opt/ros/humble/setup.bash and install/setup.bash):
    python3 run_scene3_sweep.py [--num-runs 10] [--start-index 1]
                                 [--output-dir scene3_runs] [--gui] [--seed 42]
"""
import argparse
from pathlib import Path

import scene_sweep_lib as lib

WORLD_TEMPLATE_PATH = lib.REPO_ROOT / "ros2_ws/src/box_pick_place_demo/worlds/ball_sort_demo3.sdf"

# Exact pose lines in worlds/ball_sort_demo3.sdf - matched verbatim and
# substituted per run. If that file's default positions ever change, these
# must be updated to match (a mismatch here means the substitution below
# silently does nothing and every run uses the file's default layout).
POSE_TOKENS = {
    "ball_a": ("<pose>0.40 -0.08 0.775 0 0 0</pose>", "{x:.4f} {y:.4f} 0.775 0 0 0"),
    "ball_b": ("<pose>0.40 0.08 0.775 0 0 0</pose>", "{x:.4f} {y:.4f} 0.775 0 0 0"),
    "basket_a": ("<pose>0.65 -0.08 0 0 0 0</pose>", "{x:.4f} {y:.4f} 0 0 0 0"),
    "basket_b": ("<pose>0.65 0.08 0 0 0 0</pose>", "{x:.4f} {y:.4f} 0 0 0 0"),
}

OBJECTS = [
    ("basket_b", lib.BASKET_CLEARANCE_M),
    ("basket_a", lib.BASKET_CLEARANCE_M),
    ("ball_b", lib.BALL_CLEARANCE_M),
    ("ball_a", lib.BALL_CLEARANCE_M),
]


def enforce_ball_a_on_right(layout: dict) -> dict:
    if layout["ball_a"][1] > layout["ball_b"][1]:
        layout["ball_a"], layout["ball_b"] = layout["ball_b"], layout["ball_a"]
    return layout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-runs", type=int, default=10)
    parser.add_argument("--start-index", type=int, default=1,
                         help="First run number to use (e.g. 6 to continue a sweep after runs 1-5).")
    parser.add_argument("--output-dir", type=Path, default=lib.REPO_ROOT / "scene3_runs")
    parser.add_argument("--gui", action="store_true", help="Show the Gazebo GUI (default: headless).")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--mission-timeout", type=float, default=180.0,
                         help="Max seconds to wait for a single run's mission to finish.")
    args = parser.parse_args()

    lib.run_sweep(
        args, run_prefix="scene3_run", launch_file="ball_sort_demo3.launch.py",
        world_template_path=WORLD_TEMPLATE_PATH, pose_tokens=POSE_TOKENS, objects=OBJECTS,
        basket_object="basket_b", log_tag="run_scene3_sweep",
        layout_fixup=enforce_ball_a_on_right,
    )


if __name__ == "__main__":
    main()
