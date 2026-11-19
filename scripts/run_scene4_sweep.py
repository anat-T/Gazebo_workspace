#!/usr/bin/env python3
"""Runs scene4 (ball_sort_demo4.launch.py) N times, randomizing the layout
each run, and records each run to its own scene4_run{i}/ directory: the
rosbag (db3 + yaml), a video restored from the camera images, and the
end-effector trajectory csv. Same procedure as run_scene1/2/3_sweep.py -
see scene_sweep_lib.py for the shared implementation.

Scene 4: everything is blue - 3 balls and 4 baskets. The robot picks the
RIGHT-most ball (smallest y) and places it in the LEFT-most basket (largest
y, out of all four). Layout per run:
- Balls: a tight row of three, side by side across y, --ball-gap apart
  (surface to surface). ball_a is always the right-most one - the gripper's
  DetachableJoint is hardwired to ball_a at spawn time (see
  ball_sort_demo4.launch.py), and the mission picks whichever ball has the
  smallest y, so the two have to agree.
- Baskets: basket_a/b/c in a straight row across y (same x, right to left,
  BASKET_ROW_SPACING_M apart), plus basket_d placed freely anywhere else -
  at least OFF_ROW_MIN_DX_M off the row's line, so it never reads as a 4th
  member of the row.
- The target basket (left-most of the four - can be basket_c or basket_d)
  is recomputed every run and passed to the mission as basket_x/basket_y.
- Clearances are the same as scenes 1-3 (scene_sweep_lib's
  BALL_CLEARANCE_M / BASKET_CLEARANCE_M): every ball-basket and
  basket-basket pair keeps the sum of their two clearance radii.

Unlike scenes 1-3, the layout is sampled as a whole (rejection sampling
over complete layouts, here) rather than one object at a time via
scene_sweep_lib.sample_layout(): with this many objects in the reachable
area, one-at-a-time placement regularly paints itself into a corner, and
sample_layout()'s fallback for that is to accept an overlapping position.
This script instead raises if no valid layout turns up.

Usage (after sourcing /opt/ros/humble/setup.bash and install/setup.bash):
    python3 run_scene4_sweep.py [--num-runs 10] [--start-index 1]
                                 [--output-dir scene4_runs] [--gui] [--seed 42]
                                 [--ball-gap 0.002]
"""
import argparse
import math
import random
from pathlib import Path

import scene_sweep_lib as lib

WORLD_TEMPLATE_PATH = lib.REPO_ROOT / "ros2_ws/src/box_pick_place_demo/worlds/ball_sort_demo4.sdf"

# Exact pose lines in worlds/ball_sort_demo4.sdf - matched verbatim and
# substituted per run, so each must appear exactly once in that file. If
# its default positions ever change, these must be updated to match
# (write_world_file() raises if one isn't found).
POSE_TOKENS = {
    "ball_a": ("<pose>0.40 -0.052 0.775 0 0 0</pose>", "{x:.4f} {y:.4f} 0.775 0 0 0"),
    "ball_b": ("<pose>0.40 0 0.775 0 0 0</pose>", "{x:.4f} {y:.4f} 0.775 0 0 0"),
    "ball_c": ("<pose>0.40 0.052 0.775 0 0 0</pose>", "{x:.4f} {y:.4f} 0.775 0 0 0"),
    "basket_a": ("<pose>0.65 -0.24 0 0 0 0</pose>", "{x:.4f} {y:.4f} 0 0 0 0"),
    "basket_b": ("<pose>0.65 0 0 0 0 0</pose>", "{x:.4f} {y:.4f} 0 0 0 0"),
    "basket_c": ("<pose>0.65 0.24 0 0 0 0</pose>", "{x:.4f} {y:.4f} 0 0 0 0"),
    "basket_d": ("<pose>0.40 -0.26 0 0 0 0</pose>", "{x:.4f} {y:.4f} 0 0 0 0"),
}

BALL_NAMES = ["ball_a", "ball_b", "ball_c"]          # right -> left
ROW_BASKETS = ["basket_a", "basket_b", "basket_c"]   # right -> left
FREE_BASKET = "basket_d"
BASKET_NAMES = ROW_BASKETS + [FREE_BASKET]

# Row baskets as close as the usual basket-basket clearance allows.
BASKET_ROW_SPACING_M = 2 * lib.BASKET_CLEARANCE_M
# basket_d must sit at least one full basket width (outer wall to outer
# wall) off the row's line, so it's visibly not part of the row.
OFF_ROW_MIN_DX_M = 2 * lib.BASKET_HALF_WIDTH_M
MAX_LAYOUT_ATTEMPTS = 200_000

# Surface-to-surface gap between neighboring balls. Measured, not guessed
# (2026-11-15, real two-ball scene4 missions with the neighbor ball's pose
# tracked throughout):
# - 0mm (touching): 1 of 2 missions blew up - the instant the picked ball
#   was attached while in contact with its neighbor, the physics engine
#   flung BOTH balls off the table (neighbor ended ~9m away).
# - 2mm and 5mm: 2 of 2 missions each succeeded, neighbor moved 0.0mm.
# Any nonzero gap works because nothing else touches the neighbor: there's
# no gripper model in this sim, and the picked ball is held at its own
# resting pose and lifted straight up. 2mm = the closest tested-safe gap.
DEFAULT_BALL_GAP_M = 0.002

TARGET_BASKET_KEY = "target_basket"


def _layout_ok(layout: dict) -> bool:
    min_bb = 2 * lib.BASKET_CLEARANCE_M
    min_ball_basket = lib.BALL_CLEARANCE_M + lib.BASKET_CLEARANCE_M
    for i, a in enumerate(BASKET_NAMES):
        for b in BASKET_NAMES[i + 1:]:
            if math.dist(layout[a], layout[b]) < min_bb:
                return False
        for ball in BALL_NAMES:
            if math.dist(layout[a], layout[ball]) < min_ball_basket:
                return False
    return abs(layout[FREE_BASKET][0] - layout[ROW_BASKETS[0]][0]) >= OFF_ROW_MIN_DX_M


def sample_scene4_layout(rng: random.Random, spacing: float) -> dict:
    (x_min, x_max), (y_min, y_max) = lib.REACHABLE_X, lib.REACHABLE_Y
    row_half = BASKET_ROW_SPACING_M  # center basket to either end basket
    balls_half = spacing             # middle ball to either end ball
    for _ in range(MAX_LAYOUT_ATTEMPTS):
        layout = {}
        rx, ry = rng.uniform(x_min, x_max), rng.uniform(y_min + row_half, y_max - row_half)
        for k, name in enumerate(ROW_BASKETS):
            layout[name] = (rx, ry + (k - 1) * BASKET_ROW_SPACING_M)
        bx, by = rng.uniform(x_min, x_max), rng.uniform(y_min + balls_half, y_max - balls_half)
        for k, name in enumerate(BALL_NAMES):
            layout[name] = (bx, by + (k - 1) * spacing)
        layout[FREE_BASKET] = (rng.uniform(x_min, x_max), rng.uniform(y_min, y_max))
        if _layout_ok(layout):
            layout[TARGET_BASKET_KEY] = max((layout[n] for n in BASKET_NAMES), key=lambda xy: xy[1])
            return layout
    raise RuntimeError(f"run_scene4_sweep: no valid layout found in {MAX_LAYOUT_ATTEMPTS} attempts")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--num-runs", type=int, default=10)
    parser.add_argument("--start-index", type=int, default=1,
                         help="First run number to use (e.g. 6 to continue a sweep after runs 1-5).")
    parser.add_argument("--output-dir", type=Path, default=lib.REPO_ROOT / "scene4_runs")
    parser.add_argument("--gui", action="store_true", help="Show the Gazebo GUI (default: headless).")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--mission-timeout", type=float, default=180.0,
                         help="Max seconds to wait for a single run's mission to finish.")
    parser.add_argument("--ball-gap", type=float, default=DEFAULT_BALL_GAP_M,
                         help=f"Gap between neighboring balls' surfaces, in meters (default {DEFAULT_BALL_GAP_M}).")
    args = parser.parse_args()
    if args.ball_gap < 0:
        parser.error("--ball-gap can't be negative (the balls would overlap)")

    spacing = 2 * lib.BALL_RADIUS_M + args.ball_gap
    print(f"run_scene4_sweep: ball gap {args.ball_gap * 1000:.1f}mm "
          f"(center-to-center {spacing * 1000:.1f}mm)")

    # run_once() always calls lib.sample_layout(rng, objects) first, then
    # layout_fixup on the result. objects=[] makes the first step a no-op,
    # and the fixup does the real (whole-layout) sampling - with its own
    # rng, seeded from --seed, so --seed still reproduces a sweep.
    layout_rng = random.Random(args.seed)
    lib.run_sweep(
        args, run_prefix="scene4_run", launch_file="ball_sort_demo4.launch.py",
        world_template_path=WORLD_TEMPLATE_PATH, pose_tokens=POSE_TOKENS, objects=[],
        basket_object=TARGET_BASKET_KEY, log_tag="run_scene4_sweep",
        layout_fixup=lambda _empty: sample_scene4_layout(layout_rng, spacing),
    )


if __name__ == "__main__":
    main()
