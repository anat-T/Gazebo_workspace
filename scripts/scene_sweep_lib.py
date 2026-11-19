#!/usr/bin/env python3
"""Shared logic behind run_scene1_sweep.py / run_scene2_sweep.py: run a
Gazebo ball-sort scene N times with randomized object positions, recording
each run's camera + ee-trajectory data into its own directory. Each
per-scene script just supplies that scene's launch file, world template,
object list, and which object's position becomes the basket_x/basket_y
launch arg - everything else (layout sampling, process lifecycle, the
fast-abort-on-process-death fix, post-processing) lives here so a fix only
needs to be made once.

Not meant to be run directly - see run_scene1_sweep.py's docstring for the
actual per-run behavior and usage.
"""
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
ROS_DOMAIN_ID = "33"  # must match the scene launch files' own ROS_DOMAIN_ID

# Roughly the arm's actual reach envelope for a top-down pick (subset of the
# table's full 1.0x0.7m surface, which is [-0.15,0.85]x[-0.35,0.35]) - keeps
# most runs producing a real pick+place instead of an IK-unreachable miss.
REACHABLE_X = (0.30, 0.75)
REACHABLE_Y = (-0.28, 0.28)

# Clearance radius per object (own footprint half-width + an approach
# buffer so the gripper can reach one object without sweeping through a
# neighbor) - required center-to-center separation between two objects is
# the sum of their two radii.
BALL_RADIUS_M = 0.025
BALL_CLEARANCE_M = 0.075
BASKET_HALF_WIDTH_M = 0.07  # outer wall face, from the scenes' own worlds/*.sdf
BASKET_CLEARANCE_M = 0.12
MAX_PLACEMENT_ATTEMPTS = 500

MISSION_DONE_RE = re.compile(r"=== ball_sort: (SUCCESS|FAILED)")
# Any of these dying mid-run means the mission can never produce a
# SUCCESS/FAILED line at all (confirmed happening for real: a stray
# leftover process from an earlier manual test held onto the same
# ROS_DOMAIN_ID's controller_manager services, so a fresh run's spawners
# timed out and died before the mission node ever started) - without this
# check the run sat recording for the full mission_timeout_s regardless,
# burning disk space on a bag that was never going to contain a real
# mission. Watches for the process-died log line over just polling
# launch_proc.poll(), since `ros2 launch` itself often keeps running even
# after a child action it depends on has already died.
PROCESS_DIED_RE = re.compile(
    r"process has died \[pid \d+, exit code -?\d+, cmd '([^']*(?:spawner|ball_region_node|ign gazebo|gz sim)[^']*)'"
)
# Outcomes where no real mission ever ran - the bag (if any) holds nothing
# useful (e.g. just camera frames from an idle scene), so it's deleted
# rather than left to eat disk space and get run through video/trajectory
# extraction for nothing.
NO_MISSION_OUTCOMES = {"PROCESS_DIED", "STARTUP_FAILED", "LAUNCH_DIED", "TIMEOUT"}

RUN_ENV = dict(os.environ, ROS_DOMAIN_ID=ROS_DOMAIN_ID)


def sample_layout(rng: random.Random, objects: list, log_tag: str) -> dict:
    """objects: list of (name, clearance_radius) tuples, placed in order."""
    placed = {}
    radii = dict(objects)
    for name, radius in objects:
        for attempt in range(MAX_PLACEMENT_ATTEMPTS):
            x = rng.uniform(*REACHABLE_X)
            y = rng.uniform(*REACHABLE_Y)
            ok = True
            for other_name, (ox, oy) in placed.items():
                if ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5 < radius + radii[other_name]:
                    ok = False
                    break
            if ok:
                placed[name] = (x, y)
                break
        else:
            print(f"{log_tag}: WARNING - could not find a clear spot for '{name}' "
                  f"after {MAX_PLACEMENT_ATTEMPTS} attempts, using last sampled (possibly overlapping) position")
            placed[name] = (x, y)
    return placed


def write_world_file(world_template_path: Path, pose_tokens: dict, layout: dict, out_path: Path, log_tag: str) -> None:
    content = world_template_path.read_text()
    for name, (old_pose, new_fmt) in pose_tokens.items():
        x, y = layout[name]
        new_pose = f"<pose>{new_fmt.format(x=x, y=y)}</pose>"
        if old_pose not in content:
            raise RuntimeError(
                f"{log_tag}: expected pose line for '{name}' not found in "
                f"{world_template_path} - the world file's default layout may have changed, "
                "update this scene's POSE_TOKENS to match"
            )
        content = content.replace(old_pose, new_pose)
    out_path.write_text(content)


def wait_for_topics(topics: list, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"], env=RUN_ENV, capture_output=True, text=True, timeout=10,
            )
            available = set(result.stdout.split())
            if all(t in available for t in topics):
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    return False


def _pgid_alive(pgid: int) -> bool:
    """True if any process remains in this group - os.killpg(pgid, 0) sends
    no actual signal, just probes for existence."""
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_process_group(proc: subprocess.Popen, name: str, log_tag: str, timeout_s: float = 20.0) -> None:
    """Escalates SIGINT -> SIGTERM -> SIGKILL to `proc`'s whole process
    group, verifying the GROUP is actually empty before returning - not
    just that `proc` itself (the direct child, e.g. `ros2 launch`) exited.
    Confirmed happening for real: `ros2 launch` can exit clean while a
    grandchild it spawned (an `ign gazebo` process, wrapped in a ruby
    launcher) is still alive and holding onto the shared ROS_DOMAIN_ID's
    controller_manager services - trusting proc.poll() alone let that
    orphan survive into the next run and break its spawners with the exact
    same "process has died" signature as a genuine crash, wasting a full
    run's worth of time and disk before anyone noticed the real cause."""
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    if proc.poll() is not None and not _pgid_alive(pgid):
        return

    for sig, sig_name, wait_s in (
        (signal.SIGINT, "SIGINT", timeout_s),
        (signal.SIGTERM, "SIGTERM", timeout_s),
        (signal.SIGKILL, "SIGKILL", 10.0),
    ):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        deadline = time.time() + wait_s
        while time.time() < deadline:
            try:
                proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                pass
            if not _pgid_alive(pgid):
                return
        if sig != signal.SIGKILL:
            print(f"{log_tag}: {name} still has live processes after {sig_name}, escalating")
    print(f"{log_tag}: {name} still has live processes even after SIGKILL - giving up, "
          f"a stray process may survive into the next run")


def run_once(run_index: int, output_dir: Path, rng: random.Random, gui: bool, mission_timeout_s: float, *,
             run_prefix: str, launch_file: str, world_template_path: Path, pose_tokens: dict,
             objects: list, basket_object: str, log_tag: str, layout_fixup=None) -> dict:
    run_name = f"{run_prefix}{run_index}"
    scratch_world = output_dir / f"_scratch_world_{run_index}.sdf"
    launch_log_path = output_dir / f"_scratch_launch_{run_index}.log"
    bag_dir = output_dir / run_name

    if bag_dir.exists():
        raise RuntimeError(f"{log_tag}: {bag_dir} already exists - remove it or pick a different --output-dir")

    layout = sample_layout(rng, objects, log_tag)
    if layout_fixup is not None:
        layout = layout_fixup(layout)
    write_world_file(world_template_path, pose_tokens, layout, scratch_world, log_tag)
    print(f"{log_tag}: [{run_name}] layout: "
          + ", ".join(f"{n}=({x:.3f},{y:.3f})" for n, (x, y) in layout.items()))

    gx, gy = layout[basket_object]
    launch_log = open(launch_log_path, "w")
    launch_proc = subprocess.Popen(
        [
            "ros2", "launch", "box_pick_place_demo", launch_file,
            f"world_file:={scratch_world}",
            f"gazebo_gui:={'true' if gui else 'false'}",
            f"basket_x:={gx:.4f}", f"basket_y:={gy:.4f}",
            "launch_mission:=true",
        ],
        env=RUN_ENV, stdout=launch_log, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    bag_proc = None
    outcome = "TIMEOUT"
    try:
        if not wait_for_topics(["/robot/camera/image", "/tf"], timeout_s=90.0):
            print(f"{log_tag}: [{run_name}] camera/tf topics never appeared - aborting this run")
            outcome = "STARTUP_FAILED"
        else:
            bag_proc = subprocess.Popen(
                ["ros2", "bag", "record", "-o", str(bag_dir),
                 "/robot/camera/image", "/tf", "/tf_static"],
                env=RUN_ENV, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            # Give the recorder a moment to actually attach before the
            # mission (already running in parallel) starts moving - a few
            # early frames/tf lost otherwise.
            time.sleep(2.0)

            deadline = time.time() + mission_timeout_s
            while time.time() < deadline:
                launch_log.flush()
                text = launch_log_path.read_text(errors="ignore")
                m = MISSION_DONE_RE.search(text)
                if m:
                    outcome = m.group(1)
                    break
                died = PROCESS_DIED_RE.search(text)
                if died:
                    print(f"{log_tag}: [{run_name}] a mission-critical process died early "
                          f"('{died.group(1)}') - aborting this run instead of waiting out the full timeout")
                    outcome = "PROCESS_DIED"
                    break
                if launch_proc.poll() is not None:
                    print(f"{log_tag}: [{run_name}] launch process exited early (code {launch_proc.returncode})")
                    outcome = "LAUNCH_DIED"
                    break
                time.sleep(1.0)
    finally:
        if bag_proc is not None:
            stop_process_group(bag_proc, f"{run_name} bag record", log_tag)
        stop_process_group(launch_proc, f"{run_name} launch", log_tag)
        launch_log.close()

    if bag_dir.exists():
        try:
            scratch_world.replace(bag_dir / "world.sdf")
        except OSError:
            pass
        try:
            launch_log_path.replace(bag_dir / "launch.log")
        except OSError:
            pass
    else:
        scratch_world.unlink(missing_ok=True)
        launch_log_path.unlink(missing_ok=True)

    return {"run_name": run_name, "layout": layout, "outcome": outcome, "bag_dir": bag_dir}


def postprocess(run: dict, log_tag: str) -> None:
    bag_dir = run["bag_dir"]
    if not bag_dir.exists() or not any(bag_dir.glob("*.db3")):
        print(f"{log_tag}: [{run['run_name']}] no bag data recorded - skipping video/trajectory extraction")
        return

    video_cmd = [
        sys.executable, str(SCRIPT_DIR / "bag_to_video.py"), str(bag_dir),
        "--topic", "/robot/camera/image", "--output", str(bag_dir / "video.mp4"),
    ]
    traj_cmd = [
        sys.executable, str(SCRIPT_DIR / "bag_to_ee_trajectory.py"), str(bag_dir),
        "--base-frame", "robot_base_link", "--ee-frame", "robot_tool0",
        "--output", str(bag_dir / "ee_trajectory.csv"),
    ]
    for cmd in (video_cmd, traj_cmd):
        result = subprocess.run(cmd, capture_output=True, text=True)
        print(result.stdout.strip())
        if result.returncode != 0:
            print(f"{log_tag}: [{run['run_name']}] {Path(cmd[1]).name} failed:\n{result.stderr}")


def run_sweep(args, *, run_prefix: str, launch_file: str, world_template_path: Path,
              pose_tokens: dict, objects: list, basket_object: str, log_tag: str, layout_fixup=None) -> list:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)

    results = []
    for i in range(args.start_index, args.start_index + args.num_runs):
        print(f"\n{log_tag}: === starting run {i} ({i - args.start_index + 1}/{args.num_runs}) ===")
        run = run_once(
            i, args.output_dir, rng, args.gui, args.mission_timeout,
            run_prefix=run_prefix, launch_file=launch_file, world_template_path=world_template_path,
            pose_tokens=pose_tokens, objects=objects, basket_object=basket_object, log_tag=log_tag,
            layout_fixup=layout_fixup,
        )
        print(f"{log_tag}: [{run['run_name']}] mission outcome: {run['outcome']}")
        if run["outcome"] in NO_MISSION_OUTCOMES:
            if run["bag_dir"].exists():
                shutil.rmtree(run["bag_dir"])
                print(f"{log_tag}: [{run['run_name']}] deleted - no real mission ran, nothing useful recorded")
        else:
            postprocess(run, log_tag)
        results.append(run)
        time.sleep(2.0)  # let ports/processes from this run fully settle before the next

    print(f"\n{log_tag}: === summary ===")
    for r in results:
        print(f"  {r['run_name']}: {r['outcome']}  -> {r['bag_dir']}")
    return results
