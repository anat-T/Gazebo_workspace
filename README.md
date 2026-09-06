# Box Pick-and-Place Gazebo Demo

Two UR5e arms in one shared Gazebo (Ignition Fortress) world, each mounted on
its own table with its own box. Both robots get a "belief" about where their
box is - the box's true simulated position plus a margin of uncertainty -
from the same ground-truth oracle. Robot A gets a wide margin (no help) and
takes one random guess at where to pick, which usually misses. Robot B gets
a narrow margin ("help") and takes one random guess too, but its margin is
tight enough that the guess reliably lands on the box. No camera or object
detection is involved in the mission itself - whether a guess "worked" is
decided by comparing it to the box's true position (standing in for
gripper/contact feedback a real robot would get from actually touching the
box). On a hit, the arm descends to touch the box, attaches it via the
`DetachableJoint` "gripper" (confirmed via the plugin's own state feedback,
not just assumed), and carries it back to the home pose. On a miss, the arm
just stays at its (empty) guessed spot.

See `/home/anat/.claude/plans/proud-sleeping-valley.md` for the original
design rationale. The mission mechanic described there (camera-based crop
search) was later replaced with this simpler single-guess version at the
user's request; this README reflects what's actually implemented and
verified working now.

## Important: network isolation

This lab's network carries other people's real ROS2 systems on the default
ROS domain (`ROS_DOMAIN_ID=0`). `demo.launch.py` sets `ROS_DOMAIN_ID=33` for
every process it launches, so running the demo itself is safe by default.
**But any `ros2` CLI command you run yourself in another terminal to
inspect/interact with it (`ros2 topic echo`, `ros2 control list_controllers`,
etc.) needs the same domain, or it won't see the demo's nodes at all:**

```bash
export ROS_DOMAIN_ID=33
```

Run this once per terminal before any manual `ros2 ...` command against this
demo. If you ever see unfamiliar nodes (e.g. something like
`/experiment_setup`) in `ros2 node list`, you're on the wrong domain.

## GPU / rendering note

This machine (a VMware VM with no GPU passthrough - `glxinfo` reports
`llvmpipe`/Mesa software rendering only) can only run Gazebo reliably
**with the GUI enabled**. Headless mode (`gazebo_gui:=false`, Gazebo's `-s`
flag) tries to get a hardware EGL/DRM render context first and segfaults
deep in Ogre2's material loading when that fails - confirmed via the
Gazebo/Ogre2 crash log and by testing with `sg video -c "sg render -c ..."`
(the user account being in the `video`/`render` groups didn't help - this
looks like the virtual GPU itself doesn't back a working DRM render node,
not a permissions problem). With `gazebo_gui:=true` and a real `DISPLAY`
available, Gazebo uses GLX/llvmpipe instead and has been stable across many
repeated runs. **On a machine with a real GPU this restriction shouldn't
apply** - headless mode should work normally there.

## One-time setup

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

No Python venv or extra pip packages are needed - the mission code only
depends on standard ROS2 Humble packages (rclpy, message_filters, cv_bridge,
tf2_msgs), all already installed system-wide.

## Running

```bash
export ROS_DOMAIN_ID=33
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch box_pick_place_demo demo.launch.py
```

`gazebo_gui` defaults to `true` - see the GPU note above for why. Other
useful launch arguments:
- `launch_missions:=false` - bring up the world/robots only, without
  starting the belief oracle or either robot's mission (useful for
  inspecting the scene or testing pieces manually)
- `no_help_margin:=0.40` / `help_margin:=0.03` - the belief-region
  half-extents (meters) for robot_a (no help) and robot_b (help). The
  default `help_margin` is at/below `GRASP_TOLERANCE_M` (0.04m, in
  `search_and_pick_node.py`), so every possible guess in robot_b's region
  is automatically within tolerance - help always works, no help almost
  never does (robot_a's region is ~100x the area of the tolerance window).

Watch each robot's log for its `=== robot_X: SUCCESS/FAILED ===` line -
that's the demo's punchline.

### Optional: SE_server tie-in

If `SE_server` is running (`uvicorn app.server:app --port 8000` from
`SE_server/state_server`), pass `check_se_server:=true` (as a parameter
override on the `search_and_pick_node` instances, or edit the default in
`launch/demo.launch.py`) to have a robot that succeeded confirm
`holding(box)` against it right after - purely a logged sanity check reusing
the existing camera + SE_server infrastructure, not required for the demo
to work. This grabs one frame from that robot's overhead camera; it does not
need the depth channel.

## Project layout

- `worlds/box_pick_demo.sdf` - the static scene: ground, two tables (each
  sized so the robot mounted on it and its box both fit on the same
  surface), two boxes, two ceiling-mounted RGB-D cameras (used only for the
  optional SE_server tie-in - the mission itself doesn't use them)
- `urdf/box_pick_robot.urdf.xacro` - one UR5e arm wrapped for Gazebo, plus a
  `DetachableJoint` plugin on the wrist (the "gripper": attach/detach the box
  rigidly rather than simulating real finger contact)
- `launch/demo.launch.py` - single entry point: Gazebo, both robot spawns
  (mounted on their tables), camera/pose/gripper topic bridges, belief
  oracle, both missions
- `box_pick_place_demo/belief.py` - the `SearchRegion` representation
  (`sample_point()` draws one uniformly-random guess from it)
- `box_pick_place_demo/belief_region_node.py` - the ground-truth oracle:
  reads each box's true pose from Gazebo's world pose stream, publishes
  true-position-plus-margin as that robot's belief region
- `box_pick_place_demo/search_and_pick_node.py` - the per-robot mission:
  wait for belief region -> guess one point -> move there -> compare to
  ground truth -> attach (or not) -> report SUCCESS/FAILED
- `box_pick_place_demo/arm_commander.py` / `ik_solver.py` - topic-commanded
  joint trajectories and the self-contained numerical IK behind
  `move_to_world_point()` (see `ik_solver.py`'s docstring for why this
  exists instead of MoveIt)
- `box_pick_place_demo/grasp_controller.py` - attach()/detach() over the
  `DetachableJoint` plugin's bridged topics
- `box_pick_place_demo/camera_sensor.py` / `se_server_client.py` - only used
  by the optional SE_server tie-in

## Status

Verified working end-to-end by actually launching the demo repeatedly,
watching the Gazebo GUI, and independently subscribing to
`/world/box_pick_demo/pose/info` to log each box's exact pose over time
(not just reading the code or trusting the mission's own log lines): world +
both robots spawn, both mounted on their tables; both
`joint_trajectory_controller`s come up active and reachable by topic;
`belief_region_node` publishes the correct region for each robot; each
mission's arm reaches its guessed point via real IK; on a miss the box never
moves at all; on a hit the arm touches down on the box, `attach()` is
confirmed via the plugin's own state topic (not assumed), and the box then
follows the arm smoothly through the lift and the return to home (checked
point-by-point in the pose log, not just before/after) with no jumps or
ejections. A representative run: robot_a FAILED (guessed ~0.08-0.42m off,
well outside the 0.04m tolerance, across many runs), robot_b SUCCEEDED
(guessed within ~0.01-0.04m every time tested) and ended holding the box at
its home pose.

## Rough edges found while building/testing this

- The installed `ign_ros2_control` plugin binary only recognizes the SDF tag
  `<controller_manager_name>`, not `<controller_manager_node_name>` (which is
  what `ur_description`'s own xacro uses upstream) - a version-skew bug on
  this machine, worked around in `box_pick_robot.urdf.xacro`.
- Its `robot_param_node` SDF tag (default: a node literally named
  "robot_state_publisher") also needs setting explicitly per robot, or both
  robots' `ign_ros2_control` instances fail to find their own
  `<ros2_control>` block and silently fall back to real-hardware defaults.
- Running two robots' `ign_ros2_control` plugin instances in one Gazebo
  process was observed cross-contaminating controller parameters (one
  robot's `joint_trajectory_controller` would come up with empty
  joints/command_interfaces) unless each spawner is also given its
  `--param-file` explicitly - see the comment in `demo.launch.py`.
- Gazebo's per-model `PosePublisher` plugin never actually publishes for a
  dynamic (non-`<static>`) body that's at rest from spawn - no pose-changed
  event ever fires, regardless of `<static_publisher>`/`<update_frequency>`
  tags. Switched to bridging `/world/<world>/pose/info` (always-on,
  SceneBroadcaster-driven) instead - see `belief_region_node.py`.
- SDF only allows `<plugin>` as a child of `<model>`/`<world>`/`<sensor>`,
  never `<link>` - wrapping the `DetachableJoint` plugin in
  `<gazebo reference="wrist_3_link">` (mirroring how link-specific URDF
  extensions normally work) silently dropped it during URDF->SDF conversion,
  so attach() had no effect at all even though the topics existed and the
  mission logic reported success. Fixed by using a bare `<gazebo>` block, the
  same way the `ign_ros2_control` plugin above it already does correctly -
  the plugin's own `<parent_link>` parameter is what targets the link, not
  the `reference` attribute. Found by manually publishing to the attach
  topic (both via the ROS bridge and as a raw Gazebo Transport message) and
  watching the box's world pose not move at all, then confirming zero
  "DetachableJoint" mentions anywhere in a full Gazebo startup log.
- Gazebo headless mode (`-s`) segfaults on this GPU-less VM - see the "GPU /
  rendering note" above. Run with `gazebo_gui:=true` (the default) instead.
- `grasp_controller.py`'s `attach()`/`detach()` publish to the
  `DetachableJoint` plugin's topics, but a single (or even a fixed-count
  retried) publish can lose a discovery-timing race against the plugin's own
  Gazebo-transport subscriber - and that race isn't resolved just because
  `ros2_control`'s controller is already active, since the two are separate
  systems with separate discovery. This showed up as the box getting
  violently dragged/ejected the instant the arm made its first motion, even
  with a `detach()` sent (and slept after) beforehand. Fixed for real by
  bridging the plugin's own `.../gripper/state` output topic (it publishes
  literal `"attached"`/`"detached"` strings on each transition) and having
  `attach()`/`detach()` retry-publish while spinning the node until that
  feedback confirms the transition actually happened - not by guessing at
  how long discovery takes.
- `JointTrajectory` interpolates joint angles linearly between the arm's
  current and target configuration - it has no notion of the Cartesian path
  that traces, so a single big move (e.g. home -> hovering over the guessed
  point) can sweep some other link (forearm/elbow, not just the wrist)
  straight through the box on the way, even though both endpoints are fine.
  Confirmed by watching the box start drifting mid-transition, well before
  the arm ever reached its low target. Mitigated (not truly fixed - there's
  still no real collision-aware path planning here) by routing every
  transition between the home pose and the guessed point through a high
  "safe transit" waypoint well above table height first, so the one large
  joint-space move happens between two elevated points instead of dipping
  toward the table mid-sweep.
- Driving the wrist to the box's exact center for a grasp guarantees direct
  collision interpenetration with the box's own collision geometry, which
  the physics engine then resolves violently. Fixed with a clearance height
  above the box for the general approach (`grasp_z`), then - only once a hit
  is confirmed and the arm is already stably hovering there - one further
  small, controlled descent much closer to the box (`touch_z`, a couple mm
  above its top face) so the pick looks like real contact rather than the
  arm hovering visibly above the box.
