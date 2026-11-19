"""Self-contained numerical IK for a UR5e arm's real gripper TCP (not bare
wrist_3_link/tool0), built directly from ur_description's own fixed joint
origins (config/ur5e/default_kinematics.yaml) rather than pulling in MoveIt.


All six UR revolute joints rotate about their own local +Z axis; the fixed
offsets below (translation + rpy) are exactly what turns that into the real
arm geometry, taken verbatim from ur_description's default_kinematics.yaml
plus the base_link -> base_link_inertia 180-degree yaw flip that
ur_macro.xacro applies (REP-103 base_link is X+-forward; the internal
kinematic chain is X+-backward).
"""
import numpy as np

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

_BASE_FLIP = ((0.0, 0.0, 0.0), (0.0, 0.0, np.pi))  # base_link -> base_link_inertia

_JOINT_ORIGINS = [
    ((0.0, 0.0, 0.1625), (0.0, 0.0, 0.0)),                      # -> shoulder_link
    ((0.0, 0.0, 0.0), (1.570796327, 0.0, 0.0)),                 # -> upper_arm_link
    ((-0.425, 0.0, 0.0), (0.0, 0.0, 0.0)),                      # -> forearm_link
    ((-0.3922, 0.0, 0.1333), (0.0, 0.0, 0.0)),                  # -> wrist_1_link
    ((0.0, -0.0997, 0.0), (1.570796327, 0.0, 0.0)),             # -> wrist_2_link
    ((0.0, 0.0996, 0.0), (1.570796326589793, 3.141592653589793, 3.141592653589793)),  # -> wrist_3_link
]

# wrist_3_link -> flange -> tool0: fixed joints from ur_description's own
# ur_macro.xacro, both zero-translation (rotation only).
_FLANGE_OFFSET = ((0.0, 0.0, 0.0), (0.0, -1.570796327, -1.570796327))
_TOOL0_OFFSET = ((0.0, 0.0, 0.0), (1.570796327, 0.0, 1.570796327))

# tool0 -> the real gripper's own TCP (OnRobot 2FG7 + mount) - measured
# empirically, not from any CAD/datasheet: jogged the real gripper to touch
# a known point, then compared the UR controller's own
# /tcp_pose_broadcaster reading (which knows the real, configured tool
# offset) against this file's forward_kinematics() output at that same
# joint configuration before this offset existed. They disagreed by 13.9cm,
# and converting that gap into tool0's own local frame put it almost
# entirely on the local Z axis (X/Y components were 1-2mm, i.e. noise) -
# consistent with the gripper extending straight out along tool0's Z with
# no lateral offset. Before this, forward_kinematics() had no notion of the
# real gripper's length at all and silently targeted bare tool0 - meaning
# every move_to_world_point() call was landing this far short of the real
# target (toward the robot) the whole time.
_GRIPPER_TCP_OFFSET = ((0.0, 0.0, 0.139), (0.0, 0.0, 0.0))


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rot_z @ rot_y @ rot_x


def _fixed_transform(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = _rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def _rot_z(theta):
    t = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    t[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return t


def forward_kinematics(q) -> np.ndarray:
    """4x4 pose of the real gripper's own TCP (OnRobot 2FG7 + mount, not
    bare wrist_3_link/tool0) relative to base_link (REP-103 aligned, i.e.
    this robot's spawn frame in world coordinates). Chain: the 6 solved-for
    UR joints, then three fixed offsets (wrist_3_link -> flange -> tool0 ->
    gripper TCP)."""
    t = _fixed_transform(*_BASE_FLIP)
    for (xyz, rpy), theta in zip(_JOINT_ORIGINS, q):
        t = t @ _fixed_transform(xyz, rpy) @ _rot_z(theta)
    t = t @ _fixed_transform(*_FLANGE_OFFSET) @ _fixed_transform(*_TOOL0_OFFSET) @ _fixed_transform(*_GRIPPER_TCP_OFFSET)
    return t


# A generic "reaching forward and down over the table" seed - not any
# particular target, just a sane nullspace bias / starting point.
DEFAULT_SEED = np.array([0.0, -1.2, 1.6, -1.97, -1.57, 0.0])

# Real UR5e limits (ur_description's config/ur5e/joint_limits.yaml) - most
# joints are +-360deg, but elbow_joint is physically constrained to just
# +-180deg (the shoulder_lift joint gets in the way past that - see that
# file's comment). The solvers below only check Cartesian position/
# orientation error to decide "converged"; since each joint's rotation is
# 2*pi-periodic, that check alone can be satisfied by a solution that's
# wrapped some joint an extra full turn into a value nowhere near this
# range - confirmed happening in practice (elbow_joint solved to roughly
# -440deg for a real, if awkward, real-hardware target) - and since each
# solve's result seeds the next one, one such solution compounds into
# worse ones downstream. _wrap_and_validate() below is the fix.
JOINT_LIMITS_RAD = [
    (-2 * np.pi, 2 * np.pi),  # shoulder_pan_joint
    (-2 * np.pi, 2 * np.pi),  # shoulder_lift_joint
    (-np.pi, np.pi),          # elbow_joint - physically constrained, not +-2pi
    (-2 * np.pi, 2 * np.pi),  # wrist_1_joint
    (-2 * np.pi, 2 * np.pi),  # wrist_2_joint
    (-2 * np.pi, 2 * np.pi),  # wrist_3_joint
]


def _wrap_and_validate(q, seed):
    """Re-express each joint of `q` at its equivalent angle nearest to the
    corresponding `seed` value (forward_kinematics is unaffected, since
    shifting any joint by a multiple of 2*pi doesn't change _rot_z's
    result), then check the wrapped result against JOINT_LIMITS_RAD. Returns
    the wrapped q, or None if it's still out of range for some joint even
    after wrapping (a genuine failure, not a wrapping artifact) - e.g.
    elbow_joint needing more than +-180deg of motion to reach the target at
    all, which no amount of rewrapping fixes."""
    wrapped = seed + (np.asarray(q) - seed + np.pi) % (2 * np.pi) - np.pi
    for angle, (lo, hi) in zip(wrapped, JOINT_LIMITS_RAD):
        if not (lo <= angle <= hi):
            return None
    if not configuration_ok(wrapped):
        return None
    return wrapped


# Configuration-family check. Any reachable target has several IK
# solutions (elbow up/down, shoulder facing toward/away, wrist flipped), and
# the numerical solvers below return whichever one their seed happens to
# lead to - nothing else picks between them. Measured in sim (scene1 run21,
# scene4 runs 13/19/21 - every ball-sort failure on record that wasn't a
# missed grasp): each one had the arm in an ELBOW-DOWN configuration for
# the pick and/or the release, with the elbow joint as low as 10cm BELOW
# the table surface (the robot's base plane). Driving an arm link into the
# table is what flung the ball off it - the ball ended on the floor, 1-2m
# away. Every successful move in those same sweeps was elbow-up, elbow
# 6-35cm above the shoulder-wrist line and 45-60cm above the base plane.
# (Facing away from the target - shoulder_pan ~180deg from the target's
# azimuth, reaching back over the top - also showed up in successful runs,
# with the elbow up; it isn't what breaks things, so it isn't rejected.)
#
# Elbow height is measured relative to the line from the shoulder
# (upper_arm_link origin) to the wrist (wrist_1_link origin): positive = the
# elbow is above that line = elbow-up.
MIN_ELBOW_ABOVE_LINE_M = 0.02
# Every joint origin from the shoulder onward must stay this far above the
# robot's base plane (= the table surface it's mounted on) - roughly one
# UR5e link radius, so the link's own body clears the table too.
MIN_JOINT_HEIGHT_M = 0.05


def _joint_origins(q):
    """World positions (relative to base_link) of the 6 joint frames:
    shoulder_link, upper_arm_link, forearm_link (elbow), wrist_1/2/3_link."""
    t = _fixed_transform(*_BASE_FLIP)
    out = []
    for (xyz, rpy), theta in zip(_JOINT_ORIGINS, q):
        t = t @ _fixed_transform(xyz, rpy) @ _rot_z(theta)
        out.append(t[:3, 3].copy())
    return out


def elbow_height_above_line(q) -> float:
    _, shoulder, elbow, wrist, _, _ = _joint_origins(q)
    v = wrist - shoulder
    closest = shoulder + np.dot(elbow - shoulder, v) / np.dot(v, v) * v
    return float(elbow[2] - closest[2])


def configuration_ok(q) -> bool:
    """True if q is elbow-up and keeps every joint above the table - see
    the comment block above for the measurements behind both rules."""
    if elbow_height_above_line(q) < MIN_ELBOW_ABOVE_LINE_M:
        return False
    return all(p[2] >= MIN_JOINT_HEIGHT_M for p in _joint_origins(q)[1:])


def _position_jacobian(q, eps=1e-6):
    j = np.zeros((3, 6))
    p0 = forward_kinematics(q)[:3, 3]
    for i in range(6):
        dq = np.array(q, dtype=float)
        dq[i] += eps
        p1 = forward_kinematics(dq)[:3, 3]
        j[:, i] = (p1 - p0) / eps
    return j


def solve_ik_position(target_xyz, seed=None, max_iters=200, tol=1e-4, damping=0.05):
    """Damped-least-squares, position-only IK (3 constraints, 6 DOF) for
    wrist_3_link's origin, with a nullspace bias pulling the redundant DOF
    toward `seed`. Returns a 6-vector of joint angles, or None if it fails
    to converge within 1 cm.

    Position-only leaves wrist orientation completely unconstrained, which
    let the solver reach a target from any angle including sideways or from
    below - looked wrong for a "pick from the table" motion and made the
    DetachableJoint attach at an unnatural, asymmetric orientation relative
    to the box. solve_ik_pose_down() below additionally constrains the
    wrist to point straight down and should be preferred for anything that
    needs to look like an actual top-down pick; this function is kept for
    moves where only position matters (e.g. move_home()'s fixed joint
    target, which doesn't go through IK at all, or diagnostics).
    """
    q = np.array(seed if seed is not None else DEFAULT_SEED, dtype=float)
    bias = np.array(seed if seed is not None else DEFAULT_SEED, dtype=float)
    target = np.asarray(target_xyz, dtype=float)

    for _ in range(max_iters):
        pos = forward_kinematics(q)[:3, 3]
        err = target - pos
        if np.linalg.norm(err) < tol:
            return _wrap_and_validate(q, bias)
        j = _position_jacobian(q)
        jjt = j @ j.T + (damping ** 2) * np.eye(3)
        dq_task = j.T @ np.linalg.solve(jjt, err)
        null_proj = np.eye(6) - j.T @ np.linalg.solve(jjt, j)
        dq_bias = 0.05 * (bias - q)
        q = q + dq_task + null_proj @ dq_bias

    pos = forward_kinematics(q)[:3, 3]
    if np.linalg.norm(target - pos) < 0.01:
        return _wrap_and_validate(q, bias)
    return None


def _pointing_error(q, target_dir=np.array([0.0, 0.0, -1.0])):
    """Cross product between the gripper TCP's local +Z axis (world frame -
    same orientation as tool0/wrist_3_link, since _GRIPPER_TCP_OFFSET is a
    pure translation) and the desired pointing direction - zero when
    aligned, otherwise points along the rotation axis needed to align them.
    This has only 2 real degrees of freedom (rotation about the target
    direction itself leaves it unchanged), which is exactly right for
    "point this axis down, don't care about yaw about it" - confirmed at
    DEFAULT_SEED, this axis already points at ~world -Z there (checked
    numerically), so this is the correct axis to constrain.
    """
    current_z = forward_kinematics(q)[:3, 2]
    return np.cross(current_z, target_dir)


def _pointing_aligned(q, target_dir=np.array([0.0, 0.0, -1.0])):
    """True if the gripper TCP's local +Z axis actually points TOWARD
    target_dir (dot product > 0), not just colinear with it.
    _pointing_error()'s cross-product alone can't tell "pointing at
    target_dir" from "pointing at -target_dir" (a 180-degree flip) - both
    give a near-zero cross product, since two anti-parallel vectors are
    still colinear. Confirmed happening for real: on real hardware,
    solve_ik_pose_down() converged to a solution with the gripper pointing
    straight UP (dot product exactly -1 with the intended down direction),
    and reported success since the cross-product error alone was
    satisfied - the arm picked from a wrong, upside-down orientation
    without any error. This explicit sign check is what the acceptance
    criteria was missing; solve_ik_pose_down() below now requires it in
    addition to the existing cross-product/position checks.
    """
    current_z = forward_kinematics(q)[:3, 2]
    return np.dot(current_z, target_dir) > 0


def _pose_jacobian(q, eps=1e-6):
    j = np.zeros((6, 6))
    p0 = forward_kinematics(q)[:3, 3]
    o0 = _pointing_error(q)
    for i in range(6):
        dq = np.array(q, dtype=float)
        dq[i] += eps
        p1 = forward_kinematics(dq)[:3, 3]
        o1 = _pointing_error(dq)
        j[:3, i] = (p1 - p0) / eps
        j[3:, i] = (o1 - o0) / eps
    return j


def solve_ik_pose_down(target_xyz, seed=None, max_iters=300, tol=1e-4, damping=0.05):
    """Like solve_ik_position(), but also constrains wrist_3_link's local
    +Z axis to point straight down (world -Z) - found necessary after the
    position-only solver was observed reaching grasp targets from the side
    rather than from above, which looked wrong and put the DetachableJoint
    attach point in an unnatural orientation relative to the box. Yaw about
    the pointing axis is still free (resolved by the nullspace bias, same
    as solve_ik_position()). Returns a 6-vector of joint angles, or None if
    it fails to converge (position within 1cm, pointing error within ~0.1
    rad, AND actually pointing down rather than the exact opposite - see
    _pointing_aligned()'s docstring for why that last check exists).
    """
    q = np.array(seed if seed is not None else DEFAULT_SEED, dtype=float)
    bias = np.array(seed if seed is not None else DEFAULT_SEED, dtype=float)
    target = np.asarray(target_xyz, dtype=float)

    for _ in range(max_iters):
        pos = forward_kinematics(q)[:3, 3]
        pos_err = target - pos
        orient_err = -_pointing_error(q)
        err = np.concatenate([pos_err, orient_err])
        if np.linalg.norm(pos_err) < tol and np.linalg.norm(orient_err) < 1e-3 and _pointing_aligned(q):
            return _wrap_and_validate(q, bias)
        j = _pose_jacobian(q)
        jjt = j @ j.T + (damping ** 2) * np.eye(6)
        dq_task = j.T @ np.linalg.solve(jjt, err)
        null_proj = np.eye(6) - j.T @ np.linalg.solve(jjt, j)
        dq_bias = 0.05 * (bias - q)
        q = q + dq_task + null_proj @ dq_bias

    pos = forward_kinematics(q)[:3, 3]
    orient_err = _pointing_error(q)
    if np.linalg.norm(target - pos) < 0.01 and np.linalg.norm(orient_err) < 0.1 and _pointing_aligned(q):
        return _wrap_and_validate(q, bias)
    return None
