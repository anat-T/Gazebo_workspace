"""Self-contained numerical IK for a UR5e arm's wrist_3_link, built directly
from ur_description's own fixed joint origins (config/ur5e/default_kinematics.yaml)
rather than pulling in MoveIt.


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
    """4x4 pose of wrist_3_link relative to base_link (REP-103 aligned,
    i.e. this robot's spawn frame in world coordinates)."""
    t = _fixed_transform(*_BASE_FLIP)
    for (xyz, rpy), theta in zip(_JOINT_ORIGINS, q):
        t = t @ _fixed_transform(xyz, rpy) @ _rot_z(theta)
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
    return wrapped


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
    """Cross product between wrist_3_link's local +Z axis (world frame) and
    the desired pointing direction - zero when aligned, otherwise points
    along the rotation axis needed to align them. This has only 2 real
    degrees of freedom (rotation about the target direction itself leaves
    it unchanged), which is exactly right for "point this axis down, don't
    care about yaw about it" - confirmed at DEFAULT_SEED, wrist_3_link's
    local +Z already points at ~world -Z there (checked numerically), so
    this is the correct axis to constrain.
    """
    current_z = forward_kinematics(q)[:3, 2]
    return np.cross(current_z, target_dir)


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
    rad).
    """
    q = np.array(seed if seed is not None else DEFAULT_SEED, dtype=float)
    bias = np.array(seed if seed is not None else DEFAULT_SEED, dtype=float)
    target = np.asarray(target_xyz, dtype=float)

    for _ in range(max_iters):
        pos = forward_kinematics(q)[:3, 3]
        pos_err = target - pos
        orient_err = -_pointing_error(q)
        err = np.concatenate([pos_err, orient_err])
        if np.linalg.norm(pos_err) < tol and np.linalg.norm(orient_err) < 1e-3:
            return _wrap_and_validate(q, bias)
        j = _pose_jacobian(q)
        jjt = j @ j.T + (damping ** 2) * np.eye(6)
        dq_task = j.T @ np.linalg.solve(jjt, err)
        null_proj = np.eye(6) - j.T @ np.linalg.solve(jjt, j)
        dq_bias = 0.05 * (bias - q)
        q = q + dq_task + null_proj @ dq_bias

    pos = forward_kinematics(q)[:3, 3]
    orient_err = _pointing_error(q)
    if np.linalg.norm(target - pos) < 0.01 and np.linalg.norm(orient_err) < 0.1:
        return _wrap_and_validate(q, bias)
    return None
