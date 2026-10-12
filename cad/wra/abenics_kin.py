"""ABENICS kinematics (Abe, Tadakuma & Tadakuma, IEEE T-RO 37(5) 2021; inverse formulas in the
form used by F. Thomas, ARK 2022) for the WRA-1 wrist.  Pure numpy, no CAD dependency, so the
same file is used by the CAD assembly, calcs/ and the ROS 2 driver (copied there by build.py).

Geometry
  * Cross-spherical gear (CS-gear), pitch radius rho: involute ring teeth about its body axes
    g and h (orthogonal).  Output (tool) axis = body i = g x h.
  * Monopole k (pitch radius rho/2) sits on the fixed module axis u_k at distance 1.5*rho from
    the sphere centre.  Its carrier rotates about u_k by q_k; the monopole spins about the
    carrier axis I_m(q_k) (perpendicular to u_k) by Theta_k.
  * Pole-to-pole spherical-gear mesh: rotation of the sphere about u_k is free for monopole k;
    spinning the monopole by Theta tilts the sphere's body axis (g for k=1, h for k=2) by -Theta/2
    about I_m (2:1 ratio, external mesh):          R_col = Rot(I_m(q), -Theta/2) * u
Frames (housing frame A): x_A = tool axis at home (world I), u1 = G, u2 = H = x_A x ... with
(G, H, I) right-handed.  Module frame M_k: z = u_k, x = I, y = z x x;  I_m(q) = cos q x + sin q y.
"""
from __future__ import annotations

import math

import numpy as np

S2 = math.sqrt(0.5)
I_AX = np.array([1.0, 0.0, 0.0])                 # tool axis at home (x_A)
U = {1: np.array([0.0, S2, S2]),                  # module 1 axis (G): up and to +Y
     2: np.array([0.0, -S2, S2])}                 # module 2 axis (H): up and to -Y
assert np.allclose(np.cross(U[1], U[2]), I_AX)


def module_frame(k: int):
    z = U[k]
    x = I_AX
    y = np.cross(z, x)
    return x, y, z


def rot(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def i_m(k: int, q: float):
    x, y, _ = module_frame(k)
    return math.cos(q) * x + math.sin(q) * y


# Body axes of the sphere at home, expressed in A:  g = u1, h = u2, i = x_A
R_HOME = np.column_stack([U[1], U[2], I_AX])      # columns = body g, h, i in A


def fk(q1, th1, q2, th2):
    """Sphere orientation (columns = body g, h, i in frame A) from the four actuator angles (rad)."""
    g = rot(i_m(1, q1), -th1 / 2) @ U[1]
    h = rot(i_m(2, q2), -th2 / 2) @ U[2]
    h = h - g * (g @ h)                          # tiny inconsistency -> Gram-Schmidt
    h /= np.linalg.norm(h)
    i = np.cross(g, h)
    return np.column_stack([g, h, i])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ik_module(k, col, q_prev=0.0, window=None, eps=1e-6):
    """Return (q, theta) for monopole k so that the sphere's body axis (col) is reached.
    window = (lo, hi) in rad; the solution branch (n or -n) closest to q_prev inside it is used."""
    u = U[k]
    c = float(np.clip(u @ col, -1.0, 1.0))
    alpha = math.acos(c)
    n = np.cross(u, col)
    s = np.linalg.norm(n)
    if s < eps:                                  # pole-to-pole: q is free, keep the previous one
        return q_prev, 0.0
    n /= s
    x, y, _ = module_frame(k)
    qa = math.atan2(n @ y, n @ x)                # I_m = +n  -> theta = -2 alpha
    cands = [(qa, -2 * alpha), (_wrap(qa + math.pi), 2 * alpha)]
    best = None
    for q, th in cands:
        # unwrap near q_prev
        q = q_prev + _wrap(q - q_prev)
        ok = window is None or (window[0] - 1e-9 <= q <= window[1] + 1e-9)
        key = (0 if ok else 1, abs(q - q_prev))
        if best is None or key < best[0]:
            best = (key, q, th)
    return best[1], best[2]


def ik(R, q_prev=(0.0, 0.0), windows=None):
    """Actuator angles (q1, th1, q2, th2) for sphere orientation R (columns g, h, i in A)."""
    w1 = windows[0] if windows else None
    w2 = windows[1] if windows else None
    q1, t1 = ik_module(1, R[:, 0], q_prev[0], w1)
    q2, t2 = ik_module(2, R[:, 1], q_prev[1], w2)
    return q1, t1, q2, t2


def tool_rotation(tilt_dir_deg, tilt_deg, roll_deg):
    """Sphere orientation for a tool tilt (about an axis in the G-H plane at tilt_dir from G) and
    a roll about the (tilted) tool axis.  Returns R (columns g, h, i)."""
    a = math.radians(tilt_dir_deg)
    ax = math.cos(a) * U[1] + math.sin(a) * U[2]
    Rt = rot(ax, math.radians(tilt_deg))
    Rr = rot(I_AX, math.radians(roll_deg))
    return Rt @ Rr @ R_HOME


def tool_rotation_pyr(pitch_deg, yaw_deg, roll_deg):
    """Sphere orientation for a tool pose relative to the forearm (frame A):
    pitch = elevation of the tool axis (+ up), yaw = azimuth toward +y_A, roll about the tool
    axis.  R = Rz(yaw) Ry(-pitch) Rx(roll) R_HOME   (columns g, h, i in A)."""
    return (rot([0, 0, 1], math.radians(yaw_deg)) @ rot([0, 1, 0], -math.radians(pitch_deg))
            @ rot(I_AX, math.radians(roll_deg)) @ R_HOME)


def tool_pyr(R):
    """Inverse of tool_rotation_pyr: (pitch, yaw, roll) in degrees."""
    M = R @ R_HOME.T
    pitch = math.degrees(math.asin(float(np.clip(M[2, 0], -1, 1))))
    yaw = math.degrees(math.atan2(M[1, 0], M[0, 0]))
    roll = math.degrees(math.atan2(M[2, 1], M[2, 2]))
    return pitch, yaw, roll


# Joint-space limits of the wrist (degrees); tilt toward a module axis is limited by module
# packaging against the claw sweep, Theta by the Theta-servo travel (servo = 0.75 Theta, +/-180).
LIMITS = dict(tilt=(30.0, 33.0), toward_module=(21.0, 24.0), theta=(220.0, 236.0), q=(120.0, 125.0))


def wrist_check(R, act=None, hard=False):
    """Check a sphere orientation (and optionally its actuator solution) against the wrist limits.
    Returns (ok, info dict)."""
    j = 1 if hard else 0
    t = R[:, 2]
    tilt = math.degrees(math.acos(float(np.clip(t @ I_AX, -1, 1))))
    tow = [math.degrees(math.asin(float(np.clip(t @ U[k], -1, 1)))) for k in (1, 2)]
    info = dict(tilt=tilt, toward_u1=tow[0], toward_u2=tow[1])
    ok = tilt <= LIMITS["tilt"][j] + 1e-6 and max(tow) <= LIMITS["toward_module"][j] + 1e-6
    if act is not None:
        q1, t1, q2, t2 = act
        info.update(q1=math.degrees(q1), th1=math.degrees(t1), q2=math.degrees(q2), th2=math.degrees(t2))
        ok = ok and max(abs(math.degrees(t1)), abs(math.degrees(t2))) <= LIMITS["theta"][j] + 1e-6
        ok = ok and max(abs(math.degrees(q1)), abs(math.degrees(q2))) <= LIMITS["q"][j] + 1e-6
    return ok, info


def mirror_pyr(pitch, yaw, roll):
    """LEFT <-> RIGHT task mirroring (reflection in the arm's x-z plane)."""
    return pitch, -yaw, -roll


def tool_angles(R):
    """(tilt_deg, roll_deg) of the tool for sphere orientation R."""
    i = R[:, 2]
    c = float(np.clip(i @ I_AX, -1, 1))
    tilt = math.degrees(math.acos(c))
    # remove the tilt with the shortest rotation taking i back to I, then read the roll of g about I
    ax = np.cross(i, I_AX)
    s = np.linalg.norm(ax)
    R0 = R if s < 1e-9 else rot(ax / s, math.acos(c)) @ R
    g0 = R0[:, 0]
    roll = math.degrees(math.atan2(np.cross(U[1], g0) @ I_AX, U[1] @ g0))
    return tilt, roll


def module_axes(k, q):
    """Torque axes of module k for carrier angle q: actuated axis a = I_m (monopole spin -> sphere
    rotation about the parallel axis) and passive axis p = u x I_m (reacted by the carrier bearing
    structure, no actuator torque).  Module k transmits no torque about u_k (free S-gear rotation)."""
    a = i_m(k, q)
    return a, np.cross(U[k], a)


def actuator_torques(q1, q2, tau_out):
    """Static load split for an output torque tau_out (N m, frame A) acting on the sphere.
    Returns dict(t_a1, t_p1, t_a2, t_p2): sphere-side torques about the actuated / passive axes,
    choosing the internal (redundant) component that minimises t_a1^2 + t_a2^2."""
    a1, p1 = module_axes(1, q1)
    a2, p2 = module_axes(2, q2)
    M = np.column_stack([a1, p1, a2, p2])            # 3 x 4
    tau = np.asarray(tau_out, float)
    # minimise ||W x|| subject to M x = tau with heavy weight on actuated, light on passive
    W = np.diag([1.0, 1e-3, 1.0, 1e-3])
    Winv2 = np.linalg.inv(W @ W)
    x = Winv2 @ M.T @ np.linalg.solve(M @ Winv2 @ M.T, tau)
    return dict(zip(("t_a1", "t_p1", "t_a2", "t_p2"), x))


def self_test():
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(200):
        R = tool_rotation(rng.uniform(0, 360), rng.uniform(0, 30), rng.uniform(-90, 90))
        q = ik(R)
        R2 = fk(*q)
        worst = max(worst, float(np.abs(R2 - R).max()))
    return worst


if __name__ == "__main__":
    print("max FK(IK(R)) - R error:", self_test())
    for pyr in ((-30, 0, 0), (20, 15, -40), (0, 0, 90)):
        assert np.allclose(tool_pyr(tool_rotation_pyr(*pyr)), pyr), pyr
    for name, R in (("pitch +20 (tilt toward G)", tool_rotation(90, 20, 0)),
                    ("yaw +20", tool_rotation(0, 20, 0)), ("roll +60", tool_rotation(0, 0, 60))):
        q1, t1, q2, t2 = ik(R)
        print(f"{name:28s} q1 {math.degrees(q1):7.1f}  th1 {math.degrees(t1):7.1f}   q2 {math.degrees(q2):7.1f}  th2 {math.degrees(t2):7.1f}")
