#!/usr/bin/env python3
"""WRA-1 engineering calculations: masses, static joint torques, gravity springs, payload vs
reach, ABENICS wrist load split, belts, power, structure, gear teeth, accuracy budget,
workspace, door tasks and the knob claw.  Every input comes from cad/wra/params.yaml and the
CAD itself (centroids of the real solids, printed masses from output/reports/parts.json).

    python calcs/arm_calcs.py      -> calcs/output/*.png, results.json, springs.json,
                                      ../docs/05_calculations.md (generated report)

Run `python cad/build.py --parts` first so printed masses come from the exported parts.
Status tags: [V] verified source value, [C] computed here, [E] engineering estimate (replace
with a measurement), [D] design rule.  The active wrist is params.yaml wrist_type
(abenics = WRA-1A, default; pitch_roll = WRA-1B).
"""
from __future__ import annotations

import datetime
import json
import math
import pathlib
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
OUT = HERE / "output"
OUT.mkdir(exist_ok=True)
sys.path.insert(0, str(CAD))

from build123d import Location, Pos, Rot, Vector  # noqa: E402
from wra.config import P  # noqa: E402
from wra import assembly as asm, layout as L, lib, abenics_kin as AK  # noqa: E402
from wra.parts import wrist as wr, claw as cw, abenics as ab, arm as armp  # noqa: E402

G0 = 9.81
A = P.arm
S = P.servo
AB = P.abenics
WRIST = asm.WRIST
ABN = WRIST == "abenics"
TCP_X = cw.tcp_x()                      # TCP = middle of the claw pads, from the tool-flange face
R = {"variant": "WRA-1A (ABENICS wrist)" if ABN else "WRA-1B (pitch-roll wrist)",
     "generated": datetime.datetime.now().isoformat(timespec="seconds")}
inch = 25.4

# ============================================================================ 1. masses
STEEL, ALU = 7.85e-3, 2.70e-3            # g/mm^3
BEARING_G = {"6806": 24, "608": 12, "6802": 7, "6800": 5, "626": 8, "688": 4, "623": 1.3, "625": 4.5,
             "6808": 32, "6805": 22}      # [E] catalogue typical
VIT_MASS = {"V_6808": BEARING_G["6808"], "V_6805": BEARING_G["6805"], "V_688": BEARING_G["688"],
            "V_MGN9C_block": 16.0}        # [E] catalogue typical, overrides solid volume x density


def frame_key(name, group):
    if name in ("rod_end_P", "rod_tube"):
        return "RP"
    if name == "rod_end_Q":
        return "RQ"
    if name == "j3_hub":
        return "K"
    if name in ("j5_horn_adapter", "roll_spindle", "roll_spacer", "tool_flange"):
        return "R"
    if group in ("dock", "base"):
        return "B"
    if group == "turntable" or name.startswith(("stub", "ddm_pin")):
        return "T"
    if group == "upper_arm":
        return "U"
    if group == "tool":
        return "S"                       # CS gear (moves with the tool)
    if group == "gripper":
        return "G"
    if group == "wrist":
        return "W"
    if group in ("forearm", "wrist_m1", "wrist_m2"):
        return "F"                       # housing + modules: rigid with the forearm [E: carriers turn by q]
    return "T"


def all_frames(q):
    f = asm.frames(q)
    th2 = q["j2"]
    f["RP"] = f["T"] * Pos(f["P"].X, f["P"].Y, f["P"].Z) * Rot(0, -th2, 0)
    f["RQ"] = f["T"] * Pos(f["Q"].X, f["Q"].Y, f["Q"].Z) * Rot(0, -(th2 + 180.0), 0)
    f["B"] = Location()
    return f


def build_mass_model():
    """Point-mass model: (name, frame, local centroid, mass_g) from the real CAD solids."""
    parts_json = ROOT / "output" / "reports" / "parts.json"
    pm = {}
    if parts_json.exists():
        for r in json.loads(parts_json.read_text()):
            pm[r["part"]] = r["mass_g"]
    q0 = dict(asm.HOME, j6=0.0)
    f0 = all_frames(q0)
    model, missing = [], []
    for i in asm.arm_instances(q0):
        if i.shape is None or i.name.startswith(("belt", "cord", "spring")):
            continue
        fk = frame_key(i.name, i.group)
        c = i.shape.center()
        cl = (f0[fk].inverse() * Pos(c.X, c.Y, c.Z)).position
        if i.kind == "printed":
            key = i.key if i.key in pm else next((k for k in pm if k.startswith(i.key)), None)
            if key is None:
                missing.append(i.name)
                m = i.shape.volume * 1.27e-3 * 0.6
            else:
                m = pm[key]
            m *= 1.03                       # brass inserts / screws embedded [E]
        else:
            if i.key == "ST3215":
                m = S.mass_g
            elif i.key in VIT_MASS:
                m = VIT_MASS[i.key]
            elif "tube" in i.key and i.key != "V_tube8x1":
                m = i.shape.volume * ALU
            else:
                m = i.shape.volume * STEEL
        model.append((i.name, fk, (cl.X, cl.Y, cl.Z), m))
    extra = [("brg_6800x2_elbow", "U", (A.L2, armp.FA_Y, 0.0), 2 * BEARING_G["6800"]),
             ("brg_626_P", "RP", (0.0, 0.0, 0.0), BEARING_G["626"]),
             ("brg_626_Q", "RQ", (0.0, 0.0, 0.0), BEARING_G["626"]),
             ("cable_upper_arm", "U", (A.L2 / 2, A.ua_plane_y, 0.0), 15.0),
             ("cable_forearm", "F", (A.L3 / 2, armp.FA_Y, 0.0), 20.0 if ABN else 15.0),
             ("springs_j2", "U", (110.0, A.ua_plane_y, P.springs.j2_offset + 5), 30.0)]
    if not ABN:
        extra += [("brg_608x2_j4dm", "F", (A.L3 - L.C4, wr.TEETH4_C, 0.0), 2 * BEARING_G["608"]),
                  ("brg_688x2_pitch", "W", (0.0, wr.YC, 0.0), 2 * BEARING_G["688"]),
                  ("brg_6802x2_roll", "R", (38.0, wr.YC, 0.0), 2 * BEARING_G["6802"]),
                  ("belt_j4", "F", (A.L3 - L.C4 / 2, wr.TEETH4_C, 0.0), 9.0)]
    else:
        extra += [("abenics_screws_wires", "F", (A.L3, wr.YC, 40.0), 25.0)]
    return model + extra, missing


def _cached_mass_model():
    """The mass model needs the full CAD assembly (~1-2 min); cache it per params.yaml + parts.json."""
    import hashlib
    h = hashlib.sha256((CAD / "wra" / "params.yaml").read_bytes())
    pj = ROOT / "output" / "reports" / "parts.json"
    if pj.exists():
        h.update(pj.read_bytes())
    key = h.hexdigest()[:16]
    f = OUT / "mass_model_cache.json"
    if f.exists():
        d = json.loads(f.read_text())
        if d.get("key") == key:
            return [tuple(x[:2]) + (tuple(x[2]), x[3]) for x in d["model"]], d["missing"]
    model, missing = build_mass_model()
    f.write_text(json.dumps({"key": key, "model": model, "missing": missing}))
    return model, missing


MODEL, MISSING = _cached_mass_model()


def group_mass(frames_):
    return sum(m for _, fk, _, m in MODEL if fk in frames_)


R["mass_g"] = {"base_fixed_B": round(group_mass({"B"})), "turntable_T_incl_J1J2J3_servos": round(group_mass({"T"})),
               "upper_arm_U": round(group_mass({"U"})), "j3_hub_K": round(group_mass({"K"})),
               "parallelogram_rod": round(group_mass({"RP", "RQ"}))}
if ABN:
    R["mass_g"].update({"forearm_F_incl_ABENICS_housing_modules_4_servos": round(group_mass({"F"})),
                        "tool_S_cs_gear": round(group_mass({"S"})), "claw_G_incl_servo": round(group_mass({"G"}))})
else:
    R["mass_g"].update({"forearm_F_incl_J4": round(group_mass({"F"})), "wrist_W_incl_J5": round(group_mass({"W"})),
                        "roll_R": round(group_mass({"R"})), "claw_G_incl_servo": round(group_mass({"G"}))})
R["mass_g"]["arm_total_on_dock"] = round(sum(m for *_, m in MODEL))
R["mass_g"]["moving_beyond_J2"] = round(group_mass({"U", "K", "RP", "RQ", "F", "W", "R", "S", "G"}))
R["mass_model_missing_parts_json"] = MISSING
WRIST_KEYS = ("S", "G") if ABN else ("R", "G")          # bodies that move with the tool


# ============================================================================ 2. kinematics / torques
def tcp(q):
    f = asm.frames(dict(asm.HOME, **q))
    return (f["G"] * Pos(TCP_X, 0, 0)).position


def potential(q, payload_g=0.0, springs=None):
    """Gravity potential (N*mm) of everything beyond J1 (+payload at TCP) [+ spring energy]."""
    q = dict(asm.HOME, **q)
    f = all_frames(q)
    V = 0.0
    for _, fk, cl, m in MODEL:
        if fk in ("B", "T"):
            continue
        V += m / 1000 * G0 * (f[fk] * Pos(*cl)).position.Z
    if payload_g:
        V += payload_g / 1000 * G0 * (f["G"] * Pos(TCP_X, 0, 0)).position.Z
    if springs:
        th2, th3 = math.radians(q["j2"]), math.radians(q["j3"])
        a = P.springs.a
        k2, k3 = springs
        V += 0.5 * k2 * (a * a + P.springs.b2 ** 2 - 2 * a * P.springs.b2 * math.sin(th2))
        V += 0.5 * k3 * (a * a + P.springs.b3 ** 2 - 2 * a * P.springs.b3 * math.sin(th3))
    return V


ARM_JOINTS = ("j2", "j3") if ABN else ("j2", "j3", "j4")
THETA_RATIO = 2.0 * AB.theta_gear.mp_teeth / AB.theta_gear.servo_teeth   # sphere torque / Theta-servo torque = 1.5


def wrist_load(q, payload_g=0.0, f_tcp=None):
    """ABENICS: torque (N*m, frame A) that the actuators must apply to the sphere to hold gravity on
    the tool side (S, G frames + payload) and an optional external TCP force f_tcp (N, frame B),
    and the resulting Theta-servo torques (min-norm split)."""
    q = dict(asm.HOME, **q)
    f = all_frames(q)
    c = f["A"].position
    tau = np.zeros(3)
    for _, fk, cl, m in MODEL:
        if fk in WRIST_KEYS:
            p = (f[fk] * Pos(*cl)).position - c
            tau += np.cross([p.X, p.Y, p.Z], [0, 0, -m / 1000 * G0]) / 1000
    pt = (f["G"] * Pos(TCP_X, 0, 0)).position - c
    if payload_g:
        tau += np.cross([pt.X, pt.Y, pt.Z], [0, 0, -payload_g / 1000 * G0]) / 1000
    if f_tcp is not None:
        tau += np.cross([pt.X, pt.Y, pt.Z], f_tcp) / 1000
    Rb = np.array(_rotm(f["A"]))
    tau_A = Rb.T @ (-tau)                 # actuators apply the opposite of the load
    q1, t1, q2, t2 = f["act"]
    s = AK.actuator_torques(q1, q2, tau_A)
    return {"tau_A": tau_A, "w1": s["t_a1"] / THETA_RATIO, "w2": s["t_a2"] / THETA_RATIO,
            "passive": (s["t_p1"], s["t_p2"])}


def _rotm(loc):
    t = loc.wrapped.Transformation()
    return [[t.Value(i, j) for j in range(1, 4)] for i in range(1, 4)]


def joint_torques(q, payload_g=0.0, springs=None, h=0.05):
    """Static torque each ACTUATED joint must supply (N*m at the joint axis); ABENICS: also the
    two Theta-servo torques w1, w2 (N*m at the servo)."""
    out = {}
    for j in ARM_JOINTS:
        qa, qb = dict(q), dict(q)
        qa[j] = q.get(j, asm.HOME.get(j, 0.0)) + h
        qb[j] = q.get(j, asm.HOME.get(j, 0.0)) - h
        out[j] = (potential(qa, payload_g, springs) - potential(qb, payload_g, springs)) / math.radians(2 * h) / 1000.0
    if ABN:
        w = wrist_load(q, payload_g)
        out["w1"], out["w2"] = w["w1"], w["w2"]
    return out


RATIO = {"j1": P.pulleys.j1 / P.dm.teeth, "j2": P.pulleys.j2 / P.dm.teeth, "j3": P.pulleys.j3 / P.dm.teeth,
         "j6": 1.0}
if ABN:
    RATIO.update({"w1": 1.0, "w2": 1.0, "q1": 1.0, "q2": 1.0})       # torques quoted AT the servo
else:
    RATIO.update({"j4": P.pulleys.j4 / P.dm.teeth, "j5": 1.0})
TAU_STALL = S.stall_torque_Nm
TAU_CONT = {j: TAU_STALL * S.continuous_fraction * r for j, r in RATIO.items()}
TAU_PEAK = {j: TAU_STALL * S.peak_fraction * r for j, r in RATIO.items()}
R["joint_capacity_Nm"] = {j: {"ratio": round(RATIO[j], 4), "continuous": round(TAU_CONT[j], 3),
                              "peak": round(TAU_PEAK[j], 3), "stall": round(TAU_STALL * RATIO[j], 3)} for j in RATIO}
if ABN:
    R["joint_capacity_Nm"]["note"] = ("w1/w2 = Theta servos (quoted at the servo; sphere torque about the module's "
                                      f"actuated axis = {THETA_RATIO:.2f} x servo torque), q1/q2 = carrier servos "
                                      "(no static load from the tool: mesh forces act on the module axis)")


def valid(q):
    rel = q["j2"] - q["j3"]
    lo, hi = P.joints.j3_rel.soft
    if not (lo <= rel <= hi):
        return False
    if not (P.joints.j2.soft[0] <= q["j2"] <= P.joints.j2.soft[1]):
        return False
    if not (P.joints.j3.soft[0] <= q["j3"] <= P.joints.j3.soft[1]):
        return False
    if not ABN and rel > 125 and q.get("j4", 0) < 0:
        return False
    if ABN:
        ok, _ = AK.wrist_check(AK.tool_rotation_pyr(*q.get("wt", asm.HOME["wt"])))
        return ok
    return True


def wq(j2, j3, pitch=0.0, yaw=0.0, roll=0.0, j1=0.0, j6=0.0):
    """Arm pose dict for either wrist (pitch = tool pitch vs. the forearm; 1B: j4 = pitch)."""
    if ABN:
        return dict(j1=j1, j2=float(j2), j3=float(j3), wt=(float(pitch), float(yaw), float(roll)), j6=j6)
    return dict(j1=j1, j2=float(j2), j3=float(j3), j4=float(pitch), j5=float(roll), j6=j6)


WRIST_PITCHES = (-30.0, 0.0, 30.0) if ABN else (-90.0, -45.0, 0.0, 45.0, 90.0)
PITCH_LIM = AB.tilt_soft_deg if ABN else 105.0


# ============================================================================ 3. springs
def size_springs():
    """Zero-free-length balance: k*a*b = G (gravity coefficient) -> k = balance*G/(a*b)."""
    g2s, g3s = [], []
    for th2 in (-20.0, 0.0, 20.0, 40.0):
        for th3 in (-40.0, -20.0, 0.0):
            qq = wq(th2, th3)
            if not valid(qq):
                continue
            t = joint_torques(qq)
            g2s.append(t["j2"] / math.cos(math.radians(th2)))
            g3s.append(t["j3"] / math.cos(math.radians(th3)))
    G2, G3 = float(np.mean(g2s)), float(np.mean(g3s))
    a, b2, b3 = P.springs.a, P.springs.b2, P.springs.b3
    bal = P.springs.balance_fraction
    k2 = bal * G2 * 1000 / (a * b2)
    k3 = bal * G3 * 1000 / (a * b3)
    n = P.springs.per_joint
    sp = {}
    for j, k, b, G in (("j2", k2, b2, G2), ("j3", k3, b3, G3)):
        dmax, dmin = a + b, a - b
        sp[j] = {"G_Nm": round(G, 3), "k_total_N_per_mm": round(k, 3), "k_each_N_per_mm": round(k / n, 3),
                 "force_max_total_N": round(k * dmax, 1), "force_max_each_N": round(k * dmax / n, 1),
                 "stroke_mm": round(dmax - dmin, 1),
                 "spec_each": (f"extension spring, rate {k / n:.2f} N/mm (+/-10%), max working force >= "
                               f"{1.3 * k * dmax / n:.0f} N, working stroke >= {dmax - dmin + 10:.0f} mm, "
                               f"initial tension < {0.5 * k * dmin / n:.0f} N; cord length set so tension "
                               f"= k x (distance A-B)")}
    return (k2, k3), sp


SPRINGS, SPRING_SPEC = size_springs()
R["springs"] = SPRING_SPEC


# ============================================================================ 4. torque maps / payload
def torque_survey(payload_g, springs):
    keys = ARM_JOINTS + (("w1", "w2") if ABN else ())
    worst, arg = {j: 0.0 for j in keys}, {}
    for th2 in np.arange(-30, 101, 10):
        for th3 in np.arange(-95, 61, 10):
            for pt in WRIST_PITCHES:
                for roll in ((0.0, 90.0) if ABN else (0.0,)):
                    q = wq(th2, th3, pt, 0.0, roll)
                    if not valid(q):
                        continue
                    t = joint_torques(q, payload_g, springs)
                    for j in keys:
                        if abs(t[j]) > abs(worst[j]):
                            worst[j], arg[j] = round(t[j], 3), (float(th2), float(th3), pt, roll)
    return worst, arg


R["torque_worst_no_springs_no_payload_Nm"], _ = torque_survey(0.0, None)
R["torque_worst_springs_no_payload_Nm"], _ = torque_survey(0.0, SPRINGS)
R["torque_worst_springs_rated_payload_Nm"], R["torque_worst_pose"] = torque_survey(P.payload.rated_kg * 1000, SPRINGS)


def payload_vs_reach():
    """Max payload (kg) with the tool horizontal at about shoulder height, per joint (continuous
    rating) and overall, vs. horizontal reach from the J1 axis."""
    rows = []
    O_z = L.T_ORIGIN_B + A.shoulder_h
    for rel in np.arange(P.joints.j3_rel.soft[0], 141, 4):
        best = None
        for th2 in np.arange(-30, 101, 2):
            th3 = th2 - rel
            if abs(th3) > PITCH_LIM:
                continue
            q = wq(th2, th3, -th3)
            if not valid(q):
                continue
            p = tcp(q)
            if best is None or abs(p.Z - O_z) < abs(best[1].Z - O_z):
                best = (q, p)
        if best is None:
            continue
        q, p = best
        t0 = joint_torques(q, 0.0, SPRINGS)
        t1 = joint_torques(q, 1000.0, SPRINGS)
        cap = {}
        for j in t0:
            dt = abs(t1[j] - t0[j])
            room = TAU_CONT[j] - abs(t0[j])
            cap[j] = max(0.0, room / dt) if dt > 1e-6 else 99.0
        rows.append({"reach_mm": round(math.hypot(p.X, p.Y), 1), "tcp_z_mm": round(p.Z, 1),
                     "q": {k: v for k, v in q.items()}, **{f"cap_{j}_kg": round(v, 3) for j, v in cap.items()},
                     "cap_kg": round(min(cap.values()), 3)})
    rows.sort(key=lambda r: r["reach_mm"])
    return rows


PAYLOAD = payload_vs_reach()
R["payload_vs_reach"] = PAYLOAD
R["max_horizontal_reach_tcp_mm"] = max(r["reach_mm"] for r in PAYLOAD)
R["payload_at_max_reach_kg"] = PAYLOAD[-1]["cap_kg"]
R["payload_rated_check"] = {"rated_kg": P.payload.rated_kg,
                            "reach_where_rated_payload_holds_mm": max((r["reach_mm"] for r in PAYLOAD
                                                                       if r["cap_kg"] >= P.payload.rated_kg), default=0)}

fig, ax = plt.subplots(figsize=(7.5, 4.6))
xs = [r["reach_mm"] for r in PAYLOAD]
cols = {"j2": "#1f77b4", "j3": "#ff7f0e", "j4": "#2ca02c", "w1": "#2ca02c", "w2": "#9467bd"}
for j in [k[4:-3] for k in PAYLOAD[0] if k.startswith("cap_") and k != "cap_kg"]:
    lab = {"w1": "wrist Theta servo 1", "w2": "wrist Theta servo 2"}.get(j, j.upper())
    ax.plot(xs, [min(r[f"cap_{j}_kg"], 3) for r in PAYLOAD], "--", color=cols.get(j, "k"), lw=1,
            label=f"{lab} continuous limit")
ax.plot(xs, [min(r["cap_kg"], 3) for r in PAYLOAD], "k-", lw=2.2, label="arm (min of joints)")
ax.axhline(P.payload.rated_kg, color="r", lw=0.8, ls=":", label=f"rated {P.payload.rated_kg} kg")
ax.set_xlabel("horizontal TCP distance from J1 axis (mm), tool horizontal at shoulder height")
ax.set_ylabel("payload (kg)")
ax.set_ylim(0, 2.0)
ax.grid(alpha=0.3)
ax.legend(fontsize=7)
ax.set_title(f"{R['variant']}: payload vs reach (springs {P.springs.balance_fraction * 100:.0f}% balance, "
             f"servo continuous = {S.continuous_fraction * 100:.0f}% stall)", fontsize=9)
fig.tight_layout()
fig.savefig(OUT / "payload_vs_reach.png", dpi=150)
plt.close(fig)

fig, axs = plt.subplots(1, 2, figsize=(10, 3.8))
th = np.arange(-30, 101, 5)
for spr, ls, lab in ((None, "--", "no springs"), (SPRINGS, "-", "with springs")):
    xs2, t2 = [], []
    for x in th:
        qq = wq(x, min(x - 45, 60))
        if valid(qq):
            xs2.append(x)
            t2.append(joint_torques(qq, 0, spr)["j2"])
    axs[0].plot(xs2, t2, ls, label=lab)
    xs3, t3 = [], []
    for x in np.arange(-95, 61, 5):
        qq = wq(min(x + 60, 100), x)
        if valid(qq):
            xs3.append(x)
            t3.append(joint_torques(qq, 0, spr)["j3"])
    axs[1].plot(xs3, t3, ls, label=lab)
for a_, j, xl in ((axs[0], "j2", "theta2 (deg)"), (axs[1], "j3", "theta3 (deg)")):
    a_.axhline(TAU_CONT[j], color="r", lw=0.7, ls=":")
    a_.axhline(-TAU_CONT[j], color="r", lw=0.7, ls=":", label="continuous rating")
    a_.set_xlabel(xl)
    a_.set_ylabel(f"{j.upper()} static torque (N m), no payload")
    a_.grid(alpha=0.3)
    a_.legend(fontsize=8)
fig.tight_layout()
fig.savefig(OUT / "gravity_torque_springs.png", dpi=150)
plt.close(fig)

# ============================================================================ 4b. ABENICS wrist map
if ABN:
    rows = []
    for pitch in np.arange(-33, 33.1, 3):
        for yaw in np.arange(-33, 33.1, 3):
            Rm = AK.tool_rotation_pyr(pitch, yaw, 0)
            ok_h, _ = AK.wrist_check(Rm, hard=True)
            ok_s, info = AK.wrist_check(Rm)
            rows.append((pitch, yaw, ok_s, ok_h, info["tilt"], max(info["toward_u1"], info["toward_u2"])))
    arr = np.array([(r[0], r[1], r[2], r[3]) for r in rows], float)
    fig, ax = plt.subplots(figsize=(5.6, 5.2))
    ax.scatter(arr[arr[:, 3] > 0, 1], arr[arr[:, 3] > 0, 0], s=18, c="#9ecae1", label="hard limit (mechanical)")
    ax.scatter(arr[arr[:, 2] > 0, 1], arr[arr[:, 2] > 0, 0], s=6, c="#08519c", label="soft limit (firmware)")
    ax.set_xlabel("tool yaw vs. forearm (deg, + toward module 1 side)")
    ax.set_ylabel("tool pitch vs. forearm (deg, + up)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="lower left")
    ax.set_title("ABENICS tool-direction workspace (any roll within +/-90 deg)\n"
                 "up-left / up-right diagonals limited by the module stacks", fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "abenics_direction_workspace.png", dpi=150)
    plt.close(fig)
    # load split: Theta-servo torque for the claw + rated payload, forearm horizontal, over pitch/yaw/roll
    worst = (0.0, None)
    for pitch in (-30, -15, 0, 15, 30):
        for yaw in (-30, -15, 0, 15, 30):
            for roll in (-90, -45, 0, 45, 90):
                q = wq(45, 0, pitch, yaw, roll)
                if not valid(q):
                    continue
                w = wrist_load(q, P.payload.rated_kg * 1000)
                m = max(abs(w["w1"]), abs(w["w2"]))
                if m > worst[0]:
                    worst = (m, (pitch, yaw, roll), w)
    R["abenics"] = {
        "direction_limits_deg": {"tilt_soft_hard": AK.LIMITS["tilt"], "toward_module_soft_hard": AK.LIMITS["toward_module"],
                                 "theta_soft_hard": AK.LIMITS["theta"], "q_window": AK.LIMITS["q"]},
        "theta_servo_worst_Nm_rated_payload_forearm_horizontal": round(worst[0], 3),
        "theta_servo_worst_pose_pyr": worst[1],
        "theta_servo_continuous_Nm": round(TAU_CONT["w1"], 3),
        "margin_continuous": round(TAU_CONT["w1"] / worst[0], 2),
        "sphere_torque_per_module_Nm_cont_peak": [round(THETA_RATIO * TAU_CONT["w1"], 2), round(THETA_RATIO * TAU_PEAK["w1"], 2)],
        "roll_torque_both_modules_Nm_cont_peak": [round(2 * THETA_RATIO * TAU_CONT["w1"], 2),
                                                  round(2 * THETA_RATIO * TAU_PEAK["w1"], 2)],
        "tool_speed_deg_s": {"tilt_or_roll_from_theta": round(S.no_load_rpm * 6 * AB.theta_gear.servo_teeth /
                                                               AB.theta_gear.mp_teeth / 2, 0),
                             "carrier_q": S.no_load_rpm * 6},
        "q_servo_static_load": "none from the tool (mesh forces act through the module axis); carrier "
                               "gravity only: < 0.03 N m [C]"}

# ============================================================================ 5. belts
belt = {}
for j, tdrv in (("j1", P.pulleys.j1), ("j2", P.pulleys.j2), ("j3", P.pulleys.j3)) + \
        ((("j4", P.pulleys.j4),) if not ABN else ()):
    r_small = lib.pd(P.dm.teeth) / 2 / 1000
    f_peak = TAU_STALL * S.peak_fraction / r_small
    belt[j] = {"effective_tension_at_peak_N": round(f_peak, 1), "at_servo_stall_N": round(TAU_STALL / r_small, 1),
               "allowable_N": P.belt.allowable_N, "margin_peak": round(P.belt.allowable_N / f_peak, 2),
               "recommended_static_pretension_N": round(0.5 * f_peak + 10, 0)}
R["belts"] = belt

# ============================================================================ 6. electrical
kt = S.kt_Nm_per_A
N_SERVO = 8 if ABN else 6
cur = {"each_servo": {"continuous_A": round(S.no_load_current_A + TAU_STALL * S.continuous_fraction / kt, 2),
                      "peak_A": round(S.no_load_current_A + TAU_STALL * S.peak_fraction / kt, 2),
                      "stall_A": S.stall_current_A}}
R["servo_current"] = cur
P_servo_typ = 12.0 * (N_SERVO * S.no_load_current_A + 2 * 0.5)       # idle + two joints moving [E]
P_servo_peak = 12.0 * N_SERVO * (S.no_load_current_A + TAU_STALL * S.peak_fraction / kt)
P_jetson = {"typical_W": 10.0, "max_W": 25.0}                         # [V] Orin Nano 8GB 7-25 W modes
P_misc = 2.0
batt_Wh = 18.0 * 4.0
R["power"] = {"servos": N_SERVO, "servo_bus_typical_W": round(P_servo_typ, 1), "servo_bus_worst_W": round(P_servo_peak, 1),
              "servo_bus_worst_A": round(P_servo_peak / 12, 1), "jetson_W": P_jetson, "misc_W": P_misc,
              "system_typical_W": round(P_servo_typ + P_jetson["typical_W"] + P_misc, 1),
              "system_worst_W": round(P_servo_peak + P_jetson["max_W"] + P_misc, 1), "battery_example_Wh": batt_Wh,
              "runtime_typical_h": round(0.85 * batt_Wh / (P_servo_typ + P_jetson["typical_W"] + P_misc), 1),
              "fuse_servo_bus_A": 25, "buck_rating_A": 20,
              "note": f"Worst case = all {N_SERVO} servos at the 50 % torque limit at once (not a normal state); "
                      "the 25 A fuse and the 20 A converter's own current limit cover it."}

# ============================================================================ 7. structure
MAT = {"PETG": (40.0, 25.0), "PETG-CF": (35.0, 29.0), "PA-CF": (60.0, 30.0)}   # (XY, Z) tensile MPa [E: TDS]
SF_PRINT = 4.0          # [D] static safety factor for printed structure (creep, voids, impact)
SF_GEAR = 2.0           # [D] printed gear teeth (Lewis, on the TDS XY strength)
DYN = 3.0               # [E] dynamic factor for chair-mounted loads (kerbs, door thresholds)
m_arm = R["mass_g"]["arm_total_on_dock"] / 1000
m_pay = P.payload.peak_kg
q_out = wq(10.0, -40.0, 30.0 if ABN else 40.0)
f = all_frames(dict(asm.HOME, **q_out))
mx = sum(m * (f[fk] * Pos(*cl)).position.X for _, fk, cl, m in MODEL) / sum(m for *_, m in MODEL)
reach_out = math.hypot(tcp(q_out).X, tcp(q_out).Y)
M_dock = G0 * (m_arm * mx + m_pay * reach_out) / 1000
F_v = G0 * (m_arm + m_pay)
structure = []


def chk(name, stress, allow, note):
    structure.append({"item": name, "stress_MPa": round(stress, 1), "allowable_MPa": round(allow, 1),
                      "margin": round(allow / stress, 2) if stress > 0 else 99, "note": note})


t = P.base.dock_plate_t
F_screw = DYN * M_dock * 1000 / (2 * 48.1)
chk("dock plate bending at thumb screw (base housing, PA-CF)", 6 * (F_screw * 20) / (40 * t * t), MAT["PA-CF"][0] / SF_PRINT,
    f"M_dock = {M_dock:.2f} N m static x {DYN} dynamic; screw force {F_screw:.0f} N")
R["dock"] = {"M_static_Nm": round(M_dock, 2), "M_design_Nm": round(DYN * M_dock, 2), "screw_force_N": round(F_screw),
             "M6_insert_pullout_N_est": 1200, "insert_margin": round(1200 / F_screw, 2),
             "arm_cg_offset_mm": round(mx, 1), "reach_mm": round(reach_out, 1)}
from wra.parts import mount as mt  # noqa: E402
arm_x = mt.shelf_center_x()
Msp = DYN * (F_v * arm_x + M_dock * 1000)
chk("post bracket spine/gusset root (PA-CF, layers along load)", 6 * Msp / (mt.SPINE_W * mt.SPINE_T ** 2) / 2.0,
    MAT["PA-CF"][0] / SF_PRINT, f"design moment {Msp / 1000:.1f} N m (3 g); aluminium option: output/dxf")
r_strut, T_post = 40.0, 24.0
F_strut = T_post * 1000 / (r_strut * 0.55)
R["torque_strut"] = {"T_about_post_Nm": T_post, "lever_mm": r_strut, "strut_force_N": round(F_strut),
                     "note": "door-pull scenario 40 N x 0.6 m [E]; SI5 rod ends are rated several kN [E: check]"}
chk("strut clevis bolt bearing (PA-CF)", F_strut / (2 * 5.0 * 10.0), MAT["PA-CF"][1] / SF_PRINT * 2.5,
    "bearing allowable ~2.5x tensile for confined printed material [E]")
belt_pull = 2 * (belt["j2"]["effective_tension_at_peak_N"] + belt["j2"]["recommended_static_pretension_N"])
beyond = {"U", "F", "W", "R", "S", "G", "K", "RP", "RQ"}
M_stub = belt_pull * 52.0 + DYN * G0 * (group_mass(beyond) / 1000 + m_pay) * 30.0
chk("J2 stub axle 15 mm h6 steel (bending at cheek)", 32 * M_stub / (math.pi * 15.0 ** 3), 400 / 2.0, "steel yield/2 [E]")
F_el = DYN * G0 * (group_mass({"F", "W", "R", "S", "G"}) / 1000 + m_pay)
chk("elbow pin 10 mm h6 (bending)", 32 * F_el * 25.0 / (math.pi * 10.0 ** 3), 400 / 2.0, f"forearm side + payload x {DYN}")
I1 = math.pi / 64 * (12 ** 4 - 10 ** 4)
A1 = math.pi / 4 * (12 ** 2 - 10 ** 2)
I_pair = 2 * (I1 + A1 * 15 ** 2)
M_ua = G0 * ((group_mass({"F", "W", "R", "S", "G"}) / 1000 + m_pay) * A.L2 + group_mass({"U"}) / 1000 * A.L2 / 2)
chk("upper-arm tube pair (6063-T5) at the hub, 3 g, springs ignored", M_ua * DYN * (15 + 6) / I_pair, 145 / 2.0,
    "conservative: full moment")
M_fa = G0 * ((group_mass({"W", "R", "S", "G"}) / 1000 + m_pay) * (A.L3 + 150) +
             group_mass({"F"}) / 1000 * A.L3 * 0.8)
chk("forearm tube pair (6063-T5) at the forearm root, 3 g", M_fa * DYN * (15 + 6) / I_pair, 145 / 2.0,
    "wrist + tool + payload at the wrist; conservative lever")
F_sock = M_ua * DYN / (2 * 15.0) / 2
chk("tube socket contact pressure (UA hub, PETG/PA-CF)", F_sock * 6 / (12 * A.socket_depth), MAT["PETG"][1] / SF_PRINT * 2,
    "plus M3 cross-bolt and structural epoxy - see assembly manual")
Y = {14: 0.277, 15: 0.289, 16: 0.296, 17: 0.303, 18: 0.309, 20: 0.322, 24: 0.337, 34: 0.371}   # Lewis, 20 deg full depth [V]
# claw pinion: two racks share the tangential force
for tau_lim, lab in ((P.claw.torque_cont_Nm, "continuous 25 %"), (P.claw.torque_knob_Nm, "knob mode 50 %, <= 10 s")):
    Ft = tau_lim / (cw.RP / 1000) / 2
    chk(f"claw pinion 14T m{P.claw.pinion_module} x {P.claw.pinion_width:.0f} mm, Lewis ({lab})",
        Ft / (P.claw.pinion_width * P.claw.pinion_module * Y[14]), MAT["PA-CF"][0] / SF_GEAR,
        f"tangential {Ft:.0f} N per rack mesh; print in PA-CF, 0.12 mm layers")
if ABN:
    TG_ = AB.theta_gear
    TAU_KNOB_THETA = TAU_STALL * 0.30                                  # firmware cap for the knob turn [D]
    for tau_s, lab in ((TAU_CONT["w1"], "continuous 25 %"), (TAU_KNOB_THETA, "knob turn 30 %"), (TAU_PEAK["w1"], "peak 50 % (not used)")):
        t_mp = tau_s * TG_.mp_teeth / TG_.servo_teeth
        Ft = t_mp / (ab.RM / 1000)                                     # at the monopole pitch radius
        b_eff = 6.0                                                    # [E] effective face of the spherical contact
        chk(f"ABENICS monopole 17T m{AB.module} teeth (eff. face {b_eff:.0f} mm), Lewis ({lab})",
            Ft / (b_eff * AB.module * Y[17]), MAT["PA-CF"][0] / SF_GEAR,
            f"F_t = {Ft:.0f} N; sphere torque {2 * t_mp:.2f} N m per module")
        Ft2 = tau_s / (ab.R_TG_SERVO / 1000)
        chk(f"Theta spur pair {TG_.servo_teeth}T/{TG_.mp_teeth}T m{TG_.module} x {TG_.width} mm, Lewis {TG_.mp_teeth}T ({lab})",
            Ft2 / (TG_.width * TG_.module * Y[TG_.mp_teeth]), MAT["PA-CF"][0] / SF_GEAR, f"F_t = {Ft2:.0f} N")
    R["abenics_knob_turn_theta_limit"] = {"fraction_of_stall": 0.30,
                                          "roll_torque_Nm": round(2 * THETA_RATIO * TAU_KNOB_THETA, 2)}
    # centre-ball post: tool weight + payload (x DYN) + both modules' separating forces
    m_tool = group_mass({"S", "G"}) / 1000
    Ft_sep = THETA_RATIO * TAU_CONT["w1"] / (ab.RHO / 1000) * math.tan(math.radians(20.0))
    F_post = DYN * G0 * (m_tool + m_pay) + math.sqrt(2) * Ft_sep
    lever = -(ab.POST_X[0] + 22.0)                                     # clamp boss ends at x = -40
    I_t = math.pi / 64 * (8 ** 4 - 6 ** 4)
    sig = F_post * lever * 4.0 / I_t
    chk("centre-ball post 8 x 1 mm tube, bending at the clamp (stainless 304)", sig, 210 / 1.5,
        f"F = {F_post:.0f} N (3 g tool + payload, + mesh separating forces), lever {lever:.0f} mm; "
        f"6063-T5 aluminium would give margin {145 / 1.5 / sig:.2f}")
    p_ball = F_post / (math.pi * ab.BALL_R ** 2 * 0.3)
    chk("centre ball / socket contact pressure (PA-CF on PA-CF, greased)", p_ball, 8.0,
        "allowable ~8 MPa for slow sliding of filled nylon [E]; keep greased")
for j, v in belt.items():
    chk(f"HTD-3M-15 belt {j.upper()} effective tension at peak torque", v["effective_tension_at_peak_N"] / 15,
        P.belt.allowable_N / 15, "N per mm of belt width (Gates working tension) [V]")
R["structure"] = structure

# ============================================================================ 8. accuracy budget
enc = 360.0 / S.counts_per_rev
bl_servo = 0.5                                    # deg, ST3215 output backlash [E - measure!]
reach = R["max_horizontal_reach_tcp_mm"]
lev_w = (ab.FL1 + TCP_X) if ABN else (A.wrist_to_flange + TCP_X)
err = {}
for j, lev in (("j1", reach), ("j2", reach), ("j3", A.L3 + lev_w)):
    r_ = RATIO[j]
    err[j] = {"encoder_mm": round(math.radians(enc / r_) * lev, 2), "servo_backlash_mm": round(math.radians(bl_servo / r_) * lev, 2)}
EA = 1.8e4 * 15                                   # belt [E] N per unit strain
for j, span, teeth in (("j1", L.C1, P.pulleys.j1), ("j2", L.C2, P.pulleys.j2), ("j3", L.C3, P.pulleys.j3)):
    k_rot = 2 * EA / span * (lib.pd(teeth) / 2) ** 2
    lev = reach if j in ("j1", "j2") else A.L3 + lev_w
    err[j]["belt_stretch_mm_at_rated_payload"] = round(G0 * P.payload.rated_kg * lev / k_rot * lev, 2)
if ABN:
    # tool orientation: Theta path = servo backlash/encoder + 24/18 mesh + CS mesh; q path: dog clearance
    gear_bl = 0.15                                                # mm printed backlash per mesh [D]
    th_deg = (bl_servo * AB.theta_gear.servo_teeth / AB.theta_gear.mp_teeth / 2 +
              math.degrees(gear_bl / (ab.R_TG_MP)) / 2 + math.degrees(gear_bl / ab.RHO))
    q_deg = bl_servo + math.degrees(0.05 / ab.DOG_R)
    err["wrist"] = {"theta_path_deg": round(th_deg, 2), "q_path_deg": round(q_deg, 2),
                    "tcp_mm": round(math.radians(math.hypot(th_deg, q_deg)) * lev_w, 2),
                    "encoder_mm": round(math.radians(enc * AB.theta_gear.servo_teeth / AB.theta_gear.mp_teeth / 2) * lev_w, 2)}
else:
    err["j4"] = {"encoder_mm": round(math.radians(enc / RATIO["j4"]) * lev_w, 2),
                 "servo_backlash_mm": round(math.radians(bl_servo / RATIO["j4"]) * lev_w, 2)}
R["tcp_error_budget"] = err
R["tcp_error_rss_mm"] = round(math.sqrt(sum(sum(x ** 2 for k, x in v.items() if k.endswith("_mm")) for v in err.values())), 1)

# ============================================================================ 9. workspace
pts_side = []
for th2 in np.arange(-30, 101, 4):
    for th3 in np.arange(-95, 61, 4):
        for pt in ((-30.0, 0.0, 30.0) if ABN else (-60.0, 0.0, 60.0)):
            q = wq(th2, th3, pt)
            if valid(q):
                p = tcp(q)
                pts_side.append((p.X, p.Z))
pts_side = np.array(pts_side)
z_floor = -P.chair.mount_z
fig, ax = plt.subplots(figsize=(7.5, 6))
ax.scatter(pts_side[:, 0], pts_side[:, 1], s=1.2, alpha=0.25, color="#1f77b4", label="TCP reachable (J1 = 0 plane)")
ax.axhspan(34 * inch + z_floor, 48 * inch + z_floor, color="#2ca02c", alpha=0.12, label="ADA door hardware 34-48 in")
ax.axhspan(28 * inch + z_floor, 34 * inch + z_floor, color="#ff7f0e", alpha=0.10, label="table tops 28-34 in")
ax.axhline(z_floor, color="k", lw=1.5)
ax.text(10, z_floor + 8, "floor (provisional mount height %.0f mm)" % P.chair.mount_z, fontsize=8)
ax.plot([0], [L.T_ORIGIN_B + A.shoulder_h], "r+", ms=12)
ax.set_aspect("equal")
ax.set_xlabel("x_B forward of J1 axis (mm)")
ax.set_ylabel("z_B above dock (mm)")
ax.grid(alpha=0.3)
ax.legend(fontsize=7, loc="lower right")
ax.set_title(f"{R['variant']}: reachable TCP positions in the arm plane (soft + coupled limits)", fontsize=9)
fig.tight_layout()
fig.savefig(OUT / "workspace_side.png", dpi=150)
plt.close(fig)
R["workspace"] = {"tcp_x_range_mm": [round(float(pts_side[:, 0].min())), round(float(pts_side[:, 0].max()))],
                  "tcp_z_range_B_mm": [round(float(pts_side[:, 1].min())), round(float(pts_side[:, 1].max()))],
                  "floor_z_B_mm": z_floor, "reaches_floor": bool(pts_side[:, 1].min() <= z_floor + 40),
                  "door_band_B_mm": [round(34 * inch + z_floor), round(48 * inch + z_floor)]}

from wra import chair as ch  # noqa: E402
fig, ax = plt.subplots(figsize=(7, 7))
for side, col in (("right", "#1f77b4"), ("left", "#d62728")):
    Bl = ch.frames(side)["B"]
    pts = []
    for j1 in np.arange(-105, 106, 7.5):
        for th2 in np.arange(-30, 101, 8):
            for th3 in np.arange(-95, 61, 8):
                q = wq(th2, th3, max(-30.0, min(30.0, -th3)) if ABN else 0.0, j1=float(j1))
                if not valid(q):
                    continue
                tq = tcp(q)
                p = (Bl * Pos(tq.X, tq.Y, tq.Z)).position
                if 34 * inch <= p.Z <= 48 * inch:
                    pts.append((p.X, p.Y))
    pts = np.array(pts)
    ax.scatter(pts[:, 0], pts[:, 1], s=1.0, alpha=0.2, color=col, label=f"{side.upper()} install: TCP at 34-48 in")
for nm, shp, colr in mt.reference_chair():
    bb = shp.bounding_box()
    ax.add_patch(plt.Rectangle((bb.min.X, bb.min.Y), bb.size.X, bb.size.Y, fill=False, lw=0.5, color="#555555"))
for nm, shp in mt.user_envelope():
    bb = shp.bounding_box()
    ax.add_patch(plt.Rectangle((bb.min.X, bb.min.Y), bb.size.X, bb.size.Y, fill=True, alpha=0.08, color="#d62728"))
ax.set_aspect("equal")
ax.set_xlabel("x_C forward (mm)")
ax.set_ylabel("y_C left (mm)")
ax.grid(alpha=0.3)
ax.legend(fontsize=7, loc="upper left")
ax.set_title("Top view: door-hardware-height reach, RIGHT vs LEFT install (mirror images)", fontsize=9)
fig.tight_layout()
fig.savefig(OUT / "workspace_top_LR.png", dpi=150)
plt.close(fig)

# ============================================================================ 10. door tasks
doors = {}
V_FACTOR = 1.0 / math.cos(math.radians(40.0))      # 100 deg V: N_total = F / cos(40 deg) per jaw (2 flanks)
# wrist roll available for the turn: continuous rating while just holding; in knob mode the Theta
# servos are capped at 30 % of stall (Theta-pair tooth limit, section 6) on WRA-1A, J5 peak on WRA-1B
roll_cap = ((2 * THETA_RATIO * TAU_CONT["w1"], 2 * THETA_RATIO * TAU_STALL * 0.30) if ABN
            else (TAU_CONT["j5"], TAU_PEAK["j5"]))
for (lab, tau6), cap in zip((("continuous grip", P.claw.torque_cont_Nm), ("knob mode (<= 10 s)", P.claw.torque_knob_Nm)), roll_cap):
    F_jaw = cw.jaw_force(tau6)
    for d_knob in (50.0, 56.0, 64.0):
        r_k = d_knob / 2 / 1000
        T_lo, T_hi = (2 * F_jaw * V_FACTOR * mu * r_k for mu in (0.5, 0.8))     # TPU 0.5 .. silicone 0.8 on brass [E]
        doors[f"knob_{d_knob:.0f}mm_{lab}"] = {
            "jaw_force_N": round(F_jaw, 1), "slip_torque_Nm_mu0.5_0.8": [round(T_lo, 2), round(T_hi, 2)],
            "wrist_roll_capacity_Nm": round(cap, 2),
            "knob_torque_capacity_Nm_mu0.5_0.8": [round(min(T_lo, cap), 2), round(min(T_hi, cap), 2)],
            "limited_by": ("wrist roll" if T_lo >= cap else
                           ("grip slip" if T_hi <= cap else "grip slip (TPU) / wrist roll (silicone)"))}
doors["knob_torque_demand"] = {"typical_residential_Nm": [0.3, 1.5], "BHMA_A156.2_max_Nm": 3.16,
                               "source": "BHMA A156.2 latch retraction torque limit; typical values [E] (measure YOUR door)"}


def ext_force_capacity(q, direction):
    """Max static TCP force (N) along `direction` (frame B, unit) before any actuator exceeds its
    peak rating, including gravity/payload bias (springs on)."""
    t0 = joint_torques(q, 0.0, SPRINGS)
    d = np.asarray(direction, float)
    d /= np.linalg.norm(d)
    best = 1e9
    # arm joints by virtual work: tau_j = J^T F -> numerical: displacement of TCP per joint radian
    for j in ARM_JOINTS:
        qa, qb = dict(q), dict(q)
        qa[j] += 0.05
        qb[j] -= 0.05
        dp = (tcp(qa) - tcp(qb))
        g = np.dot([dp.X, dp.Y, dp.Z], d) / math.radians(0.1) / 1000      # m per rad
        if abs(g) > 1e-6:
            best = min(best, (TAU_PEAK[j] - abs(t0[j])) / abs(g))
    if ABN:
        w1 = wrist_load(q, 0.0, f_tcp=d * 1.0)                            # per newton
        w0 = wrist_load(q, 0.0)
        for k in ("w1", "w2"):
            per = abs(w1[k] - w0[k])
            if per > 1e-9:
                best = min(best, (TAU_PEAK[k] - abs(w0[k])) / per)
    return best


lever_rows = []
for xh in np.arange(200, 560, 25):
    best = None
    for th2 in np.arange(-30, 101, 3):
        for rel in np.arange(P.joints.j3_rel.soft[0], 141, 4):
            th3 = th2 - rel
            if abs(th3) > PITCH_LIM:
                continue
            q = wq(th2, th3, -th3)
            if not valid(q):
                continue
            p = tcp(q)
            if abs(p.X - xh) > 12 or abs(p.Z - (38 * inch + z_floor)) > 25:
                continue
            down = ext_force_capacity(q, (0, 0, -1))
            pull = ext_force_capacity(q, (-1, 0, 0))
            if best is None or down > best[0]:
                best = (down, pull, q)
    if best:
        lever_rows.append({"x_mm": float(xh), "push_down_N": round(best[0], 1), "pull_N": round(best[1], 1),
                           "sideways_N_J1": round(TAU_PEAK["j1"] / (xh / 1000), 1), "q": best[2]})
doors["lever_and_pull_vs_distance"] = lever_rows
doors["ADA_reference"] = {"interior_door_opening_force_N": 22.2, "operable_parts_max_N": 22.2,
                          "source": "ADA 2010 404.2.9 / 309.4 (see docs/01_research_findings.md)"}
R["door_tasks"] = doors
if lever_rows:
    fig, ax = plt.subplots(figsize=(7.5, 4.4))
    xs = [r["x_mm"] for r in lever_rows]
    ax.plot(xs, [r["push_down_N"] for r in lever_rows], "-o", ms=3, label="push DOWN on a lever")
    ax.plot(xs, [r["pull_N"] for r in lever_rows], "-s", ms=3, label="pull toward the chair")
    ax.plot(xs, [r["sideways_N_J1"] for r in lever_rows], "-^", ms=3, label="sideways (J1 peak)")
    ax.axhline(22.2, color="r", ls=":", label="ADA 5 lbf (22.2 N)")
    ax.set_xlabel("horizontal distance J1 axis -> handle (mm), handle at 38 in")
    ax.set_ylabel("force at TCP (N)")
    ax.set_ylim(0, 100)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    ax.set_title(f"{R['variant']}: door-hardware force capability (servo peak = 50 % stall, springs on)", fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT / "door_force_capability.png", dpi=150)
    plt.close(fig)

# ============================================================================ 11. claw
R["claw"] = {"opening_mm": [0.0, P.claw.open_max], "jaw_force_N_cont_knob": [round(cw.jaw_force(P.claw.torque_cont_Nm), 1),
                                                                              round(cw.jaw_force(P.claw.torque_knob_Nm), 1)],
             "pinion_r_mm": cw.RP, "jaw_speed_mm_s": round(cw.RP * S.no_load_rpm * 2 * math.pi / 60, 1),
             "close_time_full_s": round(P.claw.open_max / 2 / (cw.RP * S.no_load_rpm * 2 * math.pi / 60), 2),
             "servo_deg_full_stroke": round(cw.q_to_servo_deg(P.claw.open_max / 2), 1),
             "v_groove_deg": 100, "v_normal_force_factor": round(V_FACTOR, 3),
             "tcp_from_flange_mm": TCP_X, "pad_length_mm": round(cw.PAD_X[1] - cw.PAD_X[0], 1),
             "lip_mm": P.claw.lip_d,
             "note": "parallel jaws: grip force independent of object size; V pads centre round knobs on the roll axis"}

(OUT / "results.json").write_text(json.dumps(R, indent=1, default=str))
(OUT / "springs.json").write_text(json.dumps(R["springs"], indent=1))
print(json.dumps({k: R[k] for k in ("variant", "mass_g", "springs")}, indent=1))
print("max reach", R["max_horizontal_reach_tcp_mm"], "payload@max", R["payload_at_max_reach_kg"],
      "tcp err rss", R["tcp_error_rss_mm"])
if ABN:
    print("abenics", json.dumps(R["abenics"], default=str))
for s_ in structure:
    print(f"  {s_['margin']:6.2f}  {s_['item']}")


# ============================================================================ 12. report (docs/05)
def write_report():
    D = ROOT / "docs"
    D.mkdir(exist_ok=True)
    m = R["mass_g"]
    L = ["# 05 - Engineering calculations (GENERATED by `python calcs/arm_calcs.py`; do not edit)", "",
         f"Variant: **{R['variant']}**.  Generated {R['generated']}.  Inputs: `cad/wra/params.yaml`, CAD centroids, "
         "printed masses from `output/reports/parts.json`.  Tags: [V] verified, [C] computed, [E] estimate, [D] design rule.",
         "Figures: `calcs/output/*.png`.  Raw numbers: `calcs/output/results.json`.", "",
         "## 1. Mass", "", "| group | mass (g) |", "|---|---|"]
    L += [f"| {k} | {v} |" for k, v in m.items()]
    L += ["", f"Arm on the dock: **{m['arm_total_on_dock'] / 1000:.2f} kg**; moving beyond the shoulder (J2): "
          f"**{m['moving_beyond_J2'] / 1000:.2f} kg** [C].", "",
          "## 2. Joint capacities (servo derating 25 % continuous / 50 % peak of 2.94 N m stall [V])", "",
          "| joint | ratio | continuous (N m) | peak (N m) |", "|---|---|---|---|"]
    for j, v in R["joint_capacity_Nm"].items():
        if isinstance(v, dict):
            L.append(f"| {j} | {v['ratio']} | {v['continuous']} | {v['peak']} |")
    if "note" in R["joint_capacity_Nm"]:
        L += ["", R["joint_capacity_Nm"]["note"]]
    L += ["", "## 3. Gravity balance springs (zero-free-length emulation)", "",
          "| joint | gravity coefficient G (N m) | k per spring (N/mm) | max force per spring (N) | spec |", "|---|---|---|---|---|"]
    for j, v in R["springs"].items():
        L.append(f"| {j} | {v['G_Nm']} | {v['k_each_N_per_mm']} | {v['force_max_each_N']} | {v['spec_each']} |")
    L += ["", "Worst static joint torques over the workspace (N m):", "", "| case | " + " | ".join(R["torque_worst_springs_rated_payload_Nm"]) + " |",
          "|---|" + "---|" * len(R["torque_worst_springs_rated_payload_Nm"])]
    for lab, key in (("no springs, no payload", "torque_worst_no_springs_no_payload_Nm"),
                     ("springs, no payload", "torque_worst_springs_no_payload_Nm"),
                     ("springs + rated payload", "torque_worst_springs_rated_payload_Nm")):
        L.append(f"| {lab} | " + " | ".join(str(R[key][j]) for j in R[key]) + " |")
    L += ["", "![gravity torque](../calcs/output/gravity_torque_springs.png)", "",
          "## 4. Payload vs reach", "",
          f"Max horizontal TCP reach **{R['max_horizontal_reach_tcp_mm']:.0f} mm** from the J1 axis; payload there "
          f"**{R['payload_at_max_reach_kg']:.2f} kg** (continuous ratings).  Rated payload {P.payload.rated_kg} kg holds out to "
          f"{R['payload_rated_check']['reach_where_rated_payload_holds_mm']:.0f} mm.", "",
          "![payload](../calcs/output/payload_vs_reach.png)", ""]
    if ABN:
        a = R["abenics"]
        L += ["## 5. ABENICS wrist", "",
              f"* Limits (soft/hard): tilt {a['direction_limits_deg']['tilt_soft_hard']} deg, toward a module axis "
              f"{a['direction_limits_deg']['toward_module_soft_hard']} deg, |Theta| {a['direction_limits_deg']['theta_soft_hard']} deg, "
              f"|q| {a['direction_limits_deg']['q_window']} deg.",
              f"* Sphere torque per module (cont/peak): {a['sphere_torque_per_module_Nm_cont_peak']} N m; roll (both modules): "
              f"{a['roll_torque_both_modules_Nm_cont_peak']} N m; knob-turn limit 30 % -> "
              f"{R['abenics_knob_turn_theta_limit']['roll_torque_Nm']} N m.",
              f"* Worst Theta-servo torque holding claw + rated payload, forearm horizontal, any (pitch, yaw, roll): "
              f"**{a['theta_servo_worst_Nm_rated_payload_forearm_horizontal']} N m** at {a['theta_servo_worst_pose_pyr']} -> "
              f"margin {a['margin_continuous']} on the continuous rating.",
              f"* q-servos: {a['q_servo_static_load']}.", f"* Speeds: {a['tool_speed_deg_s']} deg/s.", "",
              "![abenics](../calcs/output/abenics_direction_workspace.png)", ""]
    L += ["## 6. Structure, gears and belts (margin = allowable / stress; >= 1 required)", "",
          "| item | stress (MPa) | allowable (MPa) | margin | note |", "|---|---|---|---|---|"]
    for s_ in R["structure"]:
        flag = "" if s_["margin"] >= 1.0 else " **< 1**"
        L.append(f"| {s_['item']} | {s_['stress_MPa']} | {s_['allowable_MPa']} | {s_['margin']}{flag} | {s_['note']} |")
    L += ["", f"Dock: static moment {R['dock']['M_static_Nm']} N m (design x3: {R['dock']['M_design_Nm']} N m); thumb-screw "
          f"force {R['dock']['screw_force_N']} N vs ~1200 N insert pull-out [E] (margin {R['dock']['insert_margin']}).",
          f"Torque strut: {R['torque_strut']['strut_force_N']} N for a {R['torque_strut']['T_about_post_Nm']} N m twist about the post.", "",
          "## 7. Belts", "", "| joint | effective tension at peak (N) | allowable (N) | margin | static pretension (N) |", "|---|---|---|---|---|"]
    for j, v in R["belts"].items():
        L.append(f"| {j} | {v['effective_tension_at_peak_N']} | {v['allowable_N']} | {v['margin_peak']} | {v['recommended_static_pretension_N']} |")
    p = R["power"]
    L += ["", "## 8. Power", "", f"{p['servos']} servos: typical {p['servo_bus_typical_W']} W, worst {p['servo_bus_worst_W']} W "
          f"({p['servo_bus_worst_A']} A at 12 V); system typical {p['system_typical_W']} W, worst {p['system_worst_W']} W; "
          f"runtime on a {p['battery_example_Wh']} Wh pack ~{p['runtime_typical_h']} h typical.  {p['note']}", "",
          "## 9. Accuracy budget (TCP, mm)", "", "| source | values |", "|---|---|"]
    for j, v in R["tcp_error_budget"].items():
        L.append(f"| {j} | {v} |")
    L += ["", f"RSS of all listed terms: **{R['tcp_error_rss_mm']} mm** [E: servo backlash 0.5 deg and printed-gear "
          "backlash 0.15 mm are assumptions - measure in test T2.4].", "",
          "## 10. Workspace and door tasks", "",
          f"TCP x range {R['workspace']['tcp_x_range_mm']} mm, z range (above the dock) {R['workspace']['tcp_z_range_B_mm']} mm; "
          f"reaches the floor: {R['workspace']['reaches_floor']}.", "",
          "![workspace](../calcs/output/workspace_side.png)", "", "![LR](../calcs/output/workspace_top_LR.png)", "",
          "Knob turning (grip slip torque vs wrist roll capacity; knob mode caps the Theta servos at 30 % of stall):", "",
          "| case | jaw force (N) | slip torque, mu 0.5 / 0.8 (N m) | wrist roll capacity (N m) | knob torque capacity, mu 0.5 / 0.8 (N m) | limited by |",
          "|---|---|---|---|---|---|"]
    for k, v in R["door_tasks"].items():
        if k.startswith("knob_") and isinstance(v, dict) and "jaw_force_N" in v:
            L.append(f"| {k} | {v['jaw_force_N']} | {v['slip_torque_Nm_mu0.5_0.8']} | {v['wrist_roll_capacity_Nm']} | "
                     f"{v['knob_torque_capacity_Nm_mu0.5_0.8']} | {v['limited_by']} |")
    L += ["", "Demand: typical residential latch 0.3-1.5 N m [E]; BHMA A156.2 maximum 3.2 N m.  "
          "In knob mode the arm covers the typical range at either friction value; it cannot reach the BHMA maximum."]
    L += ["", "Knob reach window: `calcs/knob_window.py` -> docs/10_door_tasks.md.", "",
          "![door force](../calcs/output/door_force_capability.png)", "", "![knob window](../calcs/output/knob_window.png)", "",
          "## 11. Claw", "", f"{R['claw']}", ""]
    (D / "05_calculations.md").write_text("\n".join(L) + "\n")


write_report()
print("report written: docs/05_calculations.md")
