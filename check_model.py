#!/usr/bin/env python3
"""Checks for the generated G1 29-DoF rev 1.0 + BrainCo Revo2 model.  Exit code 1 on any FAIL.

  .venv/bin/python g1-brainco-description/check_model.py [--ultra PATH/TO/ULTRA/g1_29dof.xml] [--hold 3]   (from the hospital-sim root)

Checks: contract joint/actuator order; mesh files; MuJoCo load; home keyframe (feet on floor,
no penetration); zero-action PD hold at home; hand actuation + mimic coupling;
URDF vs MJCF (kinematics, masses, inertias, ranges, efforts, axes) using MuJoCo's own URDF
importer as an independent converter; MJCF vs Unitree's vendor MJCF and ULTRA's MJCF;
pinocchio parse of the URDFs (if pinocchio is importable).
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "g1_29dof_brainco"
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT))
from build_model import BODY_JOINTS, HAND_SLOTS, HOME_QPOS, ULTRA_KD, ULTRA_KP  # noqa: E402

RES = []


def rep(level, msg):
    RES.append(level)
    print(f"[{level}] {msg}")


def jid(m, n):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)


def name(m, obj, i):
    return mujoco.mj_id2name(m, obj, i)


def with_floor(path: Path) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(path))
    g = spec.worldbody.add_geom()
    g.name, g.type, g.size = "floor", mujoco.mjtGeom.mjGEOM_PLANE, [0, 0, 0.05]
    return spec.compile()


# ----------------------------------------------------------------------------- order
def check_order(m, tag, exact_qpos):
    body_adr = [m.jnt_qposadr[jid(m, n)] for n in BODY_JOINTS]
    if min(jid(m, n) for n in BODY_JOINTS) < 0:
        rep("FAIL", f"{tag}: missing body joints")
        return
    if exact_qpos:
        ok = body_adr == list(range(7, 36))
        rep("PASS" if ok else "FAIL", f"{tag}: qpos[7:36] == contract joint order: {ok}")
    else:
        ok = body_adr == sorted(body_adr)
        rep("PASS" if ok else "FAIL",
            f"{tag}: body joints appear in contract order in qpos (relative order {ok}); qposadr="
            f"{body_adr[:1]}..{body_adr[21:23]}..{body_adr[-1:]} (left-hand joints occupy the gap after "
            f"left_wrist_yaw: kinematic-tree order, unavoidable)")
    acts = [name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
    drv = [name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[i, 0]) for i in range(m.nu)]
    ok = drv[:29] == BODY_JOINTS
    rep("PASS" if ok else "FAIL", f"{tag}: actuators 0..28 drive the 29 body joints in contract order: {ok}")
    if m.nu > 29:
        exp = [f"{s}_hand_{h}" for s in ("left", "right") for h in HAND_SLOTS]
        ok = acts[29:] == exp
        rep("PASS" if ok else "FAIL", f"{tag}: actuators 29..40 = {acts[29:32]}..{acts[-1]} (left 6 then right 6): {ok}")
        rep("INFO", f"{tag}: hand actuator -> joint: " + ", ".join(f"{a}->{j}" for a, j in zip(acts[29:35], drv[29:35])))


# ----------------------------------------------------------------------------- meshes
def check_meshes():
    for f in sorted((OUT / "urdf").glob("*.urdf")):
        r = ET.parse(f).getroot()
        miss = []
        for mm in r.iter("mesh"):
            fn = mm.get("filename")
            p = (OUT / "meshes" / fn.split("package://g1_description/meshes/")[1]) if fn.startswith("package://") \
                else (f.parent / fn).resolve()
            if not p.exists():
                miss.append(fn)
        n = len(list(r.iter("mesh")))
        rep("PASS" if not miss else "FAIL", f"{f.name}: {n} mesh refs, {len(miss)} missing {miss[:3]}")


# ----------------------------------------------------------------------------- keyframe + hold
def check_home_and_hold(m, tag, hold):
    d = mujoco.MjData(m)
    k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
    if k < 0:
        rep("FAIL", f"{tag}: no 'home' keyframe")
        return
    mujoco.mj_resetDataKeyframe(m, d, k)
    q_home = np.array([d.qpos[m.jnt_qposadr[jid(m, n)]] for n in BODY_JOINTS])
    ok = np.allclose(q_home, HOME_QPOS)
    rep("PASS" if ok else "FAIL", f"{tag}: home body joints == ULTRA default_dof_pos: {ok}")
    mujoco.mj_forward(m, d)
    foot = [g for g in range(m.ngeom) if "foot" in (name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]
    bottoms = [d.geom_xpos[g][2] - m.geom_size[g][0] for g in foot]
    pen = min([d.contact[i].dist for i in range(d.ncon)] or [0.0])
    ok = len(foot) == 8 and max(abs(b) for b in bottoms) < 1e-3 and pen > -1e-3
    rep("PASS" if ok else "FAIL",
        f"{tag}: home pelvis z={d.qpos[2]:.4f} m, 8 foot spheres bottom z in [{min(bottoms):.5f}, {max(bottoms):.5f}], "
        f"{d.ncon} contacts, min contact dist {pen*1000:.2f} mm")
    # lowest non-foot collidable geom
    others = [g for g in range(m.ngeom) if m.geom_bodyid[g] > 0 and (m.geom_contype[g] or m.geom_conaffinity[g])
              and g not in foot]
    if others:
        mujoco.mj_forward(m, d)
        low = min(others, key=lambda g: d.geom_xpos[g][2])
        rep("INFO", f"{tag}: {len(others)} non-foot collision geoms; lowest centre z={d.geom_xpos[low][2]:.3f} "
                    f"({name(m, mujoco.mjtObj.mjOBJ_GEOM, low)})")

    # zero-action PD hold, target = home, hands commanded open
    for label, kp, kd in (("ULTRA kp/kd", ULTRA_KP, ULTRA_KD), ("stiff test PD", STIFF_KP, STIFF_KD)):
        t_fail, zs, tilts = pd_hold(m, k, kp, kd, hold)
        i1 = int(1.0 / m.opt.timestep) - 1
        if label.startswith("ULTRA"):
            lvl = "PASS" if (t_fail is None or t_fail >= 1.0) else "WARN"
        else:
            lvl = "PASS" if t_fail is None else "FAIL"
        rep(lvl, f"{tag}: zero-action PD hold ({label}) {hold:.1f}s: pelvis z {zs[0]:.4f} -> {zs[min(i1, len(zs)-1)]:.4f} (1 s) "
                 f"-> {zs[-1]:.4f}; max tilt {max(tilts):.2f} deg; fell (z<0.6 or tilt>30deg) at "
                 f"{'never' if t_fail is None else f'{t_fail:.3f} s'}")


# test-only gains (not for deployment): legs+waist 500/10, arms 100/3 -> checks contact/mass model can stand
STIFF_KP = [500.0] * 15 + [100.0] * 14
STIFF_KD = [10.0] * 15 + [3.0] * 14


def pd_hold(m, key, kp, kd, T, d=None, vendor_start=False):
    d = mujoco.MjData(m) if d is None else d
    qadr = np.array([m.jnt_qposadr[jid(m, n)] for n in BODY_JOINTS])
    vadr = np.array([m.jnt_dofadr[jid(m, n)] for n in BODY_JOINTS])
    if vendor_start:
        mujoco.mj_resetData(m, d)
        d.qpos[qadr] = HOME_QPOS
        mujoco.mj_kinematics(m, d)
        d.qpos[2] -= min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in range(m.ngeom)
                         if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE and m.geom_bodyid[g] > 0)
    else:
        mujoco.mj_resetDataKeyframe(m, d, key)
    kp, kd = np.array(kp), np.array(kd)
    act = [next(a for a in range(m.nu) if m.actuator_trnid[a, 0] == jid(m, n)) for n in BODY_JOINTS]
    lim = np.array([m.jnt_actfrcrange[jid(m, n)][1] if m.jnt_actfrclimited[jid(m, n)] else
                    m.actuator_ctrlrange[a, 1] for n, a in zip(BODY_JOINTS, act)])
    zs, tilts, t_fail = [], [], None
    for i in range(int(T / m.opt.timestep)):
        tau = kp * (np.array(HOME_QPOS) - d.qpos[qadr]) - kd * d.qvel[vadr]
        d.ctrl[act] = np.clip(tau, -lim, lim)
        mujoco.mj_step(m, d)
        w = d.qpos[3:7]
        up = 1 - 2 * (w[1] ** 2 + w[2] ** 2)
        tilt = math.degrees(math.acos(max(-1, min(1, up))))
        zs.append(d.qpos[2])
        tilts.append(tilt)
        if t_fail is None and (d.qpos[2] < 0.6 or tilt > 30):
            t_fail = (i + 1) * m.opt.timestep
    return t_fail, zs, tilts


def check_hands(m):
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    # hold body with PD while closing hands
    qadr = np.array([m.jnt_qposadr[jid(m, n)] for n in BODY_JOINTS])
    vadr = np.array([m.jnt_dofadr[jid(m, n)] for n in BODY_JOINTS])
    kp, kd = np.array(STIFF_KP), np.array(STIFF_KD)
    d.ctrl[29:] = m.actuator_ctrlrange[29:, 1]
    for _ in range(int(1.0 / m.opt.timestep)):
        d.ctrl[:29] = np.clip(kp * (np.array(HOME_QPOS) - d.qpos[qadr]) - kd * d.qvel[vadr],
                              m.actuator_ctrlrange[:29, 0], m.actuator_ctrlrange[:29, 1])
        mujoco.mj_step(m, d)
    errs = []
    for a in range(29, m.nu):
        j = m.actuator_trnid[a, 0]
        errs.append(d.ctrl[a] - d.qpos[m.jnt_qposadr[j]])
    ok = max(abs(e) for e in errs) < 0.05
    rep("PASS" if ok else "FAIL", f"hands: close to upper limit in 1 s, max tracking error {max(map(abs, errs)):.4f} rad")
    # mimic coupling
    worst = 0
    for e in range(m.neq):
        if m.eq_type[e] != mujoco.mjtEq.mjEQ_JOINT:
            continue
        j1, j2 = m.eq_obj1id[e], m.eq_obj2id[e]
        c = m.eq_data[e]
        q1, q2 = d.qpos[m.jnt_qposadr[j1]], d.qpos[m.jnt_qposadr[j2]]
        worst = max(worst, abs(q1 - (c[0] + c[1] * q2)))
    rep("PASS" if worst < 0.02 else "FAIL", f"hands: {m.neq} mimic equality constraints, worst violation {worst:.4f} rad")
    tot = 0.0
    for b in range(m.nbody):
        n = name(m, mujoco.mjtObj.mjOBJ_BODY, b)
    return errs


# ----------------------------------------------------------------------------- URDF vs MJCF
def urdf_as_mujoco(urdf_path: Path) -> mujoco.MjModel:
    """Independent conversion: MuJoCo's own URDF importer (geometry stripped; inertial/kinematics kept)."""
    r = ET.parse(urdf_path).getroot()
    for l in r.findall("link"):
        for tag in ("visual", "collision"):
            for e in l.findall(tag):
                l.remove(e)
    for e in r.findall("mujoco"):
        r.remove(e)
    mj = ET.SubElement(r, "mujoco")
    ET.SubElement(mj, "compiler", fusestatic="true", discardvisual="true", balanceinertia="false")
    return mujoco.MjModel.from_xml_string(ET.tostring(r, encoding="unicode"))


def compare_urdf_mjcf(m: mujoco.MjModel):
    u = urdf_as_mujoco(OUT / "urdf/g1_29dof_brainco.urdf")
    rng = np.random.default_rng(0)
    du, dm = mujoco.MjData(u), mujoco.MjData(m)
    # joint set
    ju = {name(u, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(u.njnt)}
    jm = {name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)} - {"floating_base_joint"}
    rep("PASS" if ju == jm else "FAIL", f"URDF vs MJCF: same {len(jm)} joints: {ju == jm} {sorted(ju ^ jm)[:4]}")
    worst = {"pos": 0, "rot": 0, "axis": 0, "range": 0}
    for trial in range(5):
        for n in jm:
            lo, hi = m.jnt_range[jid(m, n)]
            v = rng.uniform(lo, hi)
            dm.qpos[m.jnt_qposadr[jid(m, n)]] = v
            du.qpos[u.jnt_qposadr[jid(u, n)]] = v
        dm.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        mujoco.mj_kinematics(m, dm)
        mujoco.mj_kinematics(u, du)
        for b in range(1, u.nbody):
            bn = name(u, mujoco.mjtObj.mjOBJ_BODY, b)
            bm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
            worst["pos"] = max(worst["pos"], np.abs(du.xpos[b] - dm.xpos[bm]).max())
            worst["rot"] = max(worst["rot"], np.abs(du.xmat[b] - dm.xmat[bm]).max())
        for n in jm:
            worst["axis"] = max(worst["axis"], np.abs(du.xaxis[jid(u, n)] - dm.xaxis[jid(m, n)]).max())
    mimic = []
    for n in jm:
        if not m.jnt_limited[jid(m, n)]:
            mimic.append(n)
            continue
        worst["range"] = max(worst["range"], np.abs(u.jnt_range[jid(u, n)] - m.jnt_range[jid(m, n)]).max())
    rep("INFO", f"URDF vs MJCF: {len(mimic)} coupled (mimic) joints are unlimited in MJCF by design; URDF ranges: "
        + ", ".join(f"{n}={np.round(u.jnt_range[jid(u, n)], 4).tolist()}" for n in mimic[:5]) + " ...")
    ok = all(v < 1e-6 for v in worst.values())
    rep("PASS" if ok else "FAIL", "URDF vs MJCF kinematics at 5 random configs: max |dpos| %.2e m, |dR| %.2e, "
        "|daxis| %.2e, |drange| %.2e" % (worst["pos"], worst["rot"], worst["axis"], worst["range"]))
    # masses / inertia per body
    dmass = dcom = dI = 0
    for b in range(1, u.nbody):
        bn = name(u, mujoco.mjtObj.mjOBJ_BODY, b)
        bm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        dmass = max(dmass, abs(u.body_mass[b] - m.body_mass[bm]))
    mujoco.mj_forward(u, du)
    mujoco.mj_forward(m, dm)
    for b in range(1, u.nbody):
        bm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name(u, mujoco.mjtObj.mjOBJ_BODY, b))
        dcom = max(dcom, np.abs(du.xipos[b] - dm.xipos[bm]).max())
        Iu = du.ximat[b].reshape(3, 3) @ np.diag(u.body_inertia[b]) @ du.ximat[b].reshape(3, 3).T
        Im = dm.ximat[bm].reshape(3, 3) @ np.diag(m.body_inertia[bm]) @ dm.ximat[bm].reshape(3, 3).T
        dI = max(dI, np.abs(Iu - Im).max())
    urdf_total = sum(float(e.get("value")) for e in ET.parse(OUT / "urdf/g1_29dof_brainco.urdf").getroot().iter("mass"))
    ok = dmass < 1e-6 and dcom < 1e-6 and dI < 1e-8 and abs(urdf_total - m.body_mass.sum()) < 1e-6
    rep("PASS" if ok else "FAIL", f"URDF vs MJCF inertials: max |dmass| {dmass:.2e} kg, |dCOM| {dcom:.2e} m, "
        f"|dI| {dI:.2e} kg m^2; total URDF {urdf_total:.4f} kg vs MJCF {m.body_mass.sum():.4f} kg")
    # efforts: URDF <limit effort> vs MJCF actuatorfrcrange / motor ctrlrange
    r = ET.parse(OUT / "urdf/g1_29dof_brainco.urdf").getroot()
    eff = {j.get("name"): float(j.find("limit").get("effort")) for j in r.findall("joint") if j.find("limit") is not None}
    bad = []
    for a in range(m.nu):
        n = name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a, 0])
        e = eff[n]
        hi = m.actuator_ctrlrange[a, 1] if a < 29 else m.actuator_forcerange[a, 1]
        if abs(hi - e) > 1e-9:
            bad.append((n, e, hi))
    rep("PASS" if not bad else "FAIL", f"URDF effort == MJCF motor ctrlrange / hand forcerange for all {m.nu} actuators {bad[:3]}")


def mjcf_joint_table(path):
    r = ET.parse(path).getroot()
    out = {}
    for j in r.iter("joint"):
        if j.get("name") in BODY_JOINTS:
            out[j.get("name")] = {"range": [float(x) for x in j.get("range").split()],
                                  "frc": float(j.get("actuatorfrcrange", "nan nan").split()[1]),
                                  "axis": j.get("axis")}
    return out


def compare_external(m: mujoco.MjModel, path: Path, label: str):
    """Report every disagreement with an external MJCF (vendor Unitree / ULTRA)."""
    if not path.exists():
        rep("INFO", f"{label}: {path} not found, skipped")
        return
    try:
        sp = mujoco.MjSpec.from_file(str(path))
        sp.meshdir = str(path.parent / "meshes") if (path.parent / "meshes").exists() else str(OUT / "meshes/g1")
        for me in sp.meshes:  # meshes we do not ship (rubber hands): substitute; masses come from <inertial>
            if not (Path(sp.meshdir) / me.file).exists():
                me.file = "left_wrist_yaw_link.STL"
        x = sp.compile()
    except Exception as e:  # noqa: BLE001
        rep("INFO", f"{label}: could not load ({e}); comparing XML text only")
        x = None
    ext = mjcf_joint_table(path)
    ours = mjcf_joint_table(OUT / "mjcf/g1_29dof_brainco.xml")
    diffs = []
    for n in BODY_JOINTS:
        a, b = ours[n], ext.get(n)
        if b is None:
            diffs.append(f"{n} missing")
            continue
        if not np.allclose(a["range"], b["range"], atol=1e-6):
            diffs.append(f"{n} range ours {a['range']} vs {b['range']}")
        if not (math.isnan(b["frc"]) or abs(a["frc"] - b["frc"]) < 1e-9):
            diffs.append(f"{n} effort ours {a['frc']:g} vs {b['frc']:g}")
    if x is not None:
        for b in range(1, x.nbody):
            bn = name(x, mujoco.mjtObj.mjOBJ_BODY, b)
            bm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
            if bm < 0:
                continue
            if abs(x.body_mass[b] - m.body_mass[bm]) > 1e-4:
                diffs.append(f"body {bn} mass ours {m.body_mass[bm]:.5f} vs {x.body_mass[b]:.5f}")
            if bn != "pelvis" and np.abs(x.body_pos[b] - m.body_pos[bm]).max() > 1e-5:
                diffs.append(f"body {bn} pos ours {np.round(m.body_pos[bm], 6)} vs {np.round(x.body_pos[b], 6)}")
        # armature / damping / frictionloss on body joints (summarised per attribute)
        for attr in ("dof_armature", "dof_damping", "dof_frictionloss"):
            ours_v, ext_v = [], []
            for n in BODY_JOINTS:
                jx, jm_ = mujoco.mj_name2id(x, mujoco.mjtObj.mjOBJ_JOINT, n), jid(m, n)
                ours_v.append(float(getattr(m, attr)[m.jnt_dofadr[jm_]]))
                ext_v.append(float(getattr(x, attr)[x.jnt_dofadr[jx]]))
            if not np.allclose(ours_v, ext_v, atol=1e-9):
                diffs.append(f"{attr[4:]} (29 body joints) ours {sorted(set(ours_v))} vs {sorted(set(ext_v))}")
        diffs.append(f"total mass ours {m.body_mass.sum():.4f} (full) vs {x.body_mass.sum():.4f}")
    if x is not None and label.startswith("vs Unitree"):
        t_fail, _, _ = pd_hold(x, -1, ULTRA_KP, ULTRA_KD, 3.0, vendor_start=True)
        diffs.append(f"reference: same ULTRA-gain zero-action PD hold on the vendor MJCF falls at "
                     f"{'never' if t_fail is None else f'{t_fail:.3f} s'}")
    rep("INFO", f"{label}: {len(diffs)} differences")
    for dd in diffs:
        print("       - " + dd)


# ----------------------------------------------------------------------------- pinocchio
def check_pinocchio(m):
    try:
        import pinocchio as pin
    except ImportError:
        rep("INFO", "pinocchio not importable in this interpreter; run under `uv run --with pin` to test")
        return
    for f in ("g1_29dof_brainco.urdf", "g1_29dof_brainco_ik.urdf"):
        p = OUT / "urdf" / f
        try:
            robot = pin.RobotWrapper.BuildFromURDF(str(p), [str(OUT / "urdf")])
        except Exception as e:  # noqa: BLE001
            rep("FAIL", f"pinocchio {pin.__version__} cannot load {f}: {e}")
            continue
        mdl = robot.model
        rep("PASS", f"pinocchio {pin.__version__} loads {f}: nq={mdl.nq} nv={mdl.nv} njoints={mdl.njoints - 1}, "
                    f"{len(robot.visual_model.geometryObjects)} visual geoms")
        if f.endswith("_ik.urdf"):
            lock = [mdl.getJointId(n) for n in BODY_JOINTS[:15]]
            red = pin.buildReducedModel(mdl, lock, pin.neutral(mdl))
            rep("PASS" if red.nq == 14 else "FAIL", f"pinocchio reduced (legs+waist locked) nq={red.nq} (xr_teleoperate arm IK)")
            # FK cross-check vs MuJoCo at a random arm config: left_base_link frame
            rng = np.random.default_rng(1)
            q = pin.neutral(mdl)
            d = mujoco.MjData(m)
            d.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
            for n in BODY_JOINTS:
                lo, hi = m.jnt_range[jid(m, n)]
                v = rng.uniform(lo, hi) * 0.5
                q[mdl.joints[mdl.getJointId(n)].idx_q] = v
                d.qpos[m.jnt_qposadr[jid(m, n)]] = v
            mujoco.mj_kinematics(m, d)
            data = mdl.createData()
            pin.framesForwardKinematics(mdl, data, q)
            err = 0
            for fr in ("left_base_link", "right_base_link", "mid360_link", "left_index_tip_link"):
                s = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, fr)
                T = data.oMf[mdl.getFrameId(fr)]
                err = max(err, np.abs(T.translation - d.site_xpos[s]).max(),
                          np.abs(T.rotation - d.site_xmat[s].reshape(3, 3)).max())
            rep("PASS" if err < 1e-6 else "FAIL", f"pinocchio FK == MuJoCo site poses (hand base, mid360, fingertip): max err {err:.2e}")
        else:
            mim = [j for j in ET.parse(p).getroot().findall("joint") if j.find("mimic") is not None]
            rep("INFO", f"full URDF has {len(mim)} mimic joints; pinocchio nq={mdl.nq} "
                        f"({'mimic honoured' if mdl.nq == 29 + 12 else 'mimic joints loaded as independent joints'})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ultra", type=Path,
                    default=ROOT.parent / "g1-whole-body-rl/third_party/ULTRA/ultra/data/assets/g1/g1_29dof.xml",
                    help="ULTRA's stock G1 MJCF used as the reference for the joint-order/limit comparison")
    ap.add_argument("--hold", type=float, default=3.0)
    args = ap.parse_args()
    print(f"mujoco {mujoco.__version__}")

    check_meshes()
    full = mujoco.MjModel.from_xml_path(str(OUT / "mjcf/g1_29dof_brainco.xml"))
    fixed = mujoco.MjModel.from_xml_path(str(OUT / "mjcf/g1_29dof_brainco_fixedhands.xml"))
    scene = mujoco.MjModel.from_xml_path(str(OUT / "mjcf/scene_flat.xml"))
    rep("PASS", f"MuJoCo loads full (nq={full.nq} nv={full.nv} nu={full.nu} neq={full.neq}), "
                f"fixedhands (nq={fixed.nq} nu={fixed.nu}), scene_flat")
    check_order(full, "full", exact_qpos=False)
    check_order(fixed, "fixedhands", exact_qpos=True)

    def hand_mass(m):
        wy = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_wrist_yaw_link") for s in ("left", "right")]
        return [m.body_subtreemass[b] for b in wy]
    mujoco.mj_setConst(full, mujoco.MjData(full))
    mujoco.mj_setConst(fixed, mujoco.MjData(fixed))
    hm, hx = hand_mass(full), hand_mass(fixed)
    rep("PASS" if abs(full.body_mass.sum() - fixed.body_mass.sum()) < 1e-9 and np.allclose(hm, hx) else "FAIL",
        f"mass: full {full.body_mass.sum():.4f} kg, fixedhands {fixed.body_mass.sum():.4f} kg; wrist_yaw subtree "
        f"(wrist link 0.0846 + adapter 0.0228 + Revo2 hand) L/R {hm[0]:.4f}/{hm[1]:.4f} kg (fixed {hx[0]:.4f}/{hx[1]:.4f})")

    # sole geometry identical
    def soles(m):
        return sorted((name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]), tuple(np.round(m.geom_size[g], 6)),
                       tuple(np.round(m.geom_pos[g], 6)), int(m.geom_contype[g]), int(m.geom_conaffinity[g]),
                       tuple(np.round(m.geom_friction[g], 6)))
                      for g in range(m.ngeom) if "foot" in (name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""))
    rep("PASS" if soles(full) == soles(fixed) else "FAIL", f"sole contact geometry identical full vs fixedhands "
        f"({len(soles(full))} spheres r={soles(full)[0][1][0]} m)")

    check_home_and_hold(scene, "full(scene_flat)", args.hold)
    check_home_and_hold(with_floor(OUT / "mjcf/g1_29dof_brainco_fixedhands.xml"), "fixedhands(+floor)", args.hold)
    check_hands(scene)
    compare_urdf_mjcf(full)
    compare_external(full, ROOT / "sources/unitree_ros/g1_29dof_rev_1_0.xml", "vs Unitree vendor MJCF g1_29dof_rev_1_0.xml")
    compare_external(full, args.ultra, "vs ULTRA g1_29dof.xml")
    check_pinocchio(full)

    nf = RES.count("FAIL")
    print(f"\n{RES.count('PASS')} PASS, {RES.count('WARN')} WARN, {nf} FAIL")
    sys.exit(1 if nf else 0)


if __name__ == "__main__":
    main()
