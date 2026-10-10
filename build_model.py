#!/usr/bin/env python3
"""Regenerate the Unitree G1 29-DoF rev 1.0 + BrainCo Revo2 model from pinned vendor sources.

Outputs (never hand-edit them; edit this script and re-run):
  g1_29dof_brainco/urdf/g1_29dof_brainco.urdf          full URDF, mesh paths relative (../meshes/...)
  g1_29dof_brainco/urdf/g1_29dof_brainco_ros.urdf      same, mesh paths package://g1_description/meshes/...
  g1_29dof_brainco/urdf/g1_29dof_brainco_ik.urdf       29 body joints only, hands rigid (pinocchio IK)
  g1_29dof_brainco/mjcf/g1_29dof_brainco.xml           full MuJoCo model (29 body + 12 hand actuators)
  g1_29dof_brainco/mjcf/g1_29dof_brainco_fixedhands.xml  hands welded, for RL / MJX
  g1_29dof_brainco/mjcf/scene_flat.xml                 flat floor + lights + full model
  g1_29dof_brainco/robot_spec.json                     joint order, defaults, gains, limits, hand map

Inputs: sources/ (vendored text files, pinned commits in SOURCE.md) + g1_29dof_brainco/meshes/.
`--fetch` re-downloads the pinned upstream repos and refreshes sources/ and meshes/.

Usage:  python g1-brainco-description/build_model.py [--fetch] [--cache DIR]   (from the hospital-sim root)
Needs only numpy (+ git for --fetch). Any python >= 3.8.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "sources"
OUT = ROOT / "g1_29dof_brainco"
MESH = OUT / "meshes"

# ----------------------------------------------------------------------------- pins
PINS = {
    "unitree_ros": {
        "url": "https://github.com/unitreerobotics/unitree_ros.git",
        "commit": "da52948f035165aae2709d30255f5cd3e62875a0",
        "sparse": ["robots/g1_description", "robots/g1_with_brainco_hand", "LICENSE"],
    },
    "revo2_description": {
        "url": "https://github.com/BrainCoTech/revo2_description.git",
        "commit": "92cc697c7fa691db59404ce52344f3969a5ef7a6",
        "sparse": None,
    },
}

# ----------------------------------------------------------------------------- contract
BODY_JOINTS = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]
BODY_JOINTS = [j + "_joint" for j in BODY_JOINTS]

# Contract hand order: thumb, thumb_aux (rotation), index, middle, ring, pinky.
# BrainCo Revo2 docs (brainco-hz.com/docs/revolimb-hand/en/revo2/parameters.html):
#   thumb = "Thumb Flex" 0..59 deg, thumb_aux = abduction/adduction 0..90 deg, fingers 0..81 deg.
# In revo2_description: thumb_proximal_joint 0..1.0472 (60 deg) = flex,
#                       thumb_metacarpal_joint 0..1.55334 (89 deg) = aux, finger proximal 0..1.41 (81 deg).
HAND_SLOTS = ["thumb", "thumb_aux", "index", "middle", "ring", "pinky"]
HAND_SLOT_JOINT = {
    "thumb": "thumb_proximal_joint",
    "thumb_aux": "thumb_metacarpal_joint",
    "index": "index_proximal_joint",
    "middle": "middle_proximal_joint",
    "ring": "ring_proximal_joint",
    "pinky": "pinky_proximal_joint",
}

# ULTRA (github.com/Sirui-Xu/ULTRA @ ec972c7) ultra/sim2sim_teacher.py default_dof_pos
# (identical to ultra/env/tasks/ultra_g1.py init_dof). g1_teacher.yaml itself has no
# default_joint_angles; see SOURCE.md.
HOME_QPOS = [
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
]
# ULTRA ultra/env/tasks/humanoid_g1.py (released-policy PD gains and armature), contract order.
ULTRA_KP = [40.179238, 99.098428, 40.179238, 99.098428, 28.501246, 28.501246,
            40.179238, 99.098428, 40.179238, 99.098428, 28.501246, 28.501246,
            40.179238, 28.501246, 28.501246,
            14.250623, 14.250623, 14.250623, 14.250623, 14.250623, 16.778327, 16.778327,
            14.250623, 14.250623, 14.250623, 14.250623, 14.250623, 16.778327, 16.778327]
ULTRA_KD = [2.557890, 6.308802, 2.557890, 6.308802, 1.814446, 1.814446,
            2.557890, 6.308802, 2.557890, 6.308802, 1.814446, 1.814446,
            2.557890, 1.814446, 1.814446,
            0.907223, 0.907223, 0.907223, 0.907223, 0.907223, 1.068142, 1.068142,
            0.907223, 0.907223, 0.907223, 0.907223, 0.907223, 1.068142, 1.068142]
ULTRA_ARMATURE = [0.010178, 0.025102, 0.010178, 0.025102, 0.007219, 0.007219,
                  0.010178, 0.025102, 0.010178, 0.025102, 0.007219, 0.007219,
                  0.010178, 0.007219, 0.007219,
                  0.003610, 0.003610, 0.003610, 0.003610, 0.003610, 0.004250, 0.004250,
                  0.003610, 0.003610, 0.003610, 0.003610, 0.003610, 0.004250, 0.004250]
ULTRA_EFFORT = [88.0, 139.0, 88.0, 139.0, 50.0, 50.0, 88.0, 139.0, 88.0, 139.0, 50.0, 50.0,
                88.0, 50.0, 50.0] + [25.0] * 5 + [5.0] * 2 + [25.0] * 5 + [5.0] * 2

# Revo2 finger simulation parameters. NOT specified by BrainCo -> chosen defaults (flagged in SOURCE.md).
HAND_ARMATURE = 1e-3       # kg m^2, numerical stand-in for the finger drive's reflected inertia
HAND_DAMPING = 0.01        # N m s/rad passive joint damping on every finger joint
HAND_KP_ERR_AT_EFFORT = 0.25  # kp chosen so the URDF effort limit is reached at 0.25 rad error

SIM_DT = 0.002

# ----------------------------------------------------------------------------- math helpers


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def mat_to_quat(m):
    m = np.asarray(m, dtype=float)
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    if q[0] < 0:
        q = -q
    return q / np.linalg.norm(q)


def fmt(v, nd=9):
    out = []
    for x in np.atleast_1d(v):
        x = float(x)
        if abs(x) < 10 ** (-nd):
            x = 0.0
        s = f"{x:.{nd}g}"
        out.append("0" if s in ("-0", "0") else s)
    return " ".join(out)


def parse_xyz(el, attr, default="0 0 0"):
    return np.array([float(v) for v in (el.get(attr, default) if el is not None else default).split()])


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()

# ----------------------------------------------------------------------------- fetch


def fetch(cache: Path):
    cache.mkdir(parents=True, exist_ok=True)
    dirs = {}
    for name, pin in PINS.items():
        d = cache / name
        if not (d / ".git").exists():
            args = ["git", "clone", "-q", "--filter=blob:none", "--no-checkout", pin["url"], str(d)]
            subprocess.run(args, check=True)
        subprocess.run(["git", "-C", str(d), "fetch", "-q", "--depth", "1", "origin", pin["commit"]], check=False)
        if pin["sparse"]:
            subprocess.run(["git", "-C", str(d), "sparse-checkout", "set", "--no-cone", *pin["sparse"]], check=True)
        subprocess.run(["git", "-C", str(d), "checkout", "-q", pin["commit"]], check=True)
        dirs[name] = d
    ur, rv = dirs["unitree_ros"], dirs["revo2_description"]
    g1d, bch = ur / "robots/g1_description", ur / "robots/g1_with_brainco_hand"
    (SRC / "unitree_ros").mkdir(parents=True, exist_ok=True)
    (SRC / "revo2_description").mkdir(parents=True, exist_ok=True)
    for f in ["g1_29dof_rev_1_0.urdf", "g1_29dof_rev_1_0.xml", "motor_info.md"]:
        shutil.copy2(g1d / f, SRC / "unitree_ros" / f)
    shutil.copy2(bch / "g1_29dof_mode_15_brainco_hand.urdf", SRC / "unitree_ros")
    shutil.copy2(ur / "LICENSE", SRC / "unitree_ros" / "LICENSE")
    for f in ["urdf/revo2_left_hand.urdf", "urdf/revo2_right_hand.urdf", "LICENSE", "package.xml"]:
        shutil.copy2(rv / f, SRC / "revo2_description" / Path(f).name)
    # meshes
    if MESH.exists():
        shutil.rmtree(MESH)
    (MESH / "g1").mkdir(parents=True)
    g1_urdf = ET.parse(g1d / "g1_29dof_rev_1_0.urdf").getroot()
    for m in g1_urdf.iter("mesh"):
        fn = Path(m.get("filename")).name
        if "rubber_hand" in fn:
            continue
        shutil.copy2(g1d / "meshes" / fn, MESH / "g1" / fn)
    for side in ("left", "right"):
        shutil.copy2(bch / "meshes" / f"{side}_base2_link.STL", MESH / "g1" / f"{side}_base2_link.STL")
        shutil.copytree(rv / "meshes" / f"revo2_{side}_hand", MESH / f"revo2_{side}_hand")
    print(f"fetched pinned sources into {SRC} and meshes into {MESH}")

# ----------------------------------------------------------------------------- URDF assembly


def _link_meshes_to(link: ET.Element, fn):
    for m in link.iter("mesh"):
        m.set("filename", fn(m.get("filename")))


def build_urdf() -> ET.ElementTree:
    body = ET.parse(SRC / "unitree_ros/g1_29dof_rev_1_0.urdf").getroot()
    bc = ET.parse(SRC / "unitree_ros/g1_29dof_mode_15_brainco_hand.urdf").getroot()
    body.set("name", "g1_29dof_brainco")
    # Unitree's <mujoco><compiler meshdir="meshes"/> hint no longer matches our layout; MJCF is generated below.
    for e in body.findall("mujoco"):
        body.remove(e)

    # 1) body meshes -> ../meshes/g1/
    for m in body.iter("mesh"):
        m.set("filename", "../meshes/g1/" + Path(m.get("filename")).name)

    # 2) remove the Unitree rubber hands (fixed children of *_wrist_yaw_link)
    for side in ("left", "right"):
        for j in list(body.findall("joint")):
            if j.find("child").get("link") == f"{side}_rubber_hand":
                body.remove(j)
        for l in list(body.findall("link")):
            if l.get("name") == f"{side}_rubber_hand":
                body.remove(l)

    bc_links = {l.get("name"): l for l in bc.findall("link")}
    bc_joints = {j.get("name"): j for j in bc.findall("joint")}
    for side in ("left", "right"):
        # 3) wrist adapter (base2) + hand mount transform from Unitree's official G1+BrainCo URDF
        ad = copy.deepcopy(bc_links[f"{side}_base2_link"])
        _link_meshes_to(ad, lambda f: "../meshes/g1/" + Path(f).name)
        body.append(copy.deepcopy(bc_joints[f"{side}_base2_joint"]))
        body.append(ad)
        mount = copy.deepcopy(bc_joints[f"{side}_base_joint"])
        mount.set("name", f"{side}_hand_mount_joint")
        body.append(mount)
        # 4) Revo2 hand from BrainCo's revo2_description (vendor, v1.0.0); root link = <side>_base_link
        hand = ET.parse(SRC / f"revo2_description/revo2_{side}_hand.urdf").getroot()
        for el in hand:
            if el.tag == "link":
                el = copy.deepcopy(el)
                _link_meshes_to(el, lambda f, s=side: f"../meshes/revo2_{s}_hand/" + Path(f).name)
                body.append(el)
            elif el.tag == "joint":
                body.append(copy.deepcopy(el))
            elif el.tag == "material" and body.find(f"material[@name='{el.get('name')}']") is None:
                body.insert(0, copy.deepcopy(el))
    ET.indent(body, "  ")
    return ET.ElementTree(body)


def urdf_variant_ros(tree: ET.ElementTree) -> ET.ElementTree:
    t = copy.deepcopy(tree)
    for m in t.getroot().iter("mesh"):
        m.set("filename", m.get("filename").replace("../meshes/", "package://g1_description/meshes/"))
    return t


def urdf_variant_ik(tree: ET.ElementTree) -> ET.ElementTree:
    """29 actuated body joints only: every hand joint becomes fixed (open hand, q=0), mimic removed."""
    t = copy.deepcopy(tree)
    r = t.getroot()
    r.set("name", "g1_29dof_brainco_ik")
    for j in r.findall("joint"):
        if j.get("type") != "fixed" and j.get("name") not in BODY_JOINTS:
            j.set("type", "fixed")
            for tag in ("axis", "limit", "mimic", "dynamics"):
                for e in j.findall(tag):
                    j.remove(e)
    return t

# ----------------------------------------------------------------------------- URDF -> body tree


class Body:
    def __init__(self, name):
        self.name = name
        self.pos = np.zeros(3)
        self.rot = np.eye(3)
        self.mass = 0.0
        self.com = np.zeros(3)
        self.inertia = np.zeros((3, 3))  # about COM, body frame
        self.joint = None  # dict
        self.geoms = []    # dicts with pos, rot, ...
        self.sites = []    # dicts with name, pos, rot
        self.children = []


def urdf_to_tree(urdf: ET.Element, fixed_hands: bool) -> Body:
    materials = {}
    for m in urdf.findall("material"):
        c = m.find("color")
        if c is not None:
            materials[m.get("name")] = c.get("rgba")
    links = {l.get("name"): l for l in urdf.findall("link")}
    joints = urdf.findall("joint")
    child_joints = {}
    children = set()
    for j in joints:
        child_joints.setdefault(j.find("parent").get("link"), []).append(j)
        children.add(j.find("child").get("link"))
    root = [n for n in links if n not in children]
    assert len(root) == 1, root

    def make(link_name) -> Body:
        L = links[link_name]
        b = Body(link_name)
        ine = L.find("inertial")
        if ine is not None:
            o = ine.find("origin")
            R = rpy_to_mat(*parse_xyz(o, "rpy"))
            b.mass = float(ine.find("mass").get("value"))
            b.com = parse_xyz(o, "xyz")
            i = ine.find("inertia")
            I = np.array([[float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz"))],
                          [float(i.get("ixy")), float(i.get("iyy")), float(i.get("iyz"))],
                          [float(i.get("ixz")), float(i.get("iyz")), float(i.get("izz"))]])
            b.inertia = R @ I @ R.T
        for kind in ("visual", "collision"):
            for g in L.findall(kind):
                o = g.find("origin")
                geo = g.find("geometry")[0]
                d = {"kind": kind, "pos": parse_xyz(o, "xyz"), "rot": rpy_to_mat(*parse_xyz(o, "rpy")),
                     "link": link_name}
                if geo.tag == "mesh":
                    d["type"] = "mesh"
                    d["file"] = geo.get("filename").replace("../meshes/", "")
                elif geo.tag == "sphere":
                    d["type"] = "sphere"
                    d["size"] = [float(geo.get("radius"))]
                elif geo.tag == "cylinder":
                    d["type"] = "cylinder"
                    d["size"] = [float(geo.get("radius")), float(geo.get("length")) / 2]
                elif geo.tag == "box":
                    d["type"] = "box"
                    d["size"] = [v / 2 for v in parse_xyz(geo, "size")]
                else:
                    raise ValueError(geo.tag)
                rgba = None
                mat = g.find("material")
                if mat is not None:
                    c = mat.find("color")
                    rgba = c.get("rgba") if c is not None else materials.get(mat.get("name"))
                d["rgba"] = rgba
                b.geoms.append(d)
        for j in child_joints.get(link_name, []):
            c = make(j.find("child").get("link"))
            o = j.find("origin")
            c.pos = parse_xyz(o, "xyz")
            c.rot = rpy_to_mat(*parse_xyz(o, "rpy"))
            jt = j.get("type")
            jn = j.get("name")
            is_hand = jn not in BODY_JOINTS
            if jt in ("revolute", "continuous") and not (fixed_hands and is_hand):
                lim = j.find("limit")
                mim = j.find("mimic")
                c.joint = {
                    "name": jn,
                    "axis": parse_xyz(j.find("axis"), "xyz", "1 0 0"),
                    "range": (float(lim.get("lower")), float(lim.get("upper"))),
                    "effort": float(lim.get("effort")),
                    "velocity": float(lim.get("velocity")),
                    "mimic": None if mim is None else (mim.get("joint"), float(mim.get("multiplier", 1)),
                                                      float(mim.get("offset", 0))),
                }
            elif jt not in ("fixed", "revolute", "continuous"):
                raise ValueError(f"unsupported joint type {jt}")
            b.children.append(c)
        return b

    return make(root[0])


def fuse_static(b: Body):
    """Fuse every joint-less child into its parent (mass, COM, inertia, geoms); leave a site at its frame."""
    changed = True
    while changed:
        changed = False
        new_children = []
        for c in b.children:
            if c.joint is not None:
                new_children.append(c)
                continue
            changed = True
            p, R = c.pos, c.rot
            # inertia
            m1, m2 = b.mass, c.mass
            if m2 > 0:
                c2 = p + R @ c.com
                I2 = R @ c.inertia @ R.T
                m = m1 + m2
                com = (m1 * b.com + m2 * c2) / m
                def shift(I, mm, r):
                    return I + mm * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
                b.inertia = shift(b.inertia, m1, b.com - com) + shift(I2, m2, c2 - com)
                b.mass, b.com = m, com
            for g in c.geoms:
                g = dict(g)
                g["pos"] = p + R @ g["pos"]
                g["rot"] = R @ g["rot"]
                b.geoms.append(g)
            b.sites.append({"name": c.name, "pos": p.copy(), "rot": R.copy()})
            for s in c.sites:
                b.sites.append({"name": s["name"], "pos": p + R @ s["pos"], "rot": R @ s["rot"]})
            for gc in c.children:
                gc.pos = p + R @ gc.pos
                gc.rot = R @ gc.rot
                new_children.append(gc)
        b.children = new_children
    for c in b.children:
        fuse_static(c)


def iter_bodies(b: Body):
    yield b
    for c in b.children:
        yield from iter_bodies(c)

# ----------------------------------------------------------------------------- MJCF writer


def hand_joint_names(side):
    return [f"{side}_{HAND_SLOT_JOINT[s]}" for s in HAND_SLOTS]


def hand_actuator_names(side):
    return [f"{side}_hand_{s}" for s in HAND_SLOTS]


def write_mjcf(urdf: ET.Element, fixed_hands: bool, path: Path, model_name: str) -> dict:
    tree = urdf_to_tree(urdf, fixed_hands)
    fuse_static(tree)
    bodies = list(iter_bodies(tree))
    jmap = {b.joint["name"]: b.joint for b in bodies if b.joint}
    order = [b.joint["name"] for b in bodies if b.joint]
    body_order = [n for n in order if n in BODY_JOINTS]
    assert body_order == BODY_JOINTS, "URDF tree order != contract order"
    body_idx = {n: i for i, n in enumerate(BODY_JOINTS)}

    mj = ET.Element("mujoco", model=model_name)
    # no meshdir: mesh paths are relative to this file, which keeps <include> from other directories working
    ET.SubElement(mj, "compiler", angle="radian", autolimits="true", balanceinertia="false")
    ET.SubElement(mj, "option", timestep=str(SIM_DT), integrator="implicitfast")

    d = ET.SubElement(mj, "default")
    dd = ET.SubElement(d, "default", {"class": "g1"})
    ET.SubElement(dd, "joint", damping="0", frictionloss="0")
    dv = ET.SubElement(dd, "default", {"class": "visual"})
    ET.SubElement(dv, "geom", type="mesh", contype="0", conaffinity="0", group="2", density="0")
    dc = ET.SubElement(dd, "default", {"class": "collision"})
    # robot geoms: contype 2 / conaffinity 1  -> collide with world geoms (MuJoCo default 1/1),
    # never with each other (no self-collision).
    if fixed_hands:
        ET.SubElement(dc, "geom", group="3", contype="0", conaffinity="0", density="0", rgba="0.8 0.2 0.2 0.4")
    else:
        ET.SubElement(dc, "geom", group="3", contype="2", conaffinity="1", density="0", rgba="0.8 0.2 0.2 0.4")
    df = ET.SubElement(dd, "default", {"class": "foot"})
    ET.SubElement(df, "geom", group="3", contype="2", conaffinity="1", density="0", condim="3",
                  friction="1 0.005 0.0001", priority="1", rgba="0.2 0.8 0.2 0.6")
    dh = ET.SubElement(dd, "default", {"class": "hand"})
    ET.SubElement(dh, "joint", armature=str(HAND_ARMATURE), damping=str(HAND_DAMPING))
    ET.SubElement(dd, "site", size="0.01", rgba="1 0.5 0 0.6", group="4")

    # assets
    asset = ET.SubElement(mj, "asset")
    mesh_files = []
    for b in bodies:
        for g in b.geoms:
            if g["type"] == "mesh" and g["file"] not in mesh_files:
                mesh_files.append(g["file"])
    mesh_name = {}
    for f in mesh_files:
        n = Path(f).stem
        if n in mesh_name.values():
            n = f.replace("/", "_").rsplit(".", 1)[0]
        mesh_name[f] = n
        ET.SubElement(asset, "mesh", name=n, file="../meshes/" + f)

    wb = ET.SubElement(mj, "worldbody")
    home_z = [None]

    def emit(parent_el, b: Body, is_root=False):
        attrs = {"name": b.name, "childclass": "g1"} if is_root else {"name": b.name}
        if not is_root:
            attrs["pos"] = fmt(b.pos)
            q = mat_to_quat(b.rot)
            if not np.allclose(q, [1, 0, 0, 0], atol=1e-12):
                attrs["quat"] = fmt(q)
        else:
            attrs["pos"] = "0 0 0.8"
        el = ET.SubElement(parent_el, "body", attrs)
        if b.mass > 0:
            I = b.inertia
            ET.SubElement(el, "inertial", pos=fmt(b.com), mass=fmt(b.mass),
                          fullinertia=fmt([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]], 10))
        if is_root:
            ET.SubElement(el, "freejoint", name="floating_base_joint")
        if b.joint:
            j = b.joint
            ja = {"name": j["name"], "axis": fmt(j["axis"]), "range": fmt(j["range"])}
            if j["mimic"]:
                # Coupled (mimic) joint: its motion is fixed by the equality constraint to the driving joint.
                # revo2_description's distal ranges are inconsistent with its multipliers (e.g. middle:
                # 1.41 * 1.267 = 1.786 > 1.69297), which would stop the finger short of the vendor's 81 deg,
                # so the coupled joint is left unlimited here (see SOURCE.md).
                del ja["range"]
                ja["limited"] = "false"
            if j["name"] in body_idx:
                i = body_idx[j["name"]]
                ja["armature"] = fmt(ULTRA_ARMATURE[i])
                ja["actuatorfrcrange"] = fmt([-j["effort"], j["effort"]])
            else:
                ja["class"] = "hand"
            ET.SubElement(el, "joint", ja)
        for g in b.geoms:
            ga = {}
            is_foot = g["kind"] == "collision" and g["type"] == "sphere" and "ankle_roll" in g["link"]
            ga["class"] = "visual" if g["kind"] == "visual" else ("foot" if is_foot else "collision")
            if g["kind"] == "collision":
                ga["name"] = f"{g['link']}_col{sum(1 for x in el.findall('geom') if x.get('class') != 'visual')}"
            if is_foot:
                ga["name"] = f"{g['link'].split('_')[0]}_foot{sum(1 for x in el.findall('geom') if x.get('class') == 'foot')}"
            if np.linalg.norm(g["pos"]) > 0:
                ga["pos"] = fmt(g["pos"])
            q = mat_to_quat(g["rot"])
            if not np.allclose(q, [1, 0, 0, 0], atol=1e-12):
                ga["quat"] = fmt(q)
            if g["type"] == "mesh":
                ga["mesh"] = mesh_name[g["file"]]
                if g["kind"] == "collision":
                    ga["type"] = "mesh"
            else:
                ga["type"] = g["type"]
                ga["size"] = fmt(g["size"])
            if g["kind"] == "visual" and g.get("rgba"):
                ga["rgba"] = g["rgba"]
            ET.SubElement(el, "geom", ga)
        for s in b.sites:
            sa = {"name": s["name"], "pos": fmt(s["pos"])}
            q = mat_to_quat(s["rot"])
            if not np.allclose(q, [1, 0, 0, 0], atol=1e-12):
                sa["quat"] = fmt(q)
            ET.SubElement(el, "site", sa)
            if s["name"] == "d435_link":
                # camera looks along -z_cam: map z_cam=-x_link, y_cam=+z_link (x forward in URDF)
                Rc = s["rot"] @ np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
                ET.SubElement(el, "camera", name="head_d435", pos=fmt(s["pos"]), quat=fmt(mat_to_quat(Rc)),
                              fovy="42")
        for c in b.children:
            emit(el, c)
        return el

    emit(wb, tree, is_root=True)

    # mimic couplings -> joint equality constraints
    eq_list = []
    for n in order:
        mim = jmap[n]["mimic"]
        if mim:
            eq_list.append((n, mim))
    if eq_list:
        eq = ET.SubElement(mj, "equality")
        for n, (src, mult, off) in eq_list:
            ET.SubElement(eq, "joint", name=f"{n}_mimic", joint1=n, joint2=src,
                          polycoef=fmt([off, mult, 0, 0, 0]), solref="0.005 1")

    # actuators
    act = ET.SubElement(mj, "actuator")
    for j in BODY_JOINTS:
        e = jmap[j]["effort"]
        ET.SubElement(act, "motor", name=j, joint=j, ctrlrange=fmt([-e, e]))
    hand_info = {}
    if not fixed_hands:
        for side in ("left", "right"):
            for slot, an, jn in zip(HAND_SLOTS, hand_actuator_names(side), hand_joint_names(side)):
                j = jmap[jn]
                kp = j["effort"] / HAND_KP_ERR_AT_EFFORT
                ET.SubElement(act, "position", name=an, joint=jn, kp=fmt(kp), dampratio="1",
                              ctrlrange=fmt(j["range"]), forcerange=fmt([-j["effort"], j["effort"]]))
                coupled = [(n, m[1]) for n, m in eq_list if m[0] == jn]
                hand_info[an] = {"slot": slot, "joint": jn, "range": list(j["range"]), "effort": j["effort"],
                                 "velocity": j["velocity"], "kp": kp, "coupled": coupled}

    # sensors: unitree_mujoco layout first (jointpos*29, jointvel*29, torque*29, imu...), then extras
    sen = ET.SubElement(mj, "sensor")
    short = [j[:-6] for j in BODY_JOINTS]
    for j, s in zip(BODY_JOINTS, short):
        ET.SubElement(sen, "jointpos", name=f"{s}_pos", joint=j)
    for j, s in zip(BODY_JOINTS, short):
        ET.SubElement(sen, "jointvel", name=f"{s}_vel", joint=j)
    for j, s in zip(BODY_JOINTS, short):
        ET.SubElement(sen, "jointactuatorfrc", name=f"{s}_torque", joint=j)
    ET.SubElement(sen, "framequat", name="imu_quat", objtype="site", objname="imu_in_pelvis")
    ET.SubElement(sen, "gyro", name="imu_gyro", site="imu_in_pelvis")
    ET.SubElement(sen, "accelerometer", name="imu_acc", site="imu_in_pelvis")
    ET.SubElement(sen, "framepos", name="frame_pos", objtype="site", objname="imu_in_pelvis")
    ET.SubElement(sen, "framelinvel", name="frame_vel", objtype="site", objname="imu_in_pelvis")
    ET.SubElement(sen, "framequat", name="secondary_imu_quat", objtype="site", objname="imu_in_torso")
    ET.SubElement(sen, "gyro", name="secondary_imu_gyro", site="imu_in_torso")
    ET.SubElement(sen, "accelerometer", name="secondary_imu_acc", site="imu_in_torso")
    ET.SubElement(sen, "framepos", name="pelvis_pos", objtype="body", objname="pelvis")
    ET.SubElement(sen, "framequat", name="pelvis_quat", objtype="body", objname="pelvis")
    ET.SubElement(sen, "framelinvel", name="pelvis_linvel", objtype="body", objname="pelvis")
    ET.SubElement(sen, "frameangvel", name="pelvis_angvel", objtype="body", objname="pelvis")
    if not fixed_hands:
        for side in ("left", "right"):
            for an, jn in zip(hand_actuator_names(side), hand_joint_names(side)):
                ET.SubElement(sen, "jointpos", name=f"{an}_pos", joint=jn)

    ET.indent(mj, "  ")
    body_text = ET.tostring(mj, encoding="unicode")
    return {"xml": body_text, "order": order, "jmap": jmap, "hand_info": hand_info, "eq": eq_list,
            "mesh_files": mesh_files}


def add_keyframe(xml_text: str, xml_path: Path, order) -> tuple[str, dict]:
    """Compute the home pose (ULTRA default angles, feet resting on z=0) with MuJoCo and add <keyframe>."""
    import mujoco
    xml_path.write_text(xml_text)
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    d = mujoco.MjData(m)
    q = m.qpos0.copy()
    q[0:3] = [0, 0, 1.0]
    q[3:7] = [1, 0, 0, 0]
    for n, v in zip(BODY_JOINTS, HOME_QPOS):
        q[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] = v
    d.qpos[:] = q
    mujoco.mj_kinematics(m, d)
    foot = [g for g in range(m.ngeom) if "foot" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]
    assert len(foot) == 8, len(foot)
    zmin = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in foot)
    q[2] = 1.0 - zmin
    ctrl = np.zeros(m.nu)
    root = ET.fromstring(xml_text)
    kf = ET.SubElement(root, "keyframe")
    ET.SubElement(kf, "key", name="home", qpos=fmt(q, 8), ctrl=fmt(ctrl))
    ET.indent(root, "  ")
    return ET.tostring(root, encoding="unicode"), {"pelvis_z": float(q[2]), "qpos": q.tolist()}


HEADER = """<!-- GENERATED by g1-brainco-description/build_model.py - do not edit by hand.
     Unitree G1 29-DoF rev 1.0 + BrainCo Revo2 hands{extra}.
     Sources and every modelling decision: g1-brainco-description/SOURCE.md -->
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fetch", action="store_true", help="re-download pinned upstream sources + meshes first")
    ap.add_argument("--cache", type=Path, default=Path.home() / ".cache/hospital_g1_robot_sources")
    args = ap.parse_args()
    if args.fetch:
        fetch(args.cache)

    (OUT / "urdf").mkdir(parents=True, exist_ok=True)
    (OUT / "mjcf").mkdir(parents=True, exist_ok=True)

    urdf = build_urdf()
    hdr = "<?xml version=\"1.0\"?>\n" + HEADER.format(extra="")
    variants = {
        "g1_29dof_brainco.urdf": urdf,
        "g1_29dof_brainco_ros.urdf": urdf_variant_ros(urdf),
        "g1_29dof_brainco_ik.urdf": urdf_variant_ik(urdf),
    }
    for fn, t in variants.items():
        (OUT / "urdf" / fn).write_text(hdr + ET.tostring(t.getroot(), encoding="unicode") + "\n")

    # mesh existence
    missing = [m.get("filename") for m in urdf.getroot().iter("mesh")
               if not (OUT / "urdf" / m.get("filename")).resolve().exists()]
    if missing:
        raise SystemExit(f"missing meshes (run with --fetch): {missing[:5]}")

    full = write_mjcf(urdf.getroot(), False, OUT / "mjcf/g1_29dof_brainco.xml", "g1_29dof_brainco")
    fixed = write_mjcf(urdf.getroot(), True, OUT / "mjcf/g1_29dof_brainco_fixedhands.xml",
                       "g1_29dof_brainco_fixedhands")
    out_full = OUT / "mjcf/g1_29dof_brainco.xml"
    out_fixed = OUT / "mjcf/g1_29dof_brainco_fixedhands.xml"
    txt_full, home_full = add_keyframe(full["xml"], out_full, full["order"])
    txt_fixed, home_fixed = add_keyframe(fixed["xml"], out_fixed, fixed["order"])
    out_full.write_text(HEADER.format(extra=" (full: 29 body + 12 hand actuators)") + txt_full + "\n")
    out_fixed.write_text(HEADER.format(extra=" (hands welded; for RL / MJX)") + txt_fixed + "\n")

    scene = f"""{HEADER.format(extra=" - flat ground scene")}<mujoco model="g1_29dof_brainco_scene_flat">
  <include file="g1_29dof_brainco.xml"/>
  <statistic center="0 0 0.8" extent="1.6"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="140" elevation="-20" offwidth="1280" offheight="720"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
             markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <light pos="2 2 3" dir="-0.5 -0.5 -1" diffuse="0.4 0.4 0.4"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" contype="1" conaffinity="1"/>
  </worldbody>
</mujoco>
"""
    (OUT / "mjcf/scene_flat.xml").write_text(scene)

    # robot_spec.json (machine-readable facts for g1-whole-body-rl, the sims, g1_controller, teleop)
    import mujoco
    mf = mujoco.MjModel.from_xml_path(str(out_full))
    mx = mujoco.MjModel.from_xml_path(str(out_fixed))
    jm = full["jmap"]
    spec = {
        "generated_by": "g1-brainco-description/build_model.py",
        "robot": "Unitree G1 29-DoF rev 1.0 + BrainCo Revo2 (6 actuated DoF / hand)",
        "sim_dt": SIM_DT,
        "joint_names": BODY_JOINTS,
        "default_qpos": HOME_QPOS,
        "default_qpos_source": "ULTRA ec972c7 ultra/sim2sim_teacher.py default_dof_pos",
        "home_pelvis_height": home_full["pelvis_z"],
        "kp": ULTRA_KP, "kd": ULTRA_KD, "armature": ULTRA_ARMATURE,
        "gains_source": "ULTRA ec972c7 ultra/env/tasks/humanoid_g1.py (released-policy gains)",
        "effort_limit": [jm[j]["effort"] for j in BODY_JOINTS],
        "effort_limit_source": "unitree_ros da52948 g1_29dof_rev_1_0.urdf <limit effort>",
        "effort_limit_ultra": ULTRA_EFFORT,
        "velocity_limit": [jm[j]["velocity"] for j in BODY_JOINTS],
        "joint_range": [list(jm[j]["range"]) for j in BODY_JOINTS],
        "hands": {
            "slots": HAND_SLOTS,
            "normalisation": "q = u * range_upper (range_lower = 0); u = 0 open, 1 closed",
            "left": {k: v for k, v in full["hand_info"].items() if k.startswith("left")},
            "right": {k: v for k, v in full["hand_info"].items() if k.startswith("right")},
        },
        "mjcf_full": {
            "file": "mjcf/g1_29dof_brainco.xml", "nq": mf.nq, "nv": mf.nv, "nu": mf.nu,
            "body_joint_qposadr": [int(mf.jnt_qposadr[mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_JOINT, j)])
                                   for j in BODY_JOINTS],
            "body_joint_dofadr": [int(mf.jnt_dofadr[mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_JOINT, j)])
                                  for j in BODY_JOINTS],
            "actuators": [mujoco.mj_id2name(mf, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(mf.nu)],
            "note": "qpos is tree-ordered: left-hand joints sit between left_wrist_yaw and right_shoulder_pitch. "
                    "Index body joints by name / body_joint_qposadr, never by slicing 7:36.",
        },
        "mjcf_fixedhands": {
            "file": "mjcf/g1_29dof_brainco_fixedhands.xml", "nq": mx.nq, "nv": mx.nv, "nu": mx.nu,
            "note": "qpos = [free(7), 29 body joints in contract order]; actuators = 29 motors in contract order.",
        },
        "frames": {"imu": "imu_in_pelvis", "secondary_imu": "imu_in_torso", "lidar": "mid360_link",
                   "camera": "d435_link (MJCF camera head_d435)",
                   "hand_base": ["left_base_link", "right_base_link"]},
        "total_mass_full": float(sum(mf.body_mass)),
        "total_mass_fixedhands": float(sum(mx.body_mass)),
    }
    (OUT / "robot_spec.json").write_text(json.dumps(spec, indent=2) + "\n")

    manifest = {str(p.relative_to(MESH)): sha256(p) for p in sorted(MESH.rglob("*.STL"))}
    (SRC / "mesh_manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"wrote {OUT}: full nq={mf.nq} nu={mf.nu} mass={spec['total_mass_full']:.3f} kg; "
          f"fixedhands nq={mx.nq} nu={mx.nu} mass={spec['total_mass_fixedhands']:.3f} kg; "
          f"home pelvis z={home_full['pelvis_z']:.4f} m")


if __name__ == "__main__":
    main()
