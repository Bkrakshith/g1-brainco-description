# g1-brainco-description: Unitree G1 29-DoF rev 1.0 + BrainCo Revo2 model

The robot model everything else builds on: the Unitree G1 29-DoF rev 1.0 with BrainCo Revo2 hands, as URDF and
MJCF with meshes and a `robot_spec.json`. `g1-whole-body-rl`, `g1-mujoco-sim`, `g1-isaac-sim`, the
`g1_description` package in `hospital-sim-ros2` and `g1-webcam-teleop` all read from here. Building and
checking the model needs nothing else; `check_model.py` can also compare against ULTRA's MJCF when
`g1-whole-body-rl` is checked out next door. The master repo `hospital-sim` clones all of these side by side
and provides `env.sh`, which activates the shared `.venv` with mujoco. Paths inside this repo are relative to
its root. The commands below assume you are in the master checkout.

The generated files are regenerated, never edited by hand. `SOURCE.md` lists the upstream commits,
licences and every modelling decision.

```
g1-brainco-description/
  build_model.py              regenerate everything below from sources/ + meshes/ (--fetch re-downloads pinned upstream)
  check_model.py              all model checks (exit 1 on FAIL)
  sources/                    vendored pinned upstream text files + LICENSEs + mesh_manifest.json
  g1_29dof_brainco/
    urdf/g1_29dof_brainco.urdf       full URDF (29 body + 12 actuated + 10 mimic hand joints), meshes ../meshes/...
    urdf/g1_29dof_brainco_ros.urdf   same, package://g1_description/meshes/... (robot_state_publisher, RViz)
    urdf/g1_29dof_brainco_ik.urdf    29 body joints, hands rigid (pinocchio / arm IK)
    mjcf/g1_29dof_brainco.xml        full MuJoCo model: 29 motors + 12 hand position actuators, 'home' key
    mjcf/g1_29dof_brainco_fixedhands.xml  hands welded (same mass), 29 motors: RL / MJX
    mjcf/scene_flat.xml              floor + lights + full model
    robot_spec.json                  joint order, home pose, kp/kd, limits, hand map, qpos addresses
    meshes/{g1,revo2_left_hand,revo2_right_hand}/*.STL
```

## Use

```bash
.venv/bin/python -m mujoco.viewer --mjcf g1-brainco-description/g1_29dof_brainco/mjcf/scene_flat.xml
.venv/bin/python g1-brainco-description/build_model.py            # after editing build_model.py (needs numpy + mujoco)
.venv/bin/python g1-brainco-description/build_model.py --fetch    # re-download pinned upstream sources and meshes
.venv/bin/python g1-brainco-description/check_model.py            # 27 PASS, 2 WARN expected (see below)
ros2 launch g1_description description.launch.py use_sim_time:=true    # after sourcing env.sh
ros2 launch g1_description view.launch.py                             # RViz + joint sliders
```

To embed the model in another scene (for example `g1-mujoco-sim`), either `<include file=".../mjcf/g1_29dof_brainco.xml"/>`
or use `<asset><model name="g1" file="..."/></asset>` with `<attach model="g1" body="pelvis" prefix=""/>`.
Mesh paths are relative to the robot file, so both methods work from any directory. With `attach`, set
`<option timestep="0.002" integrator="implicitfast"/>` in the parent scene.

## Things every consumer must know

- **Body joints and actuators.** Actuators 0–28 are the 29 body motors (torque, `ctrlrange` = ±effort) in contract
  order. Actuators 29–40 are the hands: `left_hand_{thumb,thumb_aux,index,middle,ring,pinky}`, then `right_hand_*`.
  They are `position` actuators whose ctrl is the joint angle in radians, so a normalised command `u` becomes
  `ctrl = u * ctrlrange_upper`.
- **Full-model qpos is in kinematic-tree order.** The 11 left-hand joints sit between `left_wrist_yaw` (qpos 28)
  and `right_shoulder_pitch` (qpos 40). A body tree cannot be ordered any other way. Index body joints through
  `robot_spec.json → mjcf_full.body_joint_qposadr` / `body_joint_dofadr`, or by joint name. Never slice `qpos[7:36]`.
  In `fixedhands`, `qpos[7:36]` is exactly the contract order.
- **Hand joint names**, as used in `/joint_states` and the URDF:
  - actuated: `<side>_thumb_proximal_joint` (thumb), `<side>_thumb_metacarpal_joint` (thumb_aux),
    `<side>_{index,middle,ring,pinky}_proximal_joint`
  - coupled: `<side>_*_distal_joint` are URDF mimic joints, which robot_state_publisher computes.
- **Frames**, as URDF links and MJCF sites: `pelvis` (root), `imu_in_pelvis` (LowState IMU), `imu_in_torso`,
  `mid360_link`, `d435_link` (MJCF camera `head_d435`), `left_base_link`/`right_base_link` (hand base) and
  `*_tip_link` (fingertips).
- **The MID-360 is mounted upside down**, per Unitree's latest URDF: the z-axis of `mid360_link` points down.
  Simulated point clouds must be expressed in that frame, as the real Livox driver does.
- **Effort limits come from the latest Unitree URDF**: ankle and waist roll/pitch are 35 N·m. ULTRA and
  unitree_rl_gym used 50. `robot_spec.json` has both.
- **Default pose, gains and armature** are ULTRA's released values, stored in `robot_spec.json`.
- **Zero-action PD hold with ULTRA kp/kd falls after about 0.83 s.** Unitree's own MJCF does the same (0.88 s),
  because ankle stiffness 2 × 28.5 N·m/rad is far below m·g·h ≈ 230 N·m/rad. Standing needs a balancing policy.
  A stiff test PD (500/10 legs) stands indefinitely, so the contact and mass model is sound.
