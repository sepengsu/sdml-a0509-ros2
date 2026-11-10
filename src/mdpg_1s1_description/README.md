# mDPG_1S1 pneumatic suction gripper: ROS2 description

Created from the uploaded `mDPG_1S1 CS (1).stp` (Autodesk Inventor 2024 STEP/AP214).

## Contents
- `meshes/*.stl`: six independent solids from five named STEP assembly components
- `urdf/mdpg_1s1_macro.xacro`: fixed mounting joint, six visual meshes, approximate collisions, nominal TCP
- `urdf/mdpg_1s1_standalone.urdf.xacro`: independent RViz preview
- `launch/view_mdpg_1s1.launch.py`: preview launch
- `config/mesh_manifest.json`: STEP-derived dimensions and coordinate transform
- `config/dynamics_values_to_measure.yaml`: unfilled physical quantities

The CAD file has a pump cover, LED indicator, bottom cover, another solid (`솔리드33`), and a two-solid `air_pad`.
It is not modeled as a two-finger gripper. Vacuum actuation, suction attachment and physical I/O are NOT implemented by the STEP file.

## Installation

Copy this folder to `~/sdml-doosan-robot/src/mdpg_1s1_description` (or your ROS2 workspace's `src/`).

```bash
cd ~/sdml-doosan-robot
colcon build --symlink-install --packages-select mdpg_1s1_description
source install/setup.bash
ros2 launch mdpg_1s1_description view_mdpg_1s1.launch.py
```

## Mount to A0509

In the active A0509 robot-description xacro (NOT an unrelated copy), remove/condition the existing AG145 macro and add:

```xml
<xacro:include filename="$(find mdpg_1s1_description)/urdf/mdpg_1s1_macro.xacro"/>
<xacro:mdpg_1s1_gripper prefix="mdpg_" parent="link_6">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:mdpg_1s1_gripper>
```

**Mount origin warning:** ROS mounting pose `xyz/rpy=0` above is ONLY an illustrative default, NOT a verified interface. The mesh was centered on X around 8.2339 mm and reoriented so the air pad points to -Z; the base origin was inferred from the positive-Y face of the STEP housing. Verify flange bolt center, tooling adapter, pitch, and orientation. The `mdpg_tcp` frame is at -90 mm from this approximate base: check on real gripper.

**Mesh QA:** The `pump_cover`, `bottom_cover` and `led_indicator` STL meshes are not watertight after triangulation, so treat them as visual meshes only. The supplied collisions are simple primitives.

**For MuJoCo/dynamics**: add measured mass, center of mass, inertia tensor for each genuinely moving link/assembly (at least the complete gripper+adapter), plus contact and vacuum/IO dynamics. The STEP has no validated masses or material properties. `collision` shapes are approximate and not meant for quantitative contact validation.

**Repository integration caveat:** In `c0sogi/sdml-doosan-robot` branch `alchemist`, `src/a0509_mujoco_sim/urdf/a0509.urdf.xacro` includes AG145, but the MuJoCo launch actually constructs `robot_description` from `dsr_description2/xacro/a0509.urdf.xacro` and loads `dsr_description2/mujoco_models/a0509/scene.xml`. For real/RViz launch it may use `dsr_moveit_config_a0509/config/a0509.urdf.xacro`. Changing one unused duplicate Xacro is not sufficient. Determine the launch-specific active path before switching tools. Do not mount both grippers simultaneously.

Modeling scope: static rigid gripper **visual geometry only**, not a ROS2 controller or validated digital twin.

## A0509 integration in this repository

The active A0509 tool Xacro is `sdml_a0509_description/urdf/a0509_with_gripper.urdf.xacro`. The MuJoCo, RViz-only and RT real-robot launches use it with `gripper_type:=none|ag145|mdpg_1s1`. The mDPG macro is fixed to `link_6`; `mdpg_tcp` is a CAD-bounds estimate at -90 mm, **not a calibrated TCP**. AG145 and mDPG are mutually exclusive in the Xacro and MuJoCo scene.

No mDPG mounting transform is provided. `mdpg_mount_xyz` (m) and `mdpg_mount_rpy` (rad) must be supplied explicitly. The identity pose used in automated visualization tests is a test fixture, not a measured mounting pose. The launch rejects missing mDPG mounting values.

```bash
colcon build --symlink-install --packages-select mdpg_1s1_description sdml_a0509_description a0509_mujoco_sim sdml_a0509_bringup a0509_dynamics_experiment
source install/setup.bash

# Replace each bracketed value with a measured link_6-to-tool transform.
ros2 launch sdml_a0509_bringup rviz_only.launch.py gripper_type:=mdpg_1s1 \
  mdpg_mount_xyz:="<X_M> <Y_M> <Z_M>" mdpg_mount_rpy:="<R_RAD> <P_RAD> <Y_RAD>"
ros2 launch a0509_mujoco_sim mujoco_sim.launch.py gripper_type:=mdpg_1s1 \
  mdpg_mount_xyz:="<X_M> <Y_M> <Z_M>" mdpg_mount_rpy:="<R_RAD> <P_RAD> <Y_RAD>"

# Existing AG145 tool remains selectable without mDPG parameters.
ros2 launch a0509_mujoco_sim mujoco_sim.launch.py gripper_type:=ag145
```

The mDPG MuJoCo scene is **visual-only**: its six CAD meshes have no inertial or contact contribution and the arm retains six DoF. `run_experiment --preview --gripper-type mdpg_1s1` is supported with the explicit mount values. mDPG trajectory dynamics recording and real motion are blocked until the TODO fields in `config/dynamics_values_to_measure.yaml` are measured and validated. `a0509_real.launch.py gripper_type:=mdpg_1s1` uses the RT arm/state launch and omits AG145 control and pick-and-place nodes; it does not implement suction I/O.
