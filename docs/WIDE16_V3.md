# Wide16 v3: wide repeated MuJoCo excitation

This is a **new simulation-only dataset**. It does not modify or combine with
`data/grid_16_fourier_t16_20261111` and is not an approved real-robot motion plan.

## Design

- One A0509 per trajectory, T1–T16, in a shared 4×4 MuJoCo scene.
- Sixteen distinct joint patterns: arm sweeps, coupled arm/wrist sweeps, shifted
  starting postures, and a six-joint two-frequency pattern.
- Each joint component completes five or six full oscillations; T16 also has a
  ten-cycle harmonic. A 60 s smooth envelope starts and ends every path at rest.
- 450 s motion plus 1 s hold at each end, sampled at 100 Hz: 45,201 rows per
  trajectory. The default 7 conditions × 3 repeats × 16 paths create 336 trials
  and 15,187,536 planned rows.
- The centered starting postures and wider amplitudes approach the tested
  collision-free envelope while retaining joint-limit margin. Arm joint
  components reach 32–84 degrees; wrist components reach 31–49 degrees.
  Coupled components use different cycle
  counts to cover more joint combinations than a single in-phase sine.
- T1–T12 are train, T13–T14 validation, and T15–T16 test. Repeats of a given
  path stay in the same split.

| Path | Excited joints | Pattern |
| --- | --- | --- |
| T1 | J1, J2 | broad base turn with shoulder offset |
| T2 | J2 | shoulder sweep |
| T3 | J3 | elbow sweep |
| T4 | J1, J2, J4 | base/shoulder with wrist rotation |
| T5 | J2, J5 | shoulder with wrist bend |
| T6 | J3, J6 | elbow with flange rotation |
| T7 | J1, J2 | coupled base/shoulder at different phase |
| T8 | J2, J3 | coupled shoulder/elbow |
| T9 | J1, J2, J3 | three-arm-joint sweep |
| T10 | J1, J2, J4 | three-joint mixed sweep |
| T11 | J2, J3, J5 | arm/wrist mixed sweep |
| T12 | J1, J2, J3, J6 | four-joint mixed sweep |
| T13 | J1, J2, J5 | shifted starting posture A |
| T14 | J2, J3, J4 | shifted starting posture B |
| T15 | J1–J6 | six-joint phase-shifted sweep |
| T16 | J1–J6 | six-joint fundamental plus harmonic |

The design is in
`src/a0509_dynamics_experiment/a0509_dynamics_experiment/wide16_trajectory.py`.
The original generator and its optimized Fourier T16 remain separate.
Trial metadata records `trajectory_config.profile = wide16_v3`; the
`trajectory_design_origin` label refers to the earlier v2 design lineage.

For Bare, the exact 100 Hz kinematic preflight measured maximum TCP distance
from each path's start of **346–996 mm** (median **730 mm**). The prior Fourier
campaign measured **11–156 mm** (median **58 mm**) by the same definition.
This is a broad, collision-free candidate within the specified joint limits;
it is not a proof of the robot's global maximum reachable distance.

## Local static preflight

```powershell
cd D:\Code\sdml-doosan-robot
.\.venv\Scripts\python.exe -m scripts.validate_wide16_design
```

The report is `results/wide16_v3_design_preflight.json` and its CSV companion.
It checks every 100 Hz target in all seven saved MuJoCo condition models for
joint limits and contacts, then reports TCP range. It does not measure dynamic
tracking or controller torque saturation.

The AG145/0.5 kg MuJoCo pilot passed for T1, T5, T15 and T16:
maximum tracking error **0.000603 rad**, maximum commanded torque **85.9 Nm**,
zero torque-limit samples and zero cross-robot contacts. Its report is
`data/grid_16_wide_v3/pilot_report.json`. The full run still applies the same
quality gate to every 16-robot scene.

## Collect the new dataset

The collector runs headless MuJoCo in an isolated Docker container. It cannot
command a physical robot. The pilot checks tracking error, torque saturation,
and inter-robot contact before the full collection can start.

```powershell
cd D:\Code\sdml-doosan-robot
.\scripts\play_grid_dataset.ps1 -Campaign wide16 -Pilot -UseCadFaceCenterFixture
docker logs -f a0509-grid-grid_16_wide_v3
.\scripts\play_grid_dataset.ps1 -Campaign wide16 -Status -ResultsName grid_16_wide_v3 -Trials 1
.\scripts\play_grid_dataset.ps1 -Campaign wide16 -Play -UseCadFaceCenterFixture -Trials 1
docker logs -f a0509-grid-grid_16_wide_v3
.\scripts\play_grid_dataset.ps1 -Campaign wide16 -Status -ResultsName grid_16_wide_v3 -Trials 1
```

The `-UseCadFaceCenterFixture` mDPG mount is only a visual and simulation
fixture; its real adapter transform has not been measured. Supply the measured
`-MdpgMountXyz` and `-MdpgMountRpy` arrays instead if they become available,
using a **new** `-ResultsName` to keep mounts and datasets distinct. The pilot
and full run must use the same mount declaration.

Results are written under `data/grid_16_wide_v3`. The full run automatically
skips complete scenes; the pilot or full run can be repeated after interruption.
Acceptance requires maximum tracking error ≤ 0.05 rad, zero torque-limit
samples, and zero cross-robot contacts. The old RBD/PKAN fit must be retrained
or reevaluated on the new dataset; old metrics do not transfer to this profile.

The faster collector keeps the selected repeats, 100 Hz samples, trajectory and
dynamics CSVs, metadata, and the shared ROS bag. Wide16 omits each trial's
redundant `trajectory.json` and per-trial PNG/HTML plots. The trajectory can be
regenerated from the config in `metadata.json`; plots can be made from the CSVs
after collection. Each scene runs in a fresh Python process to release memory
before the next scene. After a collector code change, rerun `-Pilot` so its
fingerprint matches, then `-Play` resumes completed scenes.

For the initial faster pass, collect only `trial_001` for each condition:

```powershell
.\scripts\play_grid_dataset.ps1 -Campaign wide16 -Play -UseCadFaceCenterFixture -Trials 1
.\scripts\play_grid_dataset.ps1 -Campaign wide16 -Status -Trials 1
```

This targets 7 scenes and 112 trajectory trials. Existing `trial_002` and
`trial_003` files stay on disk but are excluded from the `-Trials 1` status.
`model/prepare_joint_dataset.py` also selects the first complete trial only.

For ten repeats of the same T1–T16 paths in all seven conditions, use:

```powershell
cd D:\Code\sdml-doosan-robot
.\scripts\run_wide16_10.ps1
```

This schedules 70 scenes and 1,120 trajectory trials (50,625,120 planned rows).
The launcher runs one scene per fresh Python process and skips completed scenes.
It caps the collector at 8 GB RAM or 75% of Docker's available RAM, whichever is
lower, with no extra swap and four CPUs. Docker must have enough memory to leave
at least 25% free and still provide 4 GB to the collector. If a scene reaches
the cap, the process can exit with code 137. After a code change, rerun the
wrapper; it refreshes the pilot fingerprint and resumes completed scenes.
If memory is still insufficient, increase Docker Desktop memory or pass a
higher `-MemoryGb` value. The launcher still caps that value at 75% of Docker RAM.
Keep ample disk space
for the CSVs and bags. At the size of an existing Bare scene, the full campaign
would take roughly 170 GB; other scenes may differ. The collector stops before
a new scene if less than 10 GiB remains. The repeated paths are deterministic, so additional
repeats do not broaden the set of joint configurations.

Wide16 writes each lane's dynamics CSV in 1,000-sample chunks to a temporary
file. After the scene passes its quality gate, the temporary file becomes
`dynamics.csv`. This bounds the recorder's in-memory rows while preserving the
full 100 Hz dataset. An interrupted scene is recollected on the next run.

To render an **actual recorded Bare** 4×4 MuJoCo replay as soon as the first
16-robot scene finishes, run this in a separate PowerShell window:

```powershell
cd D:\Code\sdml-doosan-robot
.\scripts\watch_wide16_bare_video.ps1
```

It waits for all 16 validated `bare_p0/trial_001/dynamics.csv` files, then
creates `results/wide16_v3_bare_recorded_mujoco.mp4`. The collection continues
through the remaining conditions while the video is rendered.
