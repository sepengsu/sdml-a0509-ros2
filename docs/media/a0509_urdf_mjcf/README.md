# A0509 URDF and MJCF slide images

- `a0509_urdf_vs_mjcf_16x9.png`: 2400 × 1350 comparison slide.
- `a0509_urdf.png`, `a0509_mjcf.png`: separate labeled model images.
- `a0509_urdf_transparent.png`, `a0509_mjcf_transparent.png`: transparent robot cutouts for slide layouts.
- `a0509_urdf_code.png`, `a0509_mjcf_code.png`: 2400 × 1350 syntax-highlighted code screenshots from the actual model files, with source line numbers.

Both views use the same six-joint pose, camera, and MuJoCo renderer. The URDF view follows `src/doosan-robot2/dsr_description2/urdf/a0509.urdf`; its `package://` Collada visual URLs are replaced **in a temporary render copy only** with corresponding same-named STL visual meshes from the repository's A0509 MuJoCo assets. The MJCF view follows `src/doosan-robot2/dsr_description2/mujoco_models/a0509/scene.xml`. Source files are not changed. The URDF image is a render of the URDF robot geometry, not an RViz screenshot. See `manifest.json` for source hashes and joint pose.

Regenerate in the `a0509-dynamics` Docker container from `/ros2_ws`. The
container shares the repository's `results` directory, so new files appear in
`results/ppt_a0509_models/` on Windows. Copy the five PNG files and manifest to
this `docs/media/a0509_urdf_mjcf/` directory when updating the committed slide
assets.

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
python3 src/a0509_dynamics_experiment/scripts/render_ppt_a0509.py
```

Regenerate the two code screenshots on Windows with Pillow and Pygments:

```powershell
.venv\Scripts\python.exe src\a0509_dynamics_experiment\scripts\render_ppt_a0509_code.py
```

The code screenshots show `a0509.urdf` lines 30–50 and the robot model in
`mujoco_models/a0509/a0509.xml` lines 24–42. The scene's `scene.xml` includes
this robot model. Long source lines are visually wrapped for slide readability;
the source files are unchanged.
