# A0509 MuJoCo dynamics trials (2026-11-10)

These are simulation results from the `ms` branch. No real A0509 was connected or moved.

| Trajectory | Trial | Dynamics rows | JointState rate in simulation time | Estimated missing rate |
| --- | --- | ---: | ---: | ---: |
| T1 | `trial_002` | 1,001 | 100.000 Hz | 0% |
| T2 | `trial_001` | 1,001 | 100.000 Hz | 0% |
| T3 | `trial_001` | 3,401 | 99.994 Hz | 0% |
| T4 | `trial_002` | 7,401 | 99.997 Hz | 0% |

Each trial contains the planned trajectory, `dynamics.csv`, metadata, comparison plots, and a ROS 2 Bag. T1 additionally contains the raw MuJoCo screen recording (`simulation.mp4`) and a cropped view (`simulation_closeup.mp4`). The videos show the simulated robot; T2–T4 were collected without video recording to reduce capture overhead.

`JointState.header.stamp` supplies the simulation clock used for the rates above. Bag storage timestamps and `wall_arrival_hz` use wall time. MuJoCo ran slower than wall time on this PC, so wall arrival frequency must not be interpreted as dropped simulator samples.

T1–T3 measured TCP paths had approximately 11.9, 10.1, and 10.6 mm RMS deviation from their plans in these runs. `qfrc_inverse` and `qfrc_bias` are reconstructed from measured joints using a separate MuJoCo model. `qfrc_actuator` is unavailable from the upstream ROS bridge and remains blank. T1–T4 dimensions are research examples, not verified real-robot paths.
