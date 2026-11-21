# M0–M4 on `grid_16_trajectories`

This run uses the **completed legacy 16-trajectory campaign**, not the later
Fourier-T16 copy or `wide16_v3`. It is a simulator-to-simulator torque comparison.

## Protocol

- Source: `data/grid_16_trajectories`, seven tool/payload conditions.
- First complete trial of each condition and trajectory; deterministic repeats
  are excluded. Every fifth 100 Hz row is selected (20 Hz).
- Train: T5–T11 and T15, 31,766 rows. Validation: T12, 4,697 rows. Held-out
  test: T13, T14, and the legacy chirp T16, 15,701 rows. T1–T4 are diagnostic
  only (41,748 rows). No test or diagnostic row selects a checkpoint.
- Inputs: six positions, velocities, accelerations, gripper one-hot, and payload
  mass. Target: six MuJoCo inverse-dynamics torques, not measured motor torque.
- M0 is **not trained**. It is a nominal RBD whose tool/payload mass and inertia
  are set to 80% of the saved scene, while the arm remains unchanged. In Bare,
  M0 is exact by construction; its near-zero Bare error is not independent
  predictive validation.
- M1 direct MLP, M2 direct spline KAN, M3 nominal RBD + MLP residual, M4
  nominal RBD + spline KAN residual. All learned models share the same 22 inputs,
  data split, 60-epoch maximum, batch size 1024, patience 8, and seed 42.

## Held-out test results

RMSE and MAE pool all 15,701 test rows and six joints; units are N·m.

| Model | RMSE | MAE | Validation RMSE | Best epoch |
|---|---:|---:|---:|---:|
| M0 Nominal RBD | 0.4733 | 0.2255 | 0.5965 | — |
| M1 Direct MLP | 2.0484 | 1.2104 | 0.1673 | 59 |
| M2 Direct KAN | 15.6387 | 11.7874 | 2.8021 | 60 |
| M3 RBD + MLP | **0.0943** | **0.0406** | 0.0009 | 55 |
| M4 RBD + KAN | 0.1611 | 0.0773 | 0.0014 | 60 |

M3 is best in this held-out test. M2's high test error is visible across all
test trajectories and is greatest on T16 (22.149 N·m). Validation and test
errors differ substantially, so the validation scores should not be presented
as generalization scores. These results use the saved MuJoCo model to generate
both labels and the RBD baseline; they do not establish real-robot accuracy.

## Files

- Prepared data: `data/grid16_legacy_joint_dataset_m0_m4/`.
- Trained checkpoints: `data/grid16_legacy_joint_models_m0_m4/M1..M4/best.pt`.
- Metrics, per-condition CSV, prediction arrays, and figures:
  `data/grid16_legacy_joint_evaluation_m0_m4/`.

## Reproduction

Run from the repository root in PowerShell, choosing fresh output directory
names if these artifacts already exist.

```powershell
$py = '.\model\.venv\Scripts\python.exe'
& $py -m model.prepare_joint_dataset --source data/grid_16_trajectories --output data/grid16_legacy_joint_dataset_new --profile trajectory16_legacy --stride 5
foreach ($id in @('M1','M2','M3','M4')) {
    & $py -m model.train_joint_models --model $id --data data/grid16_legacy_joint_dataset_new --output data/grid16_legacy_joint_models_new --epochs 60 --batch-size 1024 --patience 8 --seed 42 --threads 1
}
& $py -m model.evaluate_joint_models --data data/grid16_legacy_joint_dataset_new --models data/grid16_legacy_joint_models_new --output data/grid16_legacy_joint_evaluation_new
& $py -m model.plot_joint_evaluation --data data/grid16_legacy_joint_dataset_new --models data/grid16_legacy_joint_models_new --evaluation data/grid16_legacy_joint_evaluation_new
```

Offline inference with the trained checkpoints for one state:

```powershell
& .\model\.venv\Scripts\python.exe -m model.predict_joint_models `
  --data data/grid16_legacy_joint_dataset_m0_m4 `
  --models data/grid16_legacy_joint_models_m0_m4 `
  --condition ag145_p05 `
  --q -0.5 -0.1 0.3 0 -0.2 0 `
  --dq 0 0 0 0 0 0 `
  --ddq 0 0 0 0 0 0
```

## Longer run: larger batch and Early Stopping

The same prepared `grid_16_trajectories` dataset and split were reused. The
maximum rose from 60 to **120 epochs**, and the batch size from 1024 to **2048**.
All four learned models used AdamW (learning rate 0.002, weight decay 1e-5),
seed 42, and one CPU thread. The Early Stopping rule monitors validation
normalized MSE. An improvement must exceed `min_delta=1e-6`; after 10 epochs
without such an improvement, training stops and saves the best checkpoint.
The held-out test split is never used to choose a checkpoint.

| Model | Epochs run / maximum | Best epoch | Stop reason | Previous test RMSE | Longer-run test RMSE |
|---|---:|---:|---|---:|---:|
| M0 Nominal RBD | — | — | no training | 0.4733 | 0.4733 |
| M1 Direct MLP | 79 / 120 | 69 | Early Stopping | 2.0484 | 2.5089 |
| M2 Direct KAN | 120 / 120 | 118 | epoch limit | 15.6387 | 19.9443 |
| M3 RBD + MLP | 120 / 120 | 111 | epoch limit | 0.0943 | 0.0938 |
| M4 RBD + KAN | 73 / 120 | 63 | Early Stopping | 0.1611 | 0.1594 |

The RMSE values pool 15,701 held-out rows and six joints [N·m]. Increasing
epochs and batch size did not improve every model: M1 and M2 became worse on
the held-out trajectories even though checkpoints were selected by validation
loss. M3 remains the lowest-error model in this simulator comparison. The
larger batch and epoch limit changed together, so this run does not isolate
which change caused each difference.

The new checkpoints and per-model training histories are in
`data/grid16_legacy_joint_models_m0_m4_e120_b2048_es10/`. Scores, predictions,
tables, and figures are in
`data/grid16_legacy_joint_evaluation_m0_m4_e120_b2048_es10/`.

The PowerShell commands used for this run, from the repository root:

```powershell
$py = '.\model\.venv\Scripts\python.exe'
$data = 'data/grid16_legacy_joint_dataset_m0_m4'
$models = 'data/grid16_legacy_joint_models_m0_m4_e120_b2048_es10'
$evaluation = 'data/grid16_legacy_joint_evaluation_m0_m4_e120_b2048_es10'
foreach ($id in @('M1','M2','M3','M4')) {
    & $py -m model.train_joint_models --model $id --data $data --output $models `
        --epochs 120 --batch-size 2048 --patience 10 --min-delta 0.000001 `
        --learning-rate 0.002 --seed 42 --threads 1
}
& $py -m model.evaluate_joint_models --data $data --models $models --output $evaluation
& $py -m model.plot_joint_evaluation --data $data --models $models --evaluation $evaluation
```
