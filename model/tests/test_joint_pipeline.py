"""Exercise the shared train/evaluate path with a small synthetic dataset."""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from model.evaluate_joint_models import evaluate
from model.prepare_joint_dataset import CONDITIONS
from model.train_joint_models import train


ROOT = Path(__file__).resolve().parents[2]


class JointPipelineTests(unittest.TestCase):
    def test_four_models_and_baseline_share_one_dataset(self):
        with tempfile.TemporaryDirectory(prefix="joint_pipeline_test_", dir=ROOT) as folder:
            base = Path(folder)
            data_root = base / "data"
            data_root.mkdir()
            features, reference, nominal, condition, trajectory, split = [], [], [], [], [], []
            for ci, name in enumerate(CONDITIONS):
                for code, t in ((0, 1), (1, 13), (2, 15)):
                    for sample in range(3):
                        row = np.zeros(22, dtype=np.float32)
                        row[:18] = 0.1 * (ci + sample + code)
                        row[18 + (0 if name == "bare_p0" else 1 if name.startswith("ag145") else 2)] = 1
                        row[21] = 0 if name == "bare_p0" else 0.5
                        tau = np.full(6, 2 + ci + sample + code, dtype=np.float32)
                        features.append(row)
                        reference.append(tau)
                        nominal.append(tau * 0.8)
                        condition.append(ci)
                        trajectory.append(t)
                        split.append(code)
            path = data_root / "dataset.npz"
            np.savez_compressed(path, features=np.stack(features), tau_reference=np.stack(reference),
                                tau_nominal=np.stack(nominal), condition=np.asarray(condition),
                                trajectory=np.asarray(trajectory), split=np.asarray(split))
            manifest = {"profile": "wide16_v3", "source": str(base / "source"),
                        "conditions": CONDITIONS, "nominal_inertial_scale": 0.8,
                        "dataset_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                        "split_codes": {"train": 0, "validation": 1, "test": 2},
                        "rows": {"train": 21, "validation": 21, "test": 21},
                        "split_policy": {"train": ["T1"], "validation": ["T13"], "test": ["T15"]}}
            (data_root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            models = base / "models"
            for model_id in ("M1", "M2", "M3", "M4"):
                report = train(model_id, data_root, models, epochs=1, batch_size=16, patience=1)
                self.assertEqual(report["best_epoch"], 1)
            result = evaluate(data_root, models, base / "evaluation")
            self.assertEqual(set(result["scores"]["test"]["all"]), {"M0", "M1", "M2", "M3", "M4"})
            self.assertEqual(result["scores"]["test"]["all"]["M0"]["samples"], 21)
            self.assertTrue((base / "evaluation" / "comparison.csv").is_file())


if __name__ == "__main__":
    unittest.main()
