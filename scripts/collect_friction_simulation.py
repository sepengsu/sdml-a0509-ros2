"""Run independent A0509 MuJoCo friction simulation without ROS2."""
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
for package in ("a0509_friction_sim", "a0509_dynamics_experiment", "mdpg_1s1_description"):
    sys.path.insert(0, str(root / "src" / package))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

from a0509_friction_sim.entry import main

if __name__ == "__main__":
    raise SystemExit(main())
