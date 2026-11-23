"""Source-workspace entry for the same implementation as ROS2 validate_friction."""
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
for package in ("sim_real_gap_getting", "a0509_dynamics_experiment", "mdpg_1s1_description"):
    sys.path.insert(0, str(root / "src" / package))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

from sim_real_gap_getting.friction_engine.entry import main

if __name__ == "__main__":
    raise SystemExit(main())
