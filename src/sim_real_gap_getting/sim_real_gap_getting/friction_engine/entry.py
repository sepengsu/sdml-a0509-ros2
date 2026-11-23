"""ROS2 console entry point; simulation only, with no robot or ROS topic IO."""
import os


def main(argv=None):
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    from .cli import main as run
    try:
        return run(argv)
    except KeyboardInterrupt:
        print("Stopped. Rerun the same stage to resume verified completed work.", flush=True)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
