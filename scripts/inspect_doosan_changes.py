"""Preview the copied Doosan overlay without extracting or overwriting files."""

from __future__ import annotations

import argparse
from pathlib import Path, PurePosixPath
import tarfile


def inspect(archive: Path, target: Path) -> list[tuple[str, str]]:
    rows = []
    with tarfile.open(archive, "r:gz") as bundle:
        for member in bundle.getmembers():
            path = PurePosixPath(member.name)
            if not member.isfile() or path.is_absolute() or ".." in path.parts:
                raise ValueError(f"unsafe archive member: {member.name}")
            existing = target.joinpath(*path.parts)
            source = bundle.extractfile(member)
            if source is None:
                raise ValueError(f"cannot read archive member: {member.name}")
            body = source.read()
            status = ("new" if not existing.exists() else
                      "same" if existing.read_bytes() == body else "overwrite")
            rows.append((status, member.name))
    return rows


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    repo = Path(__file__).resolve().parents[1]
    parser.add_argument("--archive", type=Path, default=repo / "doosan_changes_export.tar.gz")
    parser.add_argument("--target", type=Path, default=repo / "src/doosan-robot2")
    args = parser.parse_args(argv)
    for status, name in inspect(args.archive, args.target):
        print(f"{status:9} {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
