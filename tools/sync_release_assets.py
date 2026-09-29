#!/usr/bin/env python3
"""Copy release tooling to sibling repositories.

Searches the parent directory for Git projects that contain tools/bump_version.py,
then overwrites that file and .github/workflows/release.yml with the versions
from this repository.
"""

from __future__ import annotations

import argparse
import pathlib
import shutil
from typing import Iterable, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--parent",
        type=pathlib.Path,
        help="Directory to scan for repositories (default: parent of this repo).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="List targets without copying files.",
    )
    return parser.parse_args()


def repo_root() -> pathlib.Path:
    # tools/ -> repo root
    return pathlib.Path(__file__).resolve().parent.parent


def source_files(root: pathlib.Path) -> Tuple[pathlib.Path, pathlib.Path]:
    bump = root / "tools" / "bump_version.py"
    release = root / ".github" / "workflows" / "release.yml"
    if not bump.exists():
        raise FileNotFoundError(f"Source bump_version.py not found at {bump}")
    if not release.exists():
        raise FileNotFoundError(f"Source release.yml not found at {release}")
    return bump, release


def iter_target_repos(parent: pathlib.Path, self_root: pathlib.Path) -> Iterable[pathlib.Path]:
    for child in parent.iterdir():
        if not child.is_dir():
            continue
        if child.resolve() == self_root:
            continue
        bump = child / "tools" / "bump_version.py"
        release = child / ".github" / "workflows" / "release.yml"
        if (child / ".git").exists() and bump.exists() and release.exists():
            yield child


def copy_assets(target: pathlib.Path, bump_src: pathlib.Path, release_src: pathlib.Path, dry_run: bool) -> None:
    bump_dst = target / "tools" / "bump_version.py"
    release_dst = target / ".github" / "workflows" / "release.yml"

    for src, dst in ((bump_src, bump_dst), (release_src, release_dst)):
        dst.parent.mkdir(parents=True, exist_ok=True)
        if dry_run:
            print(f"[DRY-RUN] Would copy {src} -> {dst}")
        else:
            shutil.copy2(src, dst)
            print(f"Copied {src} -> {dst}")


def main() -> None:
    args = parse_args()
    root = repo_root()
    parent = (args.parent or root.parent).resolve()

    bump_src, release_src = source_files(root)
    targets = list(iter_target_repos(parent, root))

    if not targets:
        print(f"No targets found under {parent}")
        return

    for target in targets:
        print(f"Updating {target}")
        copy_assets(target, bump_src, release_src, args.dry_run)


if __name__ == "__main__":
    main()
