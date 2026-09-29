"""One converter, three formats: CAD -> URDF + MJCF + xacro.

Runs the generation stages for a ``robots/<robot>/`` directory. The committed flat URDF
(``urdf/<robot>.urdf`` plus ``meshes/visual/``) is the kinematic hub. The MJCF and the
xacro both derive from it, so the three formats share one kinematic origin and cannot
drift apart.

Stages, each runnable alone with ``--only``. The three format stages are sibling modules
that each expose ``generate(robot_dir) -> list[Path]``:

  onshape  onshape_to_urdf : Onshape CAD -> urdf/<robot>.urdf + meshes/visual/
  urdf     finalize_urdf   : finalize the hub (weld, harmonize effort, mesh paths)
  mjcf     urdf_to_mjcf    : MuJoCo compile, post-process, <option>
  xacro    urdf_to_xacro   : description macro (+base_link), ros2_control, assembly
  package                  : register the robot in the repo-root CMakeLists

The onshape stage hits the Onshape API about 1000 times, so it is skipped whenever the
URDF hub is already committed. Pass ``--force`` to re-export anyway.

Usage:
  lite-description-generate lite_dummy
  lite-description-generate lite_dummy --only mjcf,xacro
  lite-description-generate ./lite_description/robots/lite_dummy --force
"""

import argparse
from pathlib import Path
import re

from . import finalize_urdf, onshape_to_urdf, robot_model, urdf_to_mjcf, urdf_to_xacro

STAGES = ("onshape", "urdf", "mjcf", "xacro", "package")


def _report(stage: str, written: list[Path]) -> None:
    for path in written:
        print(f"[{stage}] wrote {path}")


def register_in_cmake(robot_dir: Path) -> None:
    """Add the robot to the repo-root CMakeLists ROBOTS list, if it is not there."""
    robot = robot_dir.name
    cmake = robot_dir.resolve().parents[2] / "CMakeLists.txt"  # <repo>/lite_description/robots/<robot>/
    if not cmake.exists():
        print(f"[package] no CMakeLists at {cmake}; skipping")
        return
    text = cmake.read_text()
    match = re.search(r"set\(ROBOTS(?P<body>.*?)\)", text, re.DOTALL)
    if not match:
        print("[package] ROBOTS block not found; skipping")
        return
    listed = match.group("body").split()
    if robot in listed:
        return
    block = "set(ROBOTS\n  " + "\n  ".join(sorted([*listed, robot])) + "\n)"
    cmake.write_text(text[: match.start()] + block + text[match.end():])
    print(f"[package] registered {robot} in {cmake.name}")


def generate(
    robot: str,
    only: list[str] | None = None,
    force: bool = False,
    package: str = robot_model.DEFAULT_PACKAGE,
) -> None:
    """Run the requested stages for one robot."""
    unknown = sorted(set(only or ()) - set(STAGES))
    if unknown:
        raise ValueError(f"Unknown stage(s) {unknown}. Valid: {list(STAGES)}")

    robot_dir = robot_model.resolve_robot_dir(robot)
    stages = list(only) if only else list(STAGES)

    hub = robot_dir / "urdf" / f"{robot_dir.name}.urdf"
    # Skip the expensive export when the hub is committed, unless it was asked for by
    # name (--only onshape) or forced. `only is None` implies stages == STAGES, which
    # always contains "onshape".
    if only is None and hub.exists() and not force:
        print(f"[onshape] skipped (URDF hub present at {hub}); pass --force to re-export")
        stages.remove("onshape")

    for stage in stages:
        if stage == "onshape":
            print(f"[onshape] exporting {robot_dir.name} from Onshape (this hits the API)...")
            onshape_to_urdf.export(robot_dir)
        elif stage == "urdf":
            _report("urdf", finalize_urdf.generate(robot_dir))
        elif stage == "mjcf":
            _report("mjcf", urdf_to_mjcf.generate(robot_dir))
        elif stage == "xacro":
            _report("xacro", urdf_to_xacro.generate(robot_dir, package=package))
        elif stage == "package":
            register_in_cmake(robot_dir)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate URDF + MJCF + xacro for a robot from CAD.")
    parser.add_argument("robot", help="Variant name (under lite_description/robots/) or a path to a robot dir.")
    parser.add_argument("--force", action="store_true",
                        help="Re-run the Onshape export even if the URDF hub is committed.")
    parser.add_argument("--only", help="Comma-separated subset of stages, e.g. 'mjcf,xacro'.")
    parser.add_argument("--package", default=robot_model.DEFAULT_PACKAGE,
                        help="Owning ament package name.")
    args = parser.parse_args(argv)

    only = [s.strip() for s in args.only.split(",")] if args.only else None
    generate(args.robot, only=only, force=args.force, package=args.package)


if __name__ == "__main__":
    main()
