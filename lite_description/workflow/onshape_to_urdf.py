"""Export the URDF hub and the visual meshes of a robot from Onshape.

This stage runs onshape-to-robot on ``cad/`` and moves its output to ``urdf/<robot>.urdf``
and ``meshes/visual/``. An export costs about 1000 Onshape API requests.

Usage:
  lite-description-onshape-to-urdf lite_dummy --keep-assets
"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

from . import robot_model


def generate(robot_dir: Path, keep_assets: bool = False, convert: bool = False) -> list[Path]:
    """Export ``urdf/<robot>.urdf`` and ``meshes/visual/``.

    ``keep_assets`` keeps ``cad/assets/`` and ``cad/robot.pkl`` for collider editing.
    ``convert`` rebuilds from ``cad/robot.pkl`` instead of calling the Onshape API.
    """
    cad_dir = robot_dir / "cad"
    assets_dir = cad_dir / "assets"
    output_filename = json.loads((cad_dir / "config.json").read_text())["output_filename"]

    # onshape-to-robot reads the collider sources from cad/assets/.
    assets_dir.mkdir(exist_ok=True)
    for scad in (cad_dir / "scad").iterdir():
        shutil.copy(scad, assets_dir)

    binary = shutil.which("onshape-to-robot") or str(Path(sys.executable).parent / "onshape-to-robot")
    if not Path(binary).exists():
        raise FileNotFoundError(
            "onshape-to-robot ships with the CAD toolchain. Install it with `uv sync --extra cad`."
        )
    arguments = [binary, str(cad_dir)]
    if convert:
        # Do not add --save-pickle here. onshape-to-robot runs its save branch before its
        # convert branch, and the convert path builds no robot, so the two raise a NameError.
        arguments.append("--convert")
    elif keep_assets:
        # Save robot.pkl, so a later run can rebuild with --convert.
        arguments.append("--save-pickle")
    subprocess.run(arguments, check=True)

    # Replace the old meshes, so that a removed or renamed part leaves no stale mesh.
    visual_dir = robot_dir / "meshes" / "visual"
    shutil.rmtree(visual_dir, ignore_errors=True)
    shutil.copytree(assets_dir / "merged", visual_dir)
    hub = robot_dir / "urdf" / f"{robot_dir.name}.urdf"
    hub.parent.mkdir(exist_ok=True)
    produced = cad_dir / f"{output_filename}.urdf"
    hub.write_text(
        produced.read_text().replace("assets/merged/", "../meshes/visual/").replace("package://", "")
    )
    produced.unlink()

    if not keep_assets:
        shutil.rmtree(assets_dir)
        if convert:
            (cad_dir / "robot.pkl").unlink(missing_ok=True)
    return [hub]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Export the URDF hub and meshes of a robot from Onshape.")
    parser.add_argument("robot", help="Variant name, under lite_description/robots/.")
    parser.add_argument("--keep-assets", action="store_true", help="Keep cad/assets/ and cad/robot.pkl.")
    parser.add_argument("--convert", action="store_true", help="Rebuild from cad/robot.pkl, without the Onshape API.")
    args = parser.parse_args(argv)
    for path in generate(robot_model.resolve_robot_dir(args.robot), args.keep_assets, args.convert):
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
