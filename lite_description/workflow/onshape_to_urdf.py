"""Stage 1 of the converter: export the raw URDF and meshes from Onshape CAD.

Wraps the external ``onshape-to-robot`` tool and normalizes its output into the
robots/<robot>/ layout:

    robots/<robot>/
      cad/config.json          (input: Onshape doc + export options)
      cad/scad/                (input: custom collider sources)
      urdf/<robot>.urdf        (output: the flat URDF, the committed kinematic hub)
      meshes/visual/*.stl      (output: merged visual meshes, one copy)

The flat URDF is the committed, cacheable hub. The ``urdf`` finalize stage then welds and
harmonizes it, and the mjcf and xacro stages derive from it. With a committed hub, work on
physics.json or ros2_control.json never re-hits the Onshape API, which costs about 1000
requests per export.
"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

from . import robot_model


def export(robot_dir: Path, keep_assets: bool = False, convert: bool = False) -> Path:
    """Run onshape-to-robot for ``robot_dir`` and place the outputs in the cad/ layout.

    Args:
        robot_dir: The ``robots/<robot>/`` directory to export.
        keep_assets: Keep ``cad/assets/`` and ``cad/robot.pkl`` after the run.
        convert: Rebuild from the local ``robot.pkl`` instead of calling the Onshape API.

    Returns:
        Path to the written ``urdf/<robot>.urdf`` hub.
    """
    robot = robot_dir.name
    cad_dir = robot_dir / "cad"
    config_path = cad_dir / "config.json"
    if not config_path.exists():
        raise FileNotFoundError(f"Config file {config_path} does not exist!")

    output_filename = json.loads(config_path.read_text()).get("output_filename", robot)
    assets_dir = cad_dir / "assets"
    scad_dir = cad_dir / "scad"

    # onshape-to-robot reads collider sources from <workdir>/assets/.
    if scad_dir.exists():
        assets_dir.mkdir(exist_ok=True)
        for scad_file in scad_dir.iterdir():
            shutil.copy(scad_file, assets_dir / scad_file.name)

    binary = shutil.which("onshape-to-robot") or str(Path(sys.executable).parent / "onshape-to-robot")
    if not Path(binary).exists():
        raise FileNotFoundError(
            "The onshape-to-robot CLI was not found. It ships with the CAD toolchain rather than "
            "the base install; install it with `uv sync --extra cad`."
        )
    arguments = [binary, str(cad_dir)]
    if convert:
        # Offline: reload the existing robot.pkl. --save-pickle must NOT be combined with
        # --convert, because onshape-to-robot runs its save branch before its convert
        # branch and no robot is built on the convert path, so the two together raise a
        # NameError.
        arguments.append("--convert")
    elif keep_assets:
        # Build path: persist robot.pkl so later runs can re-derive offline via --convert.
        arguments.append("--save-pickle")
    subprocess.run(arguments, check=True)

    # Merged visual meshes -> the single meshes/visual/ copy.
    merged = assets_dir / "merged"
    if merged.exists():
        visual_dir = robot_dir / "meshes" / "visual"
        visual_dir.mkdir(parents=True, exist_ok=True)
        shutil.copytree(merged, visual_dir, dirs_exist_ok=True)

    # Flat URDF -> urdf/<robot>.urdf, with mesh refs pointed at meshes/visual/ and the ROS
    # package:// scheme stripped. The finalize stage then harmonizes it in place.
    hub_urdf = robot_dir / "urdf" / f"{robot}.urdf"
    hub_urdf.parent.mkdir(parents=True, exist_ok=True)
    produced_urdf = cad_dir / f"{output_filename}.urdf"
    hub_urdf.write_text(
        produced_urdf.read_text()
        .replace("assets/merged/", "../meshes/visual/")
        .replace("package://", "")
    )
    produced_urdf.unlink(missing_ok=True)

    if not keep_assets:
        shutil.rmtree(assets_dir, ignore_errors=True)
        if convert:
            (cad_dir / "robot.pkl").unlink(missing_ok=True)

    print(f"[onshape] export -> {hub_urdf}")
    return hub_urdf


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Export a raw URDF from Onshape CAD.")
    parser.add_argument("robot", help="Variant name (under lite_description/robots/) or a path to a robot dir.")
    parser.add_argument("--keep-assets", action="store_true", help="Keep the assets dir and robot.pkl.")
    parser.add_argument("--convert", action="store_true", help="Convert from a local robot.pkl.")
    args = parser.parse_args(argv)
    export(robot_model.resolve_robot_dir(args.robot),
           keep_assets=args.keep_assets, convert=args.convert)


if __name__ == "__main__":
    main()
