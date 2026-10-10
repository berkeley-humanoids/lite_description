"""Generate the URDF, MJCF and xacro of a robot from its Onshape CAD.

The flat URDF (``urdf/<robot>.urdf``) is the kinematic hub. The MJCF and the xacro derive
from it, so the three formats cannot drift apart. The stages run in this order:

  onshape  Onshape CAD -> urdf/<robot>.urdf and meshes/visual/
  urdf     finalize the hub: effort limits, mesh paths, banner
  mjcf     hub -> mjcf/<robot>.xml
  xacro    hub -> xacro/<robot>.*.xacro, with base_link

The onshape stage costs about 1000 Onshape API requests. Thus it runs only if the hub is
missing, if ``--force`` is given, or if ``--only`` names it.

Usage:
  lite-description-generate lite_dummy
  lite-description-generate lite_dummy --only mjcf,xacro
"""

import argparse

from . import finalize_urdf, onshape_to_urdf, robot_model, urdf_to_mjcf, urdf_to_xacro

STAGES = {
    "onshape": onshape_to_urdf.generate,
    "urdf": finalize_urdf.generate,
    "mjcf": urdf_to_mjcf.generate,
    "xacro": urdf_to_xacro.generate,
}


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate the URDF, MJCF and xacro of a robot from CAD.")
    parser.add_argument("robot", help="Variant name, under lite_description/robots/.")
    parser.add_argument("--only", help="Comma-separated stages to run, e.g. 'mjcf,xacro'.")
    parser.add_argument("--force", action="store_true", help="Run the onshape stage even if the hub exists.")
    args = parser.parse_args(argv)

    robot_dir = robot_model.resolve_robot_dir(args.robot)
    stages = args.only.split(",") if args.only else list(STAGES)
    unknown = sorted(set(stages) - set(STAGES))
    if unknown:
        parser.error(f"unknown stages {unknown}, expected some of {list(STAGES)}")
    if not args.only and not args.force and (robot_dir / "urdf" / f"{robot_dir.name}.urdf").exists():
        print("[onshape] skipped, because the URDF hub exists. Pass --force to export again.")
        stages.remove("onshape")

    for stage in stages:
        for path in STAGES[stage](robot_dir):
            print(f"[{stage}] wrote {path}")


if __name__ == "__main__":
    main()
