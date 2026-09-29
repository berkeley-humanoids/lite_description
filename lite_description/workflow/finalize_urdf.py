"""Finalize the flat URDF in place, the hub the mjcf and xacro stages derive from.

Welds the joints flagged fixed, harmonizes each ``<limit effort>`` to
joint_properties.json, normalizes mesh paths to ``../meshes/visual/`` and stamps the
generated-asset banner.

The hub stays ``base_link``-free, because ``base_link`` is a ROS and KDL concern that
belongs only in the description xacro. That keeps this stage idempotent and the MJCF
rooted at the CAD root link.

Parallel to urdf_to_mjcf and urdf_to_xacro: exposes ``generate(robot_dir) -> list[Path]``.
"""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from . import robot_model


def generate(robot_dir: Path) -> list[Path]:
    """Rewrite ``urdf/<robot>.urdf`` in place and return the path."""
    robot = robot_dir.name
    hub_urdf = robot_dir / "urdf" / f"{robot}.urdf"
    if not hub_urdf.exists():
        raise FileNotFoundError(
            f"{hub_urdf} not found. Run the onshape stage first, or commit the URDF so "
            f"this stage has a hub to finalize.",
        )
    joint_properties = json.loads((robot_dir / "cad" / "joint_properties.json").read_text())

    tree = robot_model.parse(hub_urdf)
    root = tree.getroot()
    robot_model.weld_joints(root, joint_properties)
    robot_model.harmonize_effort(root, joint_properties)
    robot_model.rewrite_mesh_filenames(root, lambda name: f"../meshes/visual/{name}")

    # Make the banner the first child again, dropping the one a previous run left.
    # Comment nodes carry a non-str tag, which is how they are told from elements.
    while len(root) and not isinstance(root[0].tag, str):
        root.remove(root[0])
    root.insert(0, ET.Comment(robot_model.autogen_comment(robot)))

    robot_model.serialize(tree, hub_urdf)
    return [hub_urdf]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Finalize a robot's flat URDF hub in place.")
    parser.add_argument("robot", help="Variant name (under lite_description/robots/) or a path to a robot dir.")
    args = parser.parse_args(argv)
    for path in generate(robot_model.resolve_robot_dir(args.robot)):
        print(f"Finalized {path}")


if __name__ == "__main__":
    main()
