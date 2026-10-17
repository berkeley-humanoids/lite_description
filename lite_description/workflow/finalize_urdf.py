"""Finalize the URDF hub in place: effort and velocity limits, mesh paths and the banner.

The hub has no ``base_link``, because only ROS and KDL need it. Thus this stage is
idempotent, and the MJCF has the CAD root link as its root. The description xacro adds
``base_link``.
"""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from . import robot_model


def generate(robot_dir: Path) -> list[Path]:
    """Finalize ``urdf/<robot>.urdf`` in place."""
    robot = robot_dir.name
    hub = robot_dir / "urdf" / f"{robot}.urdf"
    joint_properties = json.loads((robot_dir / "cad" / "joint_properties.json").read_text())
    tree = robot_model.parse(hub)
    root = tree.getroot()

    # The Onshape export sets each effort limit and each velocity limit to 100. A joint
    # whose motor has no spec table keeps the velocity limit of the export.
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            name, limit = joint.get("name"), joint.find("limit")
            effort = robot_model.joint_property(name, joint_properties, "effort_limit")
            limit.set("effort", str(effort))
            velocity = robot_model.joint_property(name, joint_properties, "velocity_limit", None)
            if velocity is not None:
                limit.set("velocity", str(velocity))
    robot_model.rewrite_mesh_filenames(root, lambda name: f"../meshes/visual/{name}")

    # Replace the banner of the previous run. A comment has a function as its tag.
    while len(root) and not isinstance(root[0].tag, str):
        root.remove(root[0])
    root.insert(0, ET.Comment(robot_model.autogen_comment(robot)))

    ET.indent(tree, space="  ")
    tree.write(hub, encoding="utf-8", xml_declaration=True)
    return [hub]
