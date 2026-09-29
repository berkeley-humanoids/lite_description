"""URDF parsing and the structural transforms the generator stages share.

The committed flat URDF (``urdf/<robot>.urdf``) is the single kinematic source. This
module parses it and exposes the transforms every emitter needs:

* :func:`weld_joints`      -- retire a CAD-real DoF flagged ``"fixed"`` in joint_properties.
* :func:`harmonize_effort` -- tighten ``<limit effort>`` to the value in joint_properties,
  because the raw Onshape export leaves it at the default of 100.
* :func:`rewrite_mesh_filenames` -- retarget mesh references per consumer, ``package://``
  for ROS and ``../meshes/visual/`` for the flat URDF.
* :func:`inject_base_link` -- add the massless ``base_link`` root that KDL and
  robot_state_publisher want, since the CAD root link carries inertia.

Comments and child order survive the round trip, so generated files stay reviewable.
"""

from collections.abc import Callable
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from lite_description import ROBOTS_DIR

XACRO_NS = "http://www.ros.org/wiki/xacro"

# Owning ament package name, used in package:// URLs and $(find ...).
DEFAULT_PACKAGE = "lite_description"

# Joint types that carry a movable axis, limits and dynamics.
MOVABLE_JOINT_TYPES = frozenset({"revolute", "prismatic"})


def parse(urdf_path: str | Path) -> ET.ElementTree:
    """Parse a URDF, keeping comments so re-emitted files keep their structure."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(urdf_path, parser=parser)


def root_link(root: ET.Element) -> str:
    """Return the one link that is never a joint's child, the kinematic root."""
    children = {
        joint.find("child").get("link") for joint in root.findall("joint")
        if joint.find("child") is not None
    }
    roots = [link.get("name") for link in root.findall("link")
             if link.get("name") not in children]
    if len(roots) != 1:
        raise ValueError(f"Expected exactly one root link, found {roots}.")
    return roots[0]


def joint_limits(root: ET.Element) -> dict[str, dict[str, float]]:
    """Map joint name -> {lower, upper} from each ``<limit>``, in radians.

    A joint whose ``<limit>`` omits either bound is skipped. URDF makes both optional,
    and a continuous joint legitimately has neither.
    """
    limits = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
        lower, upper = limit.get("lower"), limit.get("upper")
        if lower is not None and upper is not None:
            limits[joint.get("name")] = {"lower": float(lower), "upper": float(upper)}
    return limits


def resolve_properties(name: str, properties: dict) -> dict | None:
    """Look up per-joint config by exact name, then by regex key. ``_`` keys are notes."""
    if name in properties:
        return properties[name]
    for pattern, config in properties.items():
        if not pattern.startswith("_") and re.fullmatch(pattern, name):
            return config
    return None


def harmonize_effort(root: ET.Element, joint_properties: dict) -> None:
    """Set each movable joint's ``<limit effort>`` to its ``effort_limit``."""
    for joint in root.findall("joint"):
        if joint.get("type") not in MOVABLE_JOINT_TYPES:
            continue
        config = resolve_properties(joint.get("name"), joint_properties) or {}
        limit = joint.find("limit")
        if "effort_limit" in config and limit is not None:
            limit.set("effort", str(config["effort_limit"]))


def weld_joints(root: ET.Element, joint_properties: dict) -> None:
    """Convert joints flagged ``"fixed": true`` in joint_properties into fixed joints.

    Drops the movable-only children, so the joint rigidly welds its child to its parent.
    This retires a CAD-real DoF, a locked waist for example, from the kinematic hub
    without a re-export from Onshape. The description xacro, the MJCF, where MuJoCo welds
    a fixed joint, and the URDF-to-MJCF parity check then all see one rigid weld.
    """
    for joint in root.findall("joint"):
        # Any non-fixed type can be welded, `continuous` included.
        if joint.get("type") == "fixed":
            continue
        config = resolve_properties(joint.get("name"), joint_properties) or {}
        if not config.get("fixed"):
            continue
        joint.set("type", "fixed")
        for tag in ("axis", "limit", "dynamics", "mimic"):
            for element in joint.findall(tag):
                joint.remove(element)


def rewrite_mesh_filenames(root: ET.Element, rewrite: Callable[[str], str]) -> None:
    """Rewrite every ``<mesh filename>``. ``rewrite`` receives the file basename."""
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if filename:
            mesh.set("filename", rewrite(Path(filename).name))


def inject_base_link(root: ET.Element, base_name: str, child_name: str) -> None:
    """Prepend a massless ``base_name`` link and a fixed joint to the CAD root link."""
    joint = ET.Element("joint", {"name": f"{base_name}_to_{child_name}", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": base_name})
    ET.SubElement(joint, "child", {"link": child_name})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    root.insert(0, joint)
    root.insert(0, ET.Element("link", {"name": base_name}))


def serialize(tree: ET.ElementTree, path: str | Path) -> None:
    """Indent and write a URDF tree with an XML declaration."""
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def to_string(element: ET.Element) -> str:
    """Indent an element and return it as XML text."""
    ET.indent(element, space="  ")
    return ET.tostring(element, encoding="unicode")


def autogen_comment(robot: str) -> str:
    """Return the banner text that marks a generated asset, without ``<!-- -->``.

    Used verbatim as an XML comment at the top of every generated URDF, MJCF and xacro,
    so the source and the regenerate command are obvious to anyone who opens the file.
    """
    # NB: an XML comment may not contain '--', so keep the text dash-free.
    return (
        f" GENERATED by lite_description.workflow from Onshape CAD. Do not edit by hand. "
        f"Regenerate: lite-description-generate {robot} "
    )


def resolve_robot_dir(robot: str) -> Path:
    """Return the robot dir that ``robot`` names, as a path or as a variant under ``ROBOTS_DIR``.

    The dir must hold the ``cad/`` inputs. The installed package leaves ``cad/`` out, so the
    generator runs from a checkout only.
    """
    for path in (Path(robot), ROBOTS_DIR / robot):
        if (path / "cad").is_dir():
            return path
    raise FileNotFoundError(
        f"No robot dir with a cad/ subdir at '{robot}' or {ROBOTS_DIR / robot}. "
        "The installed package does not carry cad/, so run the generator from a checkout.",
    )
