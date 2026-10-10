"""Compile the URDF hub into ``mjcf/<robot>.xml``, the MuJoCo model for training and deployment.

MuJoCo compiles the URDF, and this stage then edits the saved MJCF as ``cad/physics.json``
and ``cad/joint_properties.json`` ask. The free joint, the base IMU, the foot contact model
and the contact excludes are opt-in per variant.
"""

import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

try:
    import mujoco
except ModuleNotFoundError as error:
    raise ModuleNotFoundError(
        "MuJoCo is needed to compile MJCF, and ships with the CAD toolchain rather than the "
        "base install. Install it with `uv sync --extra cad`."
    ) from error

from . import robot_model

# The compiler settings of the URDF import. compile_mjcf sets meshdir to the meshes dir.
COMPILER_OPTIONS = {"discardvisual": "false", "fusestatic": "false", "angle": "radian"}

# The indent of the MuJoCo XML writer, so the added sections match the rest.
INDENT = "  "

# The meshes dir of the generated MJCF, relative to mjcf/<robot>.xml.
MESHDIR = "../meshes/visual/"

# The geom group of each collision geom, as in MuJoCo Menagerie and the mjlab G1. The URDF
# import puts the collision geoms in group 0 with the floor, so a ray cast for the terrain
# in group 0 also hits the robot. Viewers and the mjlab ray sensors show groups 0 to 2.
COLLISION_GROUP = 3


def collides(geom: ET.Element) -> bool:
    return geom.get("contype", "1") != "0" or geom.get("conaffinity", "1") != "0"


def compile_mjcf(urdf_path: Path, meshes_dir: Path) -> ET.Element:
    """Compile a URDF with MuJoCo and return the root of the saved MJCF."""
    tree = robot_model.parse(urdf_path)
    # MuJoCo looks for each mesh by its file name in meshdir.
    robot_model.rewrite_mesh_filenames(tree.getroot(), lambda name: name)
    compiler = ET.SubElement(ET.SubElement(tree.getroot(), "mujoco"), "compiler", COMPILER_OPTIONS)
    compiler.set("meshdir", str(meshes_dir.resolve()))
    with tempfile.TemporaryDirectory(prefix="lite_description_mjcf_") as tmp:
        urdf_copy, saved = Path(tmp) / urdf_path.name, Path(tmp) / "mjmodel.xml"
        tree.write(urdf_copy, encoding="utf-8", xml_declaration=True)
        mujoco.mj_saveLastXML(str(saved), mujoco.MjModel.from_xml_path(str(urdf_copy)))
        return ET.parse(saved).getroot()


def add_imu(root: ET.Element, imu: dict) -> None:
    """Add the IMU site on the root body, and the base sensors on the site.

    MujocoSystem builds the ros2_control IMU from ``<prefix>_quat``, ``_gyro`` and ``_accel``.
    The base-velocity MuJoCo plugin reads the ``_vel`` velocimeter for the ``base_lin_vel``
    observation of the policy. The odometry of mujoco_ros2_control reads ``_pos`` and
    ``_quat``. Each sensor is on the site, because mujoco_ros2_control looks up the site of
    each sensor (mujoco_ros2_control.cpp:124) and aborts on a sensor without one.
    """
    site = imu["site"]
    pos = " ".join(str(value) for value in imu["pos"])
    root.find("worldbody/body").insert(0, ET.Element("site", {"name": site, "pos": pos}))
    section = ET.SubElement(root, "sensor")
    for tag, suffix, attributes in (
        ("framequat", "quat", {"objtype": "site", "objname": site}),
        ("gyro", "gyro", {"site": site}),
        ("accelerometer", "accel", {"site": site}),
        ("velocimeter", "vel", {"site": site}),
        ("framepos", "pos", {"objtype": "site", "objname": site}),
    ):
        ET.SubElement(section, tag, {"name": f"{imu['prefix']}_{suffix}", **attributes})


def add_actuators(root: ET.Element, joint_properties: dict) -> None:
    """Add one ``<position>`` actuator for each actuated joint.

    A ``<position>`` actuator puts the joint PD in each ``mj_step``, as the RobStride drive
    closes its own loop at about 1 kHz, and ``implicitfast`` integrates its damping. Thus a
    stiff PD stays stable at a coarse timestep, as in mjlab. ``kp`` and ``kv`` keep the
    MuJoCo defaults, because the gains change with the control mode and MujocoSystem writes
    them each tick.

    There is no ``ctrlrange``, because an RL policy can command a setpoint past the joint
    limit to hold a saturated torque. ``forcerange`` limits the torque.
    """
    joints = [joint.get("name") for joint in root.iter("joint") if joint.get("actuatorfrcrange")]
    section = ET.SubElement(root, "actuator")
    for joint in joints:
        effort = robot_model.joint_property(joint, joint_properties, "effort_limit")
        ET.SubElement(section, "position", {"name": joint, "joint": joint, "forcerange": f"-{effort} {effort}"})


def patch_contacts(root: ET.Element, contact: dict) -> None:
    """Apply the mjlab contact model of a legged robot, and name the collision geoms.

    Each collision geom gets ``condim`` 1, which is frictionless, and the foot geoms get
    ``condim`` 3 with friction. Thus the feet grip the ground, and a leg capsule that touches
    the ground or the other leg slides instead of catching. Priority 1 makes the foot
    friction win over the friction of the floor.

    The URDF import leaves the geoms without names, and the mjlab friction randomization
    selects the foot geoms by name. Thus each collision geom gets the name
    ``<body>_collision_<i>``.
    """
    for body in root.find("worldbody").iter("body"):
        is_foot = body.get("name") in contact["foot_bodies"]
        for index, geom in enumerate(geom for geom in body.findall("geom") if collides(geom)):
            geom.set("name", f"{body.get('name')}_collision_{index}")
            if is_foot:
                geom.set("condim", "3")
                geom.set("friction", f"{contact['foot_friction']} 0.005 0.0001")  # slide torsion roll
                geom.set("priority", "1")
            else:
                geom.set("condim", str(contact["default_condim"]))


def generate(robot_dir: Path) -> list[Path]:
    """Write ``mjcf/<robot>.xml``."""
    robot = robot_dir.name
    joint_properties = json.loads((robot_dir / "cad" / "joint_properties.json").read_text())
    physics = json.loads((robot_dir / "cad" / "physics.json").read_text())
    freejoint = physics.pop("freejoint", False)
    imu = physics.pop("imu", None)
    contact = physics.pop("contact", None)
    exclude = physics.pop("exclude", None)
    root = compile_mjcf(robot_dir / "urdf" / f"{robot}.urdf", robot_dir / "meshes" / "visual")

    for geom in root.iter("geom"):
        if geom.get("type") == "cylinder":
            geom.set("type", "capsule")
        if collides(geom):
            geom.set("group", str(COLLISION_GROUP))

    # The other keys of physics.json are the <option> attributes. Without them, the MJCF has
    # no <option> and keeps the MuJoCo defaults.
    options = {key: str(value) for key, value in physics.items() if not key.startswith("_")}
    if options:
        root.insert(list(root).index(root.find("compiler")) + 1, ET.Element("option", options))

    if exclude:
        # MuJoCo drops the contacts of a body with its parent only. A collider that overlaps
        # a body farther down the chain at rest needs an exclude, as the pelvis capsule
        # across the hip motors overlaps the butt capsule of each hip_roll.
        section = ET.SubElement(root, "contact")
        for body1, body2 in exclude:
            ET.SubElement(section, "exclude", {"body1": body1, "body2": body2})
    # Only a variant with an IMU has sensors. A joint sensor would duplicate qpos and qvel,
    # and mujoco_ros2_control aborts on a sensor that is not on a site.
    if imu:
        add_imu(root, imu)
    add_actuators(root, joint_properties)
    if freejoint:
        free = {"name": "floating_base_joint", "type": "free", "limited": "false", "actuatorfrclimited": "false"}
        root.find("worldbody/body").insert(0, ET.Element("joint", free))
    if contact:
        patch_contacts(root, contact)
    for joint in root.iter("joint"):
        if joint.get("type") != "free":
            name = joint.get("name")
            joint.set("frictionloss", str(robot_model.joint_property(name, joint_properties, "friction_loss")))
            joint.set("armature", str(robot_model.joint_property(name, joint_properties, "armature")))

    root.find("compiler").set("meshdir", MESHDIR)
    root.insert(0, ET.Comment(robot_model.autogen_comment(robot)))
    # ElementTree writes the added sections on one line. Lay them out as MuJoCo does.
    for tag in ("contact", "sensor", "actuator"):
        section = root.find(tag)
        if section is not None:
            ET.indent(section, space=INDENT, level=1)
    for child in root:
        child.tail = "\n" + INDENT
    root[-1].tail = "\n"

    mjcf_path = robot_dir / "mjcf" / f"{robot}.xml"
    mjcf_path.parent.mkdir(exist_ok=True)
    ET.ElementTree(root).write(mjcf_path, encoding="utf-8", xml_declaration=True)
    return [mjcf_path]
