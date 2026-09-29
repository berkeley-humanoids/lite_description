"""Compile the URDF hub into a post-processed MJCF, the training and deployment sim asset.

Called by ``generate.py``. The flat URDF is the hub:

    URDF + meshes  ->  MuJoCo compile  ->  post-process  ->  mjcf/<robot>.xml

Post-processing replaces cylinder geoms with capsules, injects the ``<option>`` physics
tuning from ``cad/physics.json``, synthesizes one ``<position>`` actuator per actuated
joint, and sets per-joint frictionloss and armature from ``cad/joint_properties.json``.
Floating-base, IMU and contact handling are per-variant opt-ins, also from physics.json.

The compiled model is parsed once and written once, so the intermediate states never
reach disk. Mesh resolution is decoupled from the URDF's own relative paths: mesh refs
become bare basenames and MuJoCo is pointed at the real meshes dir while it compiles, so
this works wherever the URDF lives.
"""

import argparse
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

# Applied to the URDF before MuJoCo imports it. meshdir is set separately, to the real
# meshes dir, so compilation resolves the STLs.
COMPILER_OPTIONS = {
    "discardvisual": "false",
    "fusestatic": "false",
    "angle": "radian",
}

# Indent width MuJoCo's own XML writer uses, so synthesized sections match the rest.
INDENT = "  "

# Where the generated MJCF looks for its meshes, relative to mjcf/<robot>.xml.
MESHDIR = "../meshes/visual/"


def _joint_property(joint_name: str, joint_properties: dict, key: str):
    """Return one property of a joint, by exact name then by regex key."""
    config = robot_model.resolve_properties(joint_name, joint_properties)
    if config is None:
        raise ValueError(
            f"No joint properties found for joint '{joint_name}'. "
            "Add an exact or regex entry to joint_properties.json.",
        )
    if key not in config:
        raise ValueError(
            f"Joint '{joint_name}' is missing required attribute '{key}' "
            "in joint_properties.json.",
        )
    return config[key]


def _section(root: ET.Element, tag: str, before: str | None = None) -> ET.Element:
    """Return an empty top-level section, creating or clearing it as needed."""
    section = root.find(tag)
    if section is not None:
        section.clear()
        return section

    section = ET.Element(tag)
    sibling = root.find(before) if before else None
    if sibling is None:
        root.append(section)
    else:
        root.insert(list(root).index(sibling), section)
    return section


def _indent_section(root: ET.Element, section: ET.Element) -> None:
    """Lay a synthesized top-level section out one element per line.

    ElementTree gives appended elements no whitespace, so a section built here would
    serialize as one unreadable line while the surrounding model stays indented. This
    touches the line breaks between top-level sections and the children of ``section``.
    Bodies written by MuJoCo keep their existing layout.
    """
    ET.indent(section, space=INDENT, level=1)
    for child in root:
        child.tail = "\n" + INDENT
    root[-1].tail = "\n"  # The last section closes the file, so dedent for </mujoco>.


# ---------------------------------------------------------------------------
# MuJoCo compile
# ---------------------------------------------------------------------------


def compile_mjcf(urdf_path: Path, meshes_dir: Path, work_dir: Path) -> ET.Element:
    """Compile a URDF with MuJoCo and return the saved model's root element."""
    tree = robot_model.parse(urdf_path)
    robot_model.rewrite_mesh_filenames(tree.getroot(), lambda name: name)

    # NB: an Element with no children is falsy, so these must test against None.
    mujoco_tag = tree.getroot().find("mujoco")
    if mujoco_tag is None:
        mujoco_tag = ET.SubElement(tree.getroot(), "mujoco")
    compiler = mujoco_tag.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco_tag, "compiler")
    compiler.set("meshdir", str(meshes_dir.resolve()))
    for key, value in COMPILER_OPTIONS.items():
        compiler.set(key, value)

    urdf_copy = work_dir / urdf_path.name
    tree.write(urdf_copy, encoding="utf-8", xml_declaration=True)

    saved = work_dir / "mjmodel.xml"
    mujoco.mj_saveLastXML(str(saved), mujoco.MjModel.from_xml_path(str(urdf_copy)))
    return ET.parse(saved).getroot()


# ---------------------------------------------------------------------------
# MJCF post-processing
# ---------------------------------------------------------------------------


def replace_cylinders_with_capsules(root: ET.Element) -> int:
    """Retype every cylinder geom as a capsule. Returns how many changed."""
    cylinders = [geom for geom in root.iter("geom") if geom.get("type") == "cylinder"]
    for geom in cylinders:
        geom.set("type", "capsule")
    return len(cylinders)


def add_option(root: ET.Element, options: dict) -> None:
    """Set the MuJoCo ``<option>`` attributes from physics.json.

    A variant without physics.json gets no ``<option>`` element at all, rather than an
    empty one, so its MJCF keeps MuJoCo's own defaults visible.
    """
    if not options:
        return
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        compiler = root.find("compiler")
        root.insert(list(root).index(compiler) + 1 if compiler is not None else 0, option)
    for key, value in options.items():
        if not key.startswith("_"):
            option.set(key, str(value))


def add_actuators(root: ET.Element, joint_properties: dict) -> None:
    """Synthesize one ``<position>`` actuator per actuated joint. Emits no ``<sensor>``.

    ``<position>`` rather than ``<motor>`` because the joint-level PD belongs inside the
    physics step. MuJoCo evaluates actuators every ``mj_step``, which mirrors the real
    RobStride drive closing its onboard loop at about 1 kHz independently of the host
    rate, and lets ``implicitfast`` integrate the damping term implicitly. That is why a
    stiff PD stays stable at a coarse timestep, and it is how mjlab trains. A ``<motor>``
    makes ``ctrl`` a raw torque, which forces the host to sample the PD once per
    controller_manager tick.

    ``kp`` and ``kv`` stay at MuJoCo's defaults on purpose. The operative gains are
    mode-dependent, because standby, policy and damping all differ, and they arrive over
    the ros2_control stiffness and damping command interfaces. MujocoSystem therefore
    overwrites ``actuator_gainprm`` and ``actuator_biasprm`` every tick, so values baked
    in here would only mislead.

    No ``ctrlrange`` or ``ctrllimited`` is emitted, and that is load-bearing. An RL policy
    may command a position setpoint beyond the mechanical joint limit to hold a saturating
    torque, so clamping the setpoint would silently cap the achievable torque.
    ``forcerange`` still bounds the output, which is the limit that is physically real.

    jointpos and jointvel sensors are omitted for two reasons. They duplicate
    mjData.qpos/qvel, because mjlab reads joint state from the Entity/Articulation data
    and not from named MJCF sensors. They also make the model unloadable under
    mujoco_ros2_control: its plugin init loops over every sensor and builds
    ``std::string(mj_id2name(model, mjOBJ_SITE, sensor_objid))``
    (mujoco_ros2_control.cpp:124), which is null for a joint sensor when the model has no
    ``<site>`` elements, and the process aborts. Keeping only actuators leaves the single
    MJCF usable by both the RL sim and ros2_control.
    """
    joints = [
        joint.get("name") for joint in root.iter("joint")
        if joint.get("name") and joint.get("actuatorfrcrange")
    ]
    if not joints:
        print("No joints with actuatorfrcrange found in XML")
        return

    section = _section(root, "actuator")
    for joint_name in joints:
        effort_limit = abs(_joint_property(joint_name, joint_properties, "effort_limit"))
        actuator = ET.SubElement(section, "position")
        actuator.set("name", joint_name)
        actuator.set("joint", joint_name)
        actuator.set("forcerange", f"-{effort_limit} {effort_limit}")
    _indent_section(root, section)


def add_freejoint(root: ET.Element) -> None:
    """Give the root body a 6-DoF free joint, making the base float."""
    freejoint = ET.Element("joint")
    freejoint.set("name", "floating_base_joint")
    freejoint.set("type", "free")
    freejoint.set("limited", "false")
    freejoint.set("actuatorfrclimited", "false")
    root.find("worldbody").find("body").insert(0, freejoint)
    print("Added floating_base_joint to first body element")


def add_imu_site_and_sensors(root: ET.Element, imu: dict) -> None:
    """Add an IMU ``<site>`` on the root body plus the base-state sensors.

    A floating-base locomotion sim needs two things at the base. Orientation, angular
    velocity and linear acceleration reach ros2_control's MujocoSystem as the
    ``<prefix>_imu`` sensor, built from the MJCF ``_quat``, ``_gyro`` and ``_accel``
    sensors, and imu_sensor_broadcaster republishes them as sensor_msgs/Imu on /imu/data.
    Body-frame linear velocity comes from the ``<prefix>_vel`` velocimeter, which the
    base-velocity MuJoCo physics plugin reads for the RL policy's ``base_lin_vel`` obs
    term. A state estimator supplies that term on hardware.

    A ``framepos`` is emitted as well, so mujoco_ros2_control's framepos/framequat
    Odometry publisher, which is keyed by site name, is well-formed.

    The ``<site>`` is mandatory: mujoco_ros2_control resolves every sensor's site through
    ``mj_id2name(mjOBJ_SITE, ...)`` and aborts on a siteless model. See add_actuators.
    physics.json gates this block, so the fixed-base variants stay sensor-free.
    """
    site_name = imu.get("site", "imu_site")
    prefix = imu.get("prefix", "base")

    body = root.find("worldbody").find("body")
    site = ET.Element("site")
    site.set("name", site_name)
    site.set("pos", " ".join(str(v) for v in imu.get("pos", [0.0, 0.0, 0.0])))
    # Keep a leading freejoint, if there is one, as the body's first child.
    lead = 1 if len(body) and body[0].tag == "joint" and body[0].get("type") == "free" else 0
    body.insert(lead, site)

    # The IMU triad that MujocoSystem reads, then the velocimeter (base linear velocity)
    # and a framepos (odom completeness).
    section = _section(root, "sensor", before="actuator")
    for tag, suffix, attributes in (
        ("framequat", "quat", {"objtype": "site", "objname": site_name}),
        ("gyro", "gyro", {"site": site_name}),
        ("accelerometer", "accel", {"site": site_name}),
        ("velocimeter", "vel", {"site": site_name}),
        ("framepos", "pos", {"objtype": "site", "objname": site_name}),
    ):
        ET.SubElement(section, tag, {"name": f"{prefix}_{suffix}", **attributes})
    _indent_section(root, section)
    print(f"Added IMU site '{site_name}' + {prefix}_quat/gyro/accel/vel/pos sensors")


def patch_contacts(root: ET.Element, contact: dict) -> None:
    """Match mjlab's contact model for a legged robot.

    mjlab's ``patch_spec`` defaults every collision geom to ``condim=1``, a frictionless
    point contact, and then re-promotes only the foot geoms to ``condim=3`` with friction
    and priority. The feet therefore grip the ground while a grazing shin, thigh or
    self-contact slides instead of catching, which matters for a floating-base gait. Our
    stock MJCF leaves every collision geom at MuJoCo's default ``condim=3`` with friction,
    so a leg capsule that brushes the ground or the other leg grabs and can trip the robot.

    physics.json's ``contact`` block configures this. ``foot_bodies`` lists the bodies
    whose collision geoms keep friction, ``foot_friction`` is the slide coefficient, and
    ``default_condim`` applies to everything else, where 1 means frictionless. The higher
    ``priority`` on the feet makes the foot friction win over the floor's in the pair.
    """
    foot_bodies = set(contact.get("foot_bodies", ()))
    foot_friction = contact.get("foot_friction", 1.0)
    default_condim = str(int(contact.get("default_condim", 1)))
    counts = {"foot": 0, "other": 0}

    def walk(body: ET.Element, in_foot: bool) -> None:
        is_foot = in_foot or body.get("name") in foot_bodies
        # Name collision geoms <body>_collision_<i>. MuJoCo leaves URDF-imported geoms
        # anonymous, but the training env selects them BY NAME -- mjlab's friction
        # randomization matches r"^(left|right)_foot_collision_\d+$" -- and an unnamed
        # geom can never match, so the randomization would silently do nothing. This only
        # fills blanks: an explicit name in the MJCF wins.
        index = 0
        for geom in body.findall("geom"):
            if geom.get("contype", "1") == "0" and geom.get("conaffinity", "1") == "0":
                continue
            if not geom.get("name"):
                geom.set("name", f"{body.get('name')}_collision_{index}")
            index += 1
            if is_foot:
                geom.set("condim", "3")
                geom.set("friction", f"{foot_friction} 0.005 0.0001")  # slide torsion roll
                geom.set("priority", "1")
            else:
                geom.set("condim", default_condim)
            counts["foot" if is_foot else "other"] += 1
        for child in body.findall("body"):
            walk(child, is_foot)

    for body in root.find("worldbody").findall("body"):
        walk(body, False)

    print(
        f"Patched contacts: {counts['foot']} foot geom(s) condim=3 friction={foot_friction} "
        f"priority=1 (bodies {sorted(foot_bodies)}); "
        f"{counts['other']} other geom(s) condim={default_condim}"
    )


def apply_joint_properties(root: ET.Element, joint_properties: dict) -> None:
    """Set frictionloss and armature on every non-free joint."""
    for joint in root.iter("joint"):
        name = joint.get("name")
        if not name or joint.get("type") == "free":
            continue
        joint.set("frictionloss", str(_joint_property(name, joint_properties, "friction_loss")))
        joint.set("armature", str(_joint_property(name, joint_properties, "armature")))


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------


def generate(robot_dir: Path) -> list[Path]:
    """Write mjcf/<robot>.xml for one ``robots/<robot>/`` directory."""
    robot = robot_dir.name
    cad_dir = robot_dir / "cad"
    joint_properties = json.loads((cad_dir / "joint_properties.json").read_text())

    physics_path = cad_dir / "physics.json"
    options = json.loads(physics_path.read_text()) if physics_path.exists() else {}
    # Floating base, IMU and contact are per-variant opt-ins. Pop them so they do not
    # leak into the MuJoCo <option> tag.
    freejoint = options.pop("freejoint", False)
    imu = options.pop("imu", None)
    contact = options.pop("contact", None)

    with tempfile.TemporaryDirectory(prefix="lite_description_mjcf_") as tmp:
        root = compile_mjcf(
            robot_dir / "urdf" / f"{robot}.urdf",
            robot_dir / "meshes" / "visual",
            Path(tmp),
        )

    if freejoint:
        add_freejoint(root)
    replaced = replace_cylinders_with_capsules(root)
    if replaced:
        print(f"Replaced {replaced} cylinder geom(s) with capsules")
    add_option(root, options)
    add_actuators(root, joint_properties)
    if imu:
        add_imu_site_and_sensors(root, imu)
    if contact:
        patch_contacts(root, contact)
    apply_joint_properties(root, joint_properties)

    root.find("compiler").set("meshdir", MESHDIR)
    banner = ET.Comment(robot_model.autogen_comment(robot))
    banner.tail = "\n" + INDENT
    root.insert(0, banner)

    mjcf_path = robot_dir / "mjcf" / f"{robot}.xml"
    mjcf_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(mjcf_path, encoding="utf-8", xml_declaration=True)
    return [mjcf_path]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate mjcf/<robot>.xml for a robot.")
    parser.add_argument("robot", help="Variant name (under lite_description/robots/) or a path to a robot dir.")
    args = parser.parse_args(argv)
    for path in generate(robot_model.resolve_robot_dir(args.robot)):
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
