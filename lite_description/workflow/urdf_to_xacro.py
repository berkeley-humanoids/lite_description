"""Generate the ROS 2 xacro artifacts for a robot from its URDF hub and ros2_control.json.

Writes three files into ``<robot_dir>/xacro/``, following the ``ros2_control_demos``
split of a reusable model macro, a separate hardware file, and a thin assembly:

* ``<robot>.description.xacro``  -- ``<xacro:macro name="<robot>_description">`` around the
  CAD kinematics: a ``base_link`` root, ``${mesh_root}``-parameterised mesh paths, effort
  limits harmonised to ``joint_properties.json``.
* ``<robot>.ros2_control.xacro`` -- the hardware macros: a joint macro, per-group joint
  macros, one combined block for the mock and MuJoCo backends, per-bus blocks for real
  hardware, and a top macro that dispatches between them.
* ``<robot>.urdf.xacro``         -- the assembly: declares the args, includes the two
  macros above and instantiates them.

Backend selection follows the Universal Robots description convention: one boolean per
non-real backend (``use_mock_hardware``, ``sim_mujoco``), real hardware as the fallback
when both are false. The switches reach only the ``<hardware>`` plugin choice. Joint
macros are backend-agnostic and always emit their hardware params, because every backend
ignores the params it does not know.

Every ``${...}`` below is a xacro expression, written ``${{...}}`` inside an f-string.
``$(arg ...)`` and ``$(find ...)`` use parentheses and need no escaping. Position limits
come from the URDF and are never re-declared in ros2_control.json.
"""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from . import robot_model

DEFAULT_PACKAGE = robot_model.DEFAULT_PACKAGE

# Backend-switch args. Both false selects real hardware.
MOCK_ARG = "use_mock_hardware"
SIM_ARG = "sim_mujoco"

# State interfaces of an IMU, in the order imu_sensor_broadcaster expects.
IMU_STATE_INTERFACES = (
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
)

# Four-bar geometry a joint declares under ``linkage`` in ros2_control.json. The four
# lengths share one unit, ``alpha_zero`` is in degrees and ``sign`` is +1 or -1. All six
# are required: humanoid_devices_robstride rejects a partial set, and its defaults for
# the last two would otherwise apply silently.
LINKAGE_KEYS = ("ground", "crank_in", "coupler", "crank_out", "alpha_zero", "sign")

MAX_LINE = 100


def _banner(robot: str) -> str:
    """Return the generated-asset banner as a literal XML comment."""
    return f"<!--{robot_model.autogen_comment(robot)}-->"


def _pack(tokens: list[str], indent: str) -> str:
    """Join tokens into ``MAX_LINE``-wide lines, hanging-indented by ``indent``.

    The budget leaves two columns for the tag's closing ``">`` or ``/>``.
    """
    lines: list[list[str]] = [[]]
    for token in tokens:
        if lines[-1] and len(indent) + len(" ".join(lines[-1] + [token])) > MAX_LINE - 2:
            lines.append([])
        lines[-1].append(token)
    return ("\n" + indent).join(" ".join(line) for line in lines)


def _macro_open(name: str, params: list[str]) -> str:
    """Return a ``<xacro:macro>`` opening tag, wrapping a long parameter list."""
    head = f'  <xacro:macro name="{name}"'
    if not params:
        return f"{head}>"
    single = f'{head} params="{" ".join(params)}">'
    if len(single) <= MAX_LINE:
        return single
    # Wrapped form puts params= under name=, and its continuations under the open quote.
    head_indent = " " * len("  <xacro:macro ")
    body_indent = head_indent + " " * len('params="')
    return f'{head}\n{head_indent}params="{_pack(params, body_indent)}">'


def _sensor_element(name: str, indent: str) -> str:
    """Return a ros2_control ``<sensor>`` carrying the IMU state interfaces."""
    interfaces = "\n".join(
        f'{indent}  <state_interface name="{n}"/>' for n in IMU_STATE_INTERFACES
    )
    return f'{indent}<sensor name="{name}">\n{interfaces}\n{indent}</sensor>'


# ---------------------------------------------------------------------------
# Description macro (model kinematics)
# ---------------------------------------------------------------------------


def build_description_xacro(
    urdf_path: Path,
    robot: str,
    joint_properties: dict,
    base_link: dict,
    package: str = DEFAULT_PACKAGE,
) -> str:
    """Return the description xacro: the CAD kinematics wrapped in a reusable macro."""
    root = robot_model.parse(urdf_path).getroot()
    robot_model.harmonize_effort(root, joint_properties)
    robot_model.rewrite_mesh_filenames(root, lambda name: f"${{mesh_root}}/{name}")
    robot_model.inject_base_link(root, base_link["name"], base_link["child"])

    ET.register_namespace("xacro", robot_model.XACRO_NS)
    out_root = ET.Element("robot")
    macro = ET.SubElement(
        out_root,
        f"{{{robot_model.XACRO_NS}}}macro",
        {
            "name": f"{robot}_description",
            "params": f"mesh_root:=package://{package}/robots/{robot}/meshes/visual",
        },
    )
    for element in list(root):
        macro.append(element)

    return f'<?xml version="1.0"?>\n{_banner(robot)}\n{robot_model.to_string(out_root)}\n'


# ---------------------------------------------------------------------------
# ros2_control macros
# ---------------------------------------------------------------------------


def _joint_macro(robot: str, command: list[str], state: list[str], linkage: bool) -> str:
    """Return one joint macro. With ``linkage``, it also carries the four-bar params."""
    lines = [f'      <command_interface name="{n}"/>' for n in command]
    lines += [f'      <state_interface name="{n}"/>' for n in state]
    interfaces = "\n".join(lines)
    # (macro param, emitted <param> name). They differ only for use_linkage, whose
    # plugin-side spelling is linkage_enabled.
    hardware = [(p, p) for p in ("can_id", "model", "direction",
                                 "lower_limit", "upper_limit", "torque_limit", "current_limit")]
    if linkage:
        hardware += [(f"linkage_{key}", f"linkage_{key}") for key in LINKAGE_KEYS]
        hardware += [("use_linkage", "linkage_enabled")]
        header = f"""  <!-- A joint driven through a four-bar linkage, otherwise identical to
       {robot}_joint. The geometry is a physical fact, so it is always published:
       calibrate_robot reads it from /robot_description to refer the URDF joint limits into
       actuator space even on the runs where the plugin must not apply the transform.
       linkage_enabled is the switch for that. With it true, RobstrideSystem converts
       joint<->actuator through the linkage and refers the commanded impedance by J^2, so
       /joint_states and every controller speak true joint space. Calibration sets it
       false, because it needs raw actuator angles. -->"""
    else:
        header = f"""  <!-- One joint: {len(command)} command + {len(state)} state interfaces (MIT mode).
       The params are read by humanoid_devices_robstride. The mock and MuJoCo backends
       ignore params they do not know, so this one macro serves all three. -->"""

    name = f"{robot}_linkage_joint" if linkage else f"{robot}_joint"
    params = ["name", *(param for param, _ in hardware)]
    body = "\n".join(
        f'      <param name="{emitted}">${{{param}}}</param>' for param, emitted in hardware
    )
    return f"""{header}
{_macro_open(name, params)}
    <joint name="${{name}}">
{interfaces}
{body}
    </joint>
  </xacro:macro>"""


def _joint_call(robot: str, joint: dict, limits: dict) -> str:
    """Return one joint-macro instantiation, with position limits taken from the URDF."""
    linkage = joint.get("linkage")
    macro = f"{robot}_linkage_joint" if linkage else f"{robot}_joint"
    limit = limits[joint["name"]]
    attributes = [
        f'name="{joint["name"]}"',
        f'can_id="{joint["can_id"]}"',
        f'model="{joint["model"]}"',
        f'direction="{joint["direction"]}"',
        f'lower_limit="{limit["lower"]}"',
        f'upper_limit="{limit["upper"]}"',
        f'torque_limit="{joint["torque_limit"]}"',
        f'current_limit="{joint["current_limit"]}"',
    ]
    if linkage:
        missing = [key for key in LINKAGE_KEYS if key not in linkage]
        if missing:
            raise ValueError(
                f"joint {joint['name']!r}: linkage is missing {missing}. "
                f"Give all of {list(LINKAGE_KEYS)}."
            )
        attributes += [f'linkage_{key}="{linkage[key]}"' for key in LINKAGE_KEYS]
        attributes.append('use_linkage="${use_linkage}"')

    head = f"    <xacro:{macro} "
    return f"{head}{_pack(attributes, ' ' * len(head))}/>"


def _group_macro(robot: str, group: str, joints: list[dict], limits: dict) -> str:
    """Return the macro that instantiates every joint of one group."""
    calls = "\n".join(_joint_call(robot, joint, limits) for joint in joints)
    # use_linkage only becomes a param where a joint in this group declares a four-bar.
    params = ["use_linkage:=true"] if any(j.get("linkage") for j in joints) else []
    return f"""{_macro_open(f"{robot}_{group}_joints", params)}
{calls}
  </xacro:macro>"""


def _combined_macro(robot: str, groups: list[dict], backends: dict, imu: dict | None) -> str:
    """Return the mock and MuJoCo macro: one block per bus, named as the real one."""
    blocks = []
    for group in groups:
        # A gpio only under mock. The real backend exports these two through
        # export_unlisted_state_interface_descriptions(), and MujocoSystem
        # overrides the deprecated by-value export_state_interfaces(), which
        # bypasses the base class's gpio handling -- declaring them there would
        # promise interfaces that never appear.
        safety = (
            f'\n      <xacro:unless value="${{{SIM_ARG}}}">\n'
            f'        <gpio name="{group["block_name"]}">\n'
            f'          <state_interface name="safety_level"/>\n'
            f'          <state_interface name="safety_flags"/>\n'
            f'        </gpio>\n'
            f'      </xacro:unless>'
        )
        blocks.append(f"""    <ros2_control name="{group["block_name"]}" type="system">
      <hardware>
        <xacro:if value="${{{SIM_ARG}}}">
          <plugin>{backends["sim"]}</plugin>
        </xacro:if>
        <xacro:unless value="${{{SIM_ARG}}}">
          <plugin>{backends["mock"]}</plugin>
        </xacro:unless>
      </hardware>
      <xacro:{robot}_{group["name"]}_joints/>{safety}
    </ros2_control>""")
    if imu and imu.get("real_block_name"):
        # The real backend gives the IMU its own sensor component, so sim matches
        # the name. Only MuJoCo backs the state interfaces; mock has no source.
        blocks.append(
            f'    <xacro:if value="${{{SIM_ARG}}}">\n'
            f'      <ros2_control name="{imu["real_block_name"]}" type="sensor">\n'
            f'        <hardware>\n'
            f'          <plugin>{backends["sim"]}</plugin>\n'
            f'        </hardware>\n'
            f'{_sensor_element(imu["name"], "        ")}\n'
            f'      </ros2_control>\n'
            f'    </xacro:if>')
    elif imu:
        # A sim-only IMU has no real component to match, so it stays nested in the
        # first bus block rather than inventing a component name of its own.
        nested = (
            f'\n      <xacro:if value="${{{SIM_ARG}}}">\n'
            f'{_sensor_element(imu["name"], "        ")}\n'
            f'      </xacro:if>')
        blocks[0] = blocks[0].replace("\n    </ros2_control>", f"{nested}\n    </ros2_control>")
    body = "\n".join(blocks)
    return f"""  <!-- Mock and MuJoCo backends. One block per bus, carrying the same component
       names as the real backend, so anything keyed by component name means the
       same thing on every backend: a controller's safety_components, the
       hardware_spawner, the manager's hardware_components_initial_state. -->
{_macro_open(f"{robot}_ros2_control_combined", [SIM_ARG])}
{body}
  </xacro:macro>"""


def _real_macro(
    robot: str, groups: list[dict], params: list[str], backends: dict, imu: dict | None,
    linkage_groups: set[str],
) -> str:
    """Return the real-hardware macro: one ``<ros2_control>`` block per CAN bus."""
    blocks = []
    for group in groups:
        use_linkage = ' use_linkage="${use_linkage}"' if group["name"] in linkage_groups else ""
        blocks.append(f"""    <ros2_control name="{group["block_name"]}" type="system">
      <hardware>
        <plugin>{backends["real"]}</plugin>
        <param name="can_interface">${{{group["can_interface_arg"]}}}</param>
        <param name="calibration_file">${{calibration_file}}</param>
      </hardware>
      <xacro:{robot}_{group["name"]}_joints{use_linkage}/>
    </ros2_control>""")
    # An IMU without a real_plugin exists only in sim, backed by MujocoSystem inside the
    # combined block, so real hardware gets no sensor component for it.
    if imu and imu.get("real_plugin"):
        blocks.append(f"""    <ros2_control name="{imu["real_block_name"]}" type="sensor">
      <hardware>
        <plugin>{imu["real_plugin"]}</plugin>
        <param name="port">${{imu_port}}</param>
      </hardware>
{_sensor_element(imu["name"], "      ")}
    </ros2_control>""")
    body = "\n".join(blocks)
    return f"""  <!-- Real hardware: one <ros2_control> block per CAN bus. The controller_manager
       runs them concurrently and exposes a single flat joint list to controllers. -->
{_macro_open(f"{robot}_ros2_control_real", params)}
{body}
  </xacro:macro>"""


def _top_macro(robot: str, args: list[str], real_params: list[str]) -> str:
    """Return the top macro dispatching to the combined or the real block."""
    forwarded = "\n".join(f'        {arg}="${{{arg}}}"' for arg in real_params)
    return f"""  <!-- Backend dispatch, following the Universal Robots description convention: one
       boolean per non-real backend, real hardware as the fallback when both are false.
       {SIM_ARG} wins over {MOCK_ARG}. -->
{_macro_open(f"{robot}_ros2_control", args)}
    <xacro:if value="${{{SIM_ARG} or {MOCK_ARG}}}">
      <xacro:{robot}_ros2_control_combined {SIM_ARG}="${{{SIM_ARG}}}"/>
    </xacro:if>
    <xacro:unless value="${{{SIM_ARG} or {MOCK_ARG}}}">
      <xacro:{robot}_ros2_control_real
{forwarded}/>
    </xacro:unless>
  </xacro:macro>"""


def build_ros2_control_xacro(robot: str, ros2_control: dict, limits: dict) -> str:
    """Return the hardware xacro: joint macros, group macros and the three block macros."""
    joints_by_group: dict[str, list[dict]] = {}
    for joint in ros2_control["joints"]:
        joints_by_group.setdefault(joint["group"], []).append(joint)
    linkage_groups = {
        group for group, joints in joints_by_group.items()
        if any(j.get("linkage") for j in joints)
    }
    empty = [g["name"] for g in ros2_control["groups"] if not joints_by_group.get(g["name"])]
    if empty:
        raise ValueError(
            f"{robot}: groups {empty} declare a bus but list no joints. Give each group "
            f"at least one joint, or drop the group."
        )

    command = ros2_control["interfaces"]["command"]
    state = ros2_control["interfaces"]["state"]
    groups = ros2_control["groups"]
    imu = ros2_control.get("imu")
    args = list(ros2_control["args"])
    # The real macro takes every arg except the backend switches, which only the top
    # macro reads. An arg a robot does not have (imu_port, use_linkage) is absent
    # everywhere, so ros2_control.json's `args` is the single source for both lists.
    real_params = [a for a in args if a not in (MOCK_ARG, SIM_ARG)]
    if linkage_groups and "use_linkage" not in real_params:
        raise ValueError(
            f"{robot}: joints in {sorted(linkage_groups)} declare a four-bar linkage, so "
            f'ros2_control.json must declare a "use_linkage" arg.'
        )

    parts = [_joint_macro(robot, command, state, linkage=False)]
    if linkage_groups:
        parts.append(_joint_macro(robot, command, state, linkage=True))
    parts += [
        _group_macro(robot, group["name"], joints_by_group[group["name"]], limits)
        for group in groups
    ]
    parts += [
        _combined_macro(robot, groups, ros2_control["backends"], imu),
        _real_macro(robot, groups, real_params, ros2_control["backends"], imu, linkage_groups),
        _top_macro(robot, args, real_params),
    ]

    body = "\n\n".join(parts)
    return (
        f'<?xml version="1.0"?>\n{_banner(robot)}\n'
        f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n\n{body}\n\n</robot>\n'
    )


# ---------------------------------------------------------------------------
# Top assembly
# ---------------------------------------------------------------------------


def build_assembly_xacro(robot: str, ros2_control: dict | None, package: str = DEFAULT_PACKAGE) -> str:
    """Return the assembly xacro. Without a ros2_control spec it emits the model alone."""
    includes = f'  <xacro:include filename="$(find {package})/robots/{robot}/xacro/{robot}.description.xacro"/>'
    if ros2_control is None:
        return f"""<?xml version="1.0"?>
{_banner(robot)}
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot}">

{includes}

  <xacro:{robot}_description/>
</robot>
"""

    args = ros2_control["args"]
    declarations = "\n".join(f'  <xacro:arg name="{k}" default="{v}"/>' for k, v in args.items())
    forwarded = "\n".join(f'    {arg}="$(arg {arg})"' for arg in args)
    return f"""<?xml version="1.0"?>
{_banner(robot)}
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot}">
{declarations}

{includes}
  <xacro:include filename="$(find {package})/robots/{robot}/xacro/{robot}.ros2_control.xacro"/>

  <xacro:{robot}_description/>

  <xacro:{robot}_ros2_control
{forwarded}/>
</robot>
"""


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------


def generate(robot_dir: Path, package: str = DEFAULT_PACKAGE) -> list[Path]:
    """Write the xacro artifacts for one ``robots/<robot>/`` directory."""
    robot = robot_dir.name
    hub_urdf = robot_dir / "urdf" / f"{robot}.urdf"
    joint_properties = json.loads((robot_dir / "cad" / "joint_properties.json").read_text())

    # ros2_control.json is optional. A robot without one (the full lite) generates a
    # model-only package: a description macro and a thin assembly, no <ros2_control>.
    # Deployment wiring lives only where the hardware is known.
    ros2_control_path = robot_dir / "cad" / "ros2_control.json"
    ros2_control = json.loads(ros2_control_path.read_text()) if ros2_control_path.exists() else None

    root = robot_model.parse(hub_urdf).getroot()
    base_link = (ros2_control or {}).get("base_link") or {
        "name": "base_link", "child": robot_model.root_link(root),
    }

    outputs = {
        f"{robot}.description.xacro": build_description_xacro(
            hub_urdf, robot, joint_properties, base_link, package),
        f"{robot}.urdf.xacro": build_assembly_xacro(robot, ros2_control, package),
    }
    if ros2_control is not None:
        outputs[f"{robot}.ros2_control.xacro"] = build_ros2_control_xacro(
            robot, ros2_control, robot_model.joint_limits(root))

    xacro_dir = robot_dir / "xacro"
    xacro_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for filename, content in outputs.items():
        path = xacro_dir / filename
        path.write_text(content)
        written.append(path)
    return written


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate ROS 2 xacro files for a robot.")
    parser.add_argument("robot", help="Variant name (under lite_description/robots/) or a path to a robot dir.")
    parser.add_argument("--package", default=DEFAULT_PACKAGE, help="Owning ament package name.")
    args = parser.parse_args(argv)
    for path in generate(robot_model.resolve_robot_dir(args.robot), package=args.package):
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
