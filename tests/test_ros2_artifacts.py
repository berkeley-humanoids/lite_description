"""Tests for the generated ROS 2 xacro artifacts under lite_description/robots/<robot>/xacro/.

Covers three things:
  1. every generated xacro is well-formed XML;
  2. the committed xacro matches what the generator produces *now* (regeneration is
     deterministic and the committed files are not stale) -- this is what keeps the
     ROS artifacts from drifting away from cad/;
  3. for robots with a ros2_control.json, the hardware block has the expected
     structure (backends, per-bus blocks, MIT interfaces, CAN ids, URDF-sourced limits).

A skipped-by-default test also expands the xacro with `xacro` + `check_urdf` when those
tools are on PATH (they ship with a ROS install / RoboStack-pixi, not the uv env).
"""
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from lite_description import ROBOTS_DIR
from lite_description.workflow import robot_model, urdf_to_xacro


def _robots_with_xacro() -> list[Path]:
    return sorted(p.parent.parent for p in ROBOTS_DIR.glob("*/xacro/*.description.xacro"))


def _robots_with_ros2_control() -> list[Path]:
    return sorted(p.parent.parent for p in ROBOTS_DIR.glob("*/cad/ros2_control.json"))


ROBOT_DIRS = _robots_with_xacro()
ROBOT_IDS = [p.name for p in ROBOT_DIRS]
RC_DIRS = _robots_with_ros2_control()
RC_IDS = [p.name for p in RC_DIRS]

# The four backend selections the description supports. Only the real one emits
# per-bus blocks; the rest collapse to a single combined component.
BACKENDS = [
    [],
    ["use_mock_hardware:=false"],
    ["sim_mujoco:=true"],
    ["use_mock_hardware:=false", "sim_mujoco:=true"],
]
BACKEND_IDS = ["mock", "real", "sim", "sim_over_real"]


@pytest.mark.skipif(not ROBOT_DIRS, reason="no generated xacro robots")
@pytest.mark.parametrize("robot_dir", ROBOT_DIRS, ids=ROBOT_IDS)
def test_xacro_files_well_formed(robot_dir):
    files = list((robot_dir / "xacro").glob("*.xacro"))
    assert files, f"{robot_dir.name} has no xacro files"
    for path in files:
        ET.parse(path)  # raises on malformed XML


@pytest.mark.skipif(not ROBOT_DIRS, reason="no generated xacro robots")
@pytest.mark.parametrize("robot_dir", ROBOT_DIRS, ids=ROBOT_IDS)
def test_committed_xacro_matches_generator(robot_dir):
    """The committed xacro must equal a fresh generation (no stale / hand edits)."""
    robot = robot_dir.name
    cad = robot_dir / "cad"
    hub = robot_dir / "urdf" / f"{robot}.urdf"
    joint_properties = json.loads((cad / "joint_properties.json").read_text())
    rc_path = cad / "ros2_control.json"
    cfg = json.loads(rc_path.read_text()) if rc_path.exists() else None

    tree = robot_model.parse(hub)
    limits = robot_model.joint_limits(tree.getroot())
    base_link = (cfg or {}).get("base_link") or {
        "name": "base_link",
        "child": robot_model.root_link(tree.getroot()),
    }

    expected = {
        f"{robot}.description.xacro": urdf_to_xacro.build_description_xacro(
            hub, robot, joint_properties, base_link
        ),
        f"{robot}.urdf.xacro": urdf_to_xacro.build_assembly_xacro(robot, cfg),
    }
    if cfg is not None:
        expected[f"{robot}.ros2_control.xacro"] = urdf_to_xacro.build_ros2_control_xacro(
            robot, cfg, limits
        )

    for filename, content in expected.items():
        committed = (robot_dir / "xacro" / filename).read_text()
        assert committed == content, (
            f"{robot}/{filename} is stale -- rerun `lite-description-generate {robot}`"
        )


def _biped_config():
    """Return (config, limits) for lite_biped, the variant with an IMU and a four-bar."""
    robot_dir = ROBOTS_DIR / "lite_biped"
    cfg = json.loads((robot_dir / "cad" / "ros2_control.json").read_text())
    root = robot_model.parse(robot_dir / "urdf" / "lite_biped.urdf").getroot()
    return cfg, robot_model.joint_limits(root)


@pytest.mark.skipif(not (ROBOTS_DIR / "lite_biped").is_dir(), reason="no lite_biped")
def test_group_without_joints_is_rejected():
    """A group with no joints would emit a bus block calling an undefined macro."""
    cfg, limits = _biped_config()
    cfg["groups"].append(
        {"name": "neck", "block_name": "LiteBipedNeck", "can_interface_arg": "can_interface_left"})
    with pytest.raises(ValueError, match="list no joints"):
        urdf_to_xacro.build_ros2_control_xacro("lite_biped", cfg, limits)


@pytest.mark.skipif(not (ROBOTS_DIR / "lite_biped").is_dir(), reason="no lite_biped")
def test_linkage_without_use_linkage_arg_is_rejected():
    """The four-bar macros reference ${use_linkage}, so the arg must be declared."""
    cfg, limits = _biped_config()
    del cfg["args"]["use_linkage"]
    with pytest.raises(ValueError, match="use_linkage"):
        urdf_to_xacro.build_ros2_control_xacro("lite_biped", cfg, limits)


@pytest.mark.skipif(not (ROBOTS_DIR / "lite_biped").is_dir(), reason="no lite_biped")
def test_partial_linkage_is_rejected():
    """humanoid_devices_robstride aborts on some-but-not-all of the four bar lengths."""
    cfg, limits = _biped_config()
    for joint in cfg["joints"]:
        if "linkage" in joint:
            del joint["linkage"]["sign"]
    with pytest.raises(ValueError, match="linkage is missing"):
        urdf_to_xacro.build_ros2_control_xacro("lite_biped", cfg, limits)


@pytest.mark.skipif(not (ROBOTS_DIR / "lite_biped").is_dir(), reason="no lite_biped")
def test_imu_without_real_plugin_emits_no_sensor_component():
    """A sim-only IMU is backed by MujocoSystem, so real hardware gets no sensor block."""
    cfg, limits = _biped_config()
    del cfg["imu"]["real_plugin"]
    del cfg["imu"]["real_block_name"]
    del cfg["args"]["imu_port"]
    text = urdf_to_xacro.build_ros2_control_xacro("lite_biped", cfg, limits)
    assert 'type="sensor"' not in text
    assert "imu_port" not in text
    # The combined block still carries the sim sensor.
    assert f'<sensor name="{cfg["imu"]["name"]}">' in text


@pytest.mark.skipif(not RC_DIRS, reason="no ros2_control robots")
@pytest.mark.parametrize("robot_dir", RC_DIRS, ids=RC_IDS)
def test_ros2_control_structure(robot_dir):
    robot = robot_dir.name
    cfg = json.loads((robot_dir / "cad" / "ros2_control.json").read_text())
    text = (robot_dir / "xacro" / f"{robot}.ros2_control.xacro").read_text()

    # One joint instantiation per configured joint. A joint that declares a four-bar
    # uses the linkage macro instead of the plain one.
    plain = sum(1 for j in cfg["joints"] if not j.get("linkage"))
    linkage = len(cfg["joints"]) - plain
    assert text.count(f"<xacro:{robot}_joint ") == plain
    assert text.count(f"<xacro:{robot}_linkage_joint ") == linkage
    for plugin in cfg["backends"].values():
        assert plugin in text, f"missing backend {plugin}"
    for group in cfg["groups"]:
        assert group["block_name"] in text
    for joint in cfg["joints"]:
        assert f'can_id="{joint["can_id"]}"' in text


@pytest.mark.skipif(not RC_DIRS, reason="no ros2_control robots")
@pytest.mark.parametrize("robot_dir", RC_DIRS, ids=RC_IDS)
def test_backend_args_follow_ros2_control_naming(robot_dir):
    """The backend switches use the current ros2_control / Universal Robots names.

    `use_fake_hardware` is the pre-Iron spelling, and a bare `use_sim` shadows the
    unrelated `use_sim_time` node parameter.
    """
    cfg = json.loads((robot_dir / "cad" / "ros2_control.json").read_text())
    args = cfg["args"]
    assert urdf_to_xacro.MOCK_ARG in args
    assert urdf_to_xacro.SIM_ARG in args
    assert "use_fake_hardware" not in args
    assert "use_sim" not in args


@pytest.mark.skipif(not RC_DIRS, reason="no ros2_control robots")
@pytest.mark.parametrize("robot_dir", RC_DIRS, ids=RC_IDS)
def test_joint_macros_are_backend_agnostic(robot_dir):
    """No backend switch reaches a <joint>.

    Every backend ignores the params it does not know -- MujocoSystem reads only `mimic`
    and `multiplier`, and mock_components/GenericSystem stores the rest -- so the hardware
    params are emitted unconditionally. The switches select a plugin and nothing else.
    """
    robot = robot_dir.name
    text = (robot_dir / "xacro" / f"{robot}.ros2_control.xacro").read_text()
    joint_macros = text.split("_ros2_control_combined")[0]
    for arg in (urdf_to_xacro.MOCK_ARG, urdf_to_xacro.SIM_ARG):
        assert arg not in joint_macros, f"{arg} leaked into the joint macros"
    assert "xacro:unless" not in joint_macros
    assert "xacro:if" not in joint_macros


@pytest.mark.skipif(not RC_DIRS, reason="no ros2_control robots")
@pytest.mark.parametrize("robot_dir", RC_DIRS, ids=RC_IDS)
def test_ros2_control_limits_sourced_from_urdf(robot_dir):
    """lower/upper limits in ros2_control come from the URDF, not ros2_control.json."""
    robot = robot_dir.name
    cfg = json.loads((robot_dir / "cad" / "ros2_control.json").read_text())
    # ros2_control.json must NOT carry position limits (single-sourcing rule)
    for joint in cfg["joints"]:
        assert "lower_limit" not in joint and "upper_limit" not in joint

    limits = robot_model.joint_limits(robot_model.parse(robot_dir / "urdf" / f"{robot}.urdf").getroot())
    text = (robot_dir / "xacro" / f"{robot}.ros2_control.xacro").read_text()
    sample = cfg["joints"][0]["name"]
    assert f'lower_limit="{limits[sample]["lower"]}"' in text


@pytest.mark.skipif(not ROBOT_DIRS, reason="no generated xacro robots")
@pytest.mark.parametrize("robot_dir", ROBOT_DIRS, ids=ROBOT_IDS)
def test_generated_files_carry_autogen_banner(robot_dir):
    """Every generated URDF / MJCF / xacro carries the 'do not edit' banner."""
    robot = robot_dir.name
    banner = robot_model.autogen_comment(robot).strip()
    targets = [
        robot_dir / "urdf" / f"{robot}.urdf",
        robot_dir / "mjcf" / f"{robot}.xml",
        *sorted((robot_dir / "xacro").glob("*.xacro")),
    ]
    for path in targets:
        assert path.exists(), f"missing generated file {path}"
        assert banner in path.read_text(), f"{path} is missing the autogen banner"


@pytest.mark.skipif(not ROBOT_DIRS, reason="no generated xacro robots")
@pytest.mark.parametrize("robot_dir", ROBOT_DIRS, ids=ROBOT_IDS)
def test_description_has_base_link_and_mesh_root(robot_dir):
    robot = robot_dir.name
    desc = (robot_dir / "xacro" / f"{robot}.description.xacro").read_text()
    assert f'xacro:macro name="{robot}_description"' in desc
    assert 'name="base_link"' in desc
    assert "${mesh_root}/" in desc


@pytest.mark.skipif(shutil.which("xacro") is None or shutil.which("check_urdf") is None,
                    reason="xacro/check_urdf not installed (ROS / RoboStack-pixi only)")
@pytest.mark.parametrize("robot_dir", ROBOT_DIRS, ids=ROBOT_IDS)
@pytest.mark.parametrize("backend", BACKENDS, ids=BACKEND_IDS)
def test_xacro_expands_and_check_urdf(robot_dir, backend, tmp_path):
    """Expand the top assembly and validate with check_urdf (the ros2_control_demos test)."""
    robot = robot_dir.name
    assembly = robot_dir / "xacro" / f"{robot}.urdf.xacro"
    out = tmp_path / f"{robot}.urdf"
    expanded = subprocess.run(
        ["xacro", str(assembly), *backend], capture_output=True, text=True)
    assert expanded.returncode == 0, expanded.stderr
    out.write_text(expanded.stdout)
    checked = subprocess.run(["check_urdf", str(out)], capture_output=True, text=True)
    assert checked.returncode == 0, checked.stderr


@pytest.mark.skipif(shutil.which("xacro") is None, reason="xacro not installed")
@pytest.mark.parametrize("robot_dir", RC_DIRS, ids=RC_IDS)
def test_real_backend_emits_one_block_per_bus(robot_dir):
    """The real backend must emit a <ros2_control> block per bus, plus one per sensor.

    The other three backends collapse to a single combined block, so only this one
    exercises the per-bus macro that every hardware bring-up depends on.
    """
    robot = robot_dir.name
    cfg = json.loads((robot_dir / "cad" / "ros2_control.json").read_text())
    expected = len(cfg["groups"]) + (1 if cfg.get("imu") else 0)
    expanded = subprocess.run(
        ["xacro", str(robot_dir / "xacro" / f"{robot}.urdf.xacro"), "use_mock_hardware:=false"],
        capture_output=True, text=True)
    assert expanded.returncode == 0, expanded.stderr
    names = [e.get("name") for e in ET.fromstring(expanded.stdout).iter("ros2_control")]
    assert len(names) == expected, f"{robot}: expected {expected} blocks, got {names}"
    assert len(set(names)) == len(names), f"{robot}: duplicate component names {names}"


@pytest.mark.skipif(shutil.which("xacro") is None, reason="xacro not installed")
@pytest.mark.parametrize("robot_dir", RC_DIRS, ids=RC_IDS)
def test_component_names_match_across_backends(robot_dir):
    """Every backend must name its components the same.

    Anything keyed by component name means a different thing otherwise: a
    controller's `safety_components`, the hardware_spawner, the manager's
    `hardware_components_initial_state`. A safety monitor naming the real
    components simply would not load in simulation, which is where it can be
    tested without a robot.

    The real backend may carry components the others cannot back -- an IMU has no
    source under mock -- so mock is checked as a subset, sim for equality.
    """
    robot = robot_dir.name
    assembly = robot_dir / "xacro" / f"{robot}.urdf.xacro"

    def components(*args):
        out = subprocess.run(["xacro", str(assembly), *args],
                             capture_output=True, text=True)
        assert out.returncode == 0, out.stderr
        return {e.get("name") for e in ET.fromstring(out.stdout).iter("ros2_control")}

    real = components("use_mock_hardware:=false")
    sim = components("sim_mujoco:=true")
    mock = components()

    assert sim == real, f"{robot}: sim {sorted(sim)} != real {sorted(real)}"
    assert mock <= real, f"{robot}: mock {sorted(mock)} is not a subset of real {sorted(real)}"


@pytest.mark.skipif(shutil.which("xacro") is None, reason="xacro not installed")
@pytest.mark.parametrize("robot_dir", RC_DIRS, ids=RC_IDS)
def test_mock_exports_the_safety_interfaces(robot_dir):
    """Mock declares safety_level and safety_flags per component as a <gpio>.

    The real drivers export them from
    export_unlisted_state_interface_descriptions(), which mock_components has no
    equivalent for, so without the gpio a safety monitor cannot be exercised off
    the robot. MuJoCo is excluded on purpose: our MujocoSystem overrides the
    deprecated by-value export_state_interfaces(), which bypasses the base
    class's gpio handling, so declaring them there would promise interfaces that
    never appear.
    """
    robot = robot_dir.name
    out = subprocess.run(
        ["xacro", str(robot_dir / "xacro" / f"{robot}.urdf.xacro")],
        capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    root = ET.fromstring(out.stdout)
    for block in root.iter("ros2_control"):
        if block.get("type") != "system":
            continue
        gpios = {g.get("name"): {si.get("name") for si in g.iter("state_interface")}
                 for g in block.iter("gpio")}
        assert block.get("name") in gpios, (
            f'{robot}: mock component {block.get("name")} declares no safety gpio')
        assert {"safety_level", "safety_flags"} <= gpios[block.get("name")], (
            f'{robot}: {block.get("name")} gpio is missing a safety interface')

