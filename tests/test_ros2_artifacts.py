"""Check the generated xacro of each variant.

The tests that expand the xacro need ``xacro`` and ``check_urdf`` from ROS. The ROS job of
the CI runs them.
"""
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

from lite_description import ROBOTS_DIR, VARIANTS
from lite_description.workflow import robot_model, urdf_to_xacro

ROS2_CONTROL_VARIANTS = sorted(path.parent.parent.name for path in ROBOTS_DIR.glob("*/cad/ros2_control.json"))

# The xacro args of each backend: mock (the default), real, sim, and sim over real.
BACKENDS = {
    "mock": [],
    "real": ["use_mock_hardware:=false"],
    "sim": ["sim_mujoco:=true"],
    "sim_over_real": ["use_mock_hardware:=false", "sim_mujoco:=true"],
}
requires_xacro = pytest.mark.skipif(
    shutil.which("xacro") is None or shutil.which("check_urdf") is None,
    reason="xacro and check_urdf come with ROS",
)


def ros2_control(variant: str) -> dict:
    return json.loads((ROBOTS_DIR / variant / "cad" / "ros2_control.json").read_text())


def joint_limits(variant: str) -> dict:
    return robot_model.joint_limits(robot_model.parse(ROBOTS_DIR / variant / "urdf" / f"{variant}.urdf").getroot())


def ros2_control_xacro(variant: str) -> str:
    return (ROBOTS_DIR / variant / "xacro" / f"{variant}.ros2_control.xacro").read_text()


def expand(variant: str, args: list[str]) -> str:
    """Return the URDF that the assembly xacro of ``variant`` expands to."""
    assembly = ROBOTS_DIR / variant / "xacro" / f"{variant}.urdf.xacro"
    expanded = subprocess.run(["xacro", str(assembly), *args], capture_output=True, text=True)
    assert expanded.returncode == 0, expanded.stderr
    return expanded.stdout


@pytest.mark.parametrize("variant", VARIANTS)
def test_committed_xacro_matches_generator(variant):
    for name, text in urdf_to_xacro.build(ROBOTS_DIR / variant).items():
        ET.fromstring(text.encode())
        committed = (ROBOTS_DIR / variant / "xacro" / name).read_text()
        assert committed == text, f"{name} is stale. Run `lite-description-generate {variant} --only xacro`."


@pytest.mark.parametrize("variant", VARIANTS)
def test_generated_files_carry_the_banner(variant):
    robot_dir = ROBOTS_DIR / variant
    banner = robot_model.autogen_comment(variant).strip()
    for path in [robot_dir / "urdf" / f"{variant}.urdf", robot_dir / "mjcf" / f"{variant}.xml",
                 *(robot_dir / "xacro").glob("*.xacro")]:
        assert banner in path.read_text(), f"{path} has no banner"


@pytest.mark.parametrize("variant", VARIANTS)
def test_description_has_base_link_and_mesh_root(variant):
    text = (ROBOTS_DIR / variant / "xacro" / f"{variant}.description.xacro").read_text()
    assert f'xacro:macro name="{variant}_description"' in text
    assert 'name="base_link"' in text
    assert "${mesh_root}/" in text


def test_linkage_without_use_linkage_arg_is_rejected():
    """The four-bar macros read ${use_linkage}, so the arg must exist."""
    config = ros2_control("lite_biped")
    del config["args"]["use_linkage"]
    with pytest.raises(ValueError, match="use_linkage"):
        urdf_to_xacro.build_ros2_control_xacro("lite_biped", config, {})


def test_partial_linkage_is_rejected():
    """humanoid_devices_robstride rejects a part of the four-bar params."""
    config = ros2_control("lite_biped")
    for joint in config["joints"]:
        joint.get("linkage", {}).pop("sign", None)
    with pytest.raises(ValueError, match="linkage is missing"):
        urdf_to_xacro.build_ros2_control_xacro("lite_biped", config, joint_limits("lite_biped"))


@pytest.mark.parametrize("variant", ROS2_CONTROL_VARIANTS)
def test_ros2_control_structure(variant):
    config = ros2_control(variant)
    text = ros2_control_xacro(variant)
    assert {urdf_to_xacro.MOCK_ARG, urdf_to_xacro.SIM_ARG} <= set(config["args"])
    # A joint with a four-bar uses the linkage macro instead of the plain one.
    linkage = sum(1 for joint in config["joints"] if joint.get("linkage"))
    assert text.count(f"<xacro:{variant}_joint ") == len(config["joints"]) - linkage
    assert text.count(f"<xacro:{variant}_linkage_joint ") == linkage
    for plugin in config["backends"].values():
        assert plugin in text
    for group in config["groups"]:
        assert group["block_name"] in text
    for joint in config["joints"]:
        assert f'can_id="{joint["can_id"]}"' in text


@pytest.mark.parametrize("variant", ROS2_CONTROL_VARIANTS)
def test_joint_macros_are_backend_agnostic(variant):
    """Each backend ignores the params that it does not know, so no switch reaches a joint."""
    joint_macros = ros2_control_xacro(variant).split("_ros2_control_combined")[0]
    for text in (urdf_to_xacro.MOCK_ARG, urdf_to_xacro.SIM_ARG, "xacro:if", "xacro:unless"):
        assert text not in joint_macros


@pytest.mark.parametrize("variant", ROS2_CONTROL_VARIANTS)
def test_position_limits_come_from_the_urdf(variant):
    config = ros2_control(variant)
    for joint in config["joints"]:
        assert "lower_limit" not in joint and "upper_limit" not in joint
    joint = config["joints"][0]["name"]
    assert f'lower_limit="{joint_limits(variant)[joint]["lower"]}"' in ros2_control_xacro(variant)


@requires_xacro
@pytest.mark.parametrize("variant", VARIANTS)
@pytest.mark.parametrize("backend", BACKENDS)
def test_xacro_expands_to_a_valid_urdf(variant, backend, tmp_path):
    urdf = tmp_path / f"{variant}.urdf"
    urdf.write_text(expand(variant, BACKENDS[backend]))
    checked = subprocess.run(["check_urdf", str(urdf)], capture_output=True, text=True)
    assert checked.returncode == 0, checked.stderr


@requires_xacro
@pytest.mark.parametrize("variant", ROS2_CONTROL_VARIANTS)
def test_component_names_match_across_backends(variant):
    """Each backend has one block for each CAN bus, with the same component names.

    A controller's ``safety_components``, the hardware spawner and the manager's
    ``hardware_components_initial_state`` select components by name. The real backend
    also has the IMU, which mock cannot back, so mock is a subset.
    """
    config = ros2_control(variant)
    names = {backend: [block.get("name") for block in ET.fromstring(expand(variant, args)).iter("ros2_control")]
             for backend, args in BACKENDS.items()}
    real = names["real"]
    assert len(real) == len(set(real)) == len(config["groups"]) + ("imu" in config)
    assert set(names["sim"]) == set(real)
    assert set(names["mock"]) <= set(real)


@requires_xacro
@pytest.mark.parametrize("variant", ROS2_CONTROL_VARIANTS)
def test_mock_declares_the_safety_interfaces(variant):
    """Mock declares safety_level and safety_flags of each component as a <gpio>.

    The real drivers export them from export_unlisted_state_interface_descriptions(), and
    mock_components has no equivalent. MuJoCo declares no gpio: MujocoSystem overrides the
    by-value export_state_interfaces(), which skips the gpio handling of the base class.
    """
    for block in ET.fromstring(expand(variant, BACKENDS["mock"])).iter("ros2_control"):
        if block.get("type") == "system":
            gpios = {gpio.get("name"): {i.get("name") for i in gpio.iter("state_interface")}
                     for gpio in block.iter("gpio")}
            assert {"safety_level", "safety_flags"} <= gpios.get(block.get("name"), set())
