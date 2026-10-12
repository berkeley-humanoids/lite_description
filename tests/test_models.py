"""Check the mass properties, joints and meshes of the committed URDF and MJCF of each variant."""
from functools import cache
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from lite_description import VARIANTS, get_mjcf_path, get_urdf_path

MASS_TOLERANCE = 0.01  # kg
# The largest link inertia is about 1e-2 kg m^2, and the median about 1e-4 kg m^2.
INERTIA_TOLERANCE = 1e-5  # kg m^2
# Larger values are almost certainly a unit error.
MAX_MASS = 1000.0  # kg
MAX_INERTIA = 1000.0  # kg m^2


@cache
def load(variant: str, model_format: str) -> tuple[dict, dict]:
    """Return the bodies {name: (mass, inertia)} and the joint ranges {name: (lower, upper)}.

    The URDF inertia is the 6 tensor components, and the MJCF inertia is the 3 principal
    moments. The URDF joints are the joints with both limits, and the MJCF joints are the
    limited joints. The MJCF bodies leave out the world and the bodies without mass.
    """
    bodies, ranges = {}, {}
    if model_format == "urdf":
        root = ET.parse(get_urdf_path(variant)).getroot()
        for link in root.findall("link"):
            inertia = link.find("inertial/inertia")
            components = [float(inertia.get(key)) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")]
            bodies[link.get("name")] = (float(link.find("inertial/mass").get("value")), components)
        for joint in root.findall("joint"):
            limit = joint.find("limit")
            if joint.get("type") != "fixed" and limit is not None:
                ranges[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    else:
        model = mujoco.MjModel.from_xml_path(str(get_mjcf_path(variant)))
        for i in range(1, model.nbody):
            if model.body_mass[i] > 0:
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
                bodies[name] = (float(model.body_mass[i]), model.body_inertia[i].tolist())
        for i in range(model.njnt):
            if model.jnt_limited[i]:
                ranges[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)] = tuple(model.jnt_range[i])
    return bodies, ranges


def left_right_pairs(names) -> list[tuple[str, str]]:
    return [(name, "right_" + name[5:]) for name in sorted(names)
            if name.startswith("left_") and "right_" + name[5:] in names]


def principal_moments(inertia: list[float]) -> np.ndarray:
    if len(inertia) == 3:
        return np.array(inertia)
    ixx, ixy, ixz, iyy, iyz, izz = inertia
    return np.linalg.eigvalsh([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])


@pytest.mark.parametrize("model_format", ("urdf", "mjcf"))
@pytest.mark.parametrize("variant", VARIANTS)
def test_left_and_right_are_mirrors(variant, model_format):
    bodies, ranges = load(variant, model_format)
    for left, right in left_right_pairs(bodies):
        (left_mass, left_inertia), (right_mass, right_inertia) = bodies[left], bodies[right]
        assert abs(left_mass - right_mass) <= MASS_TOLERANCE, f"{left}: {left_mass} kg, {right}: {right_mass} kg"
        # The magnitudes do not change when a mirror flips the sign of a product of inertia.
        difference = np.abs(np.sort(np.abs(left_inertia)) - np.sort(np.abs(right_inertia)))
        assert (difference <= INERTIA_TOLERANCE).all(), f"{left}: {left_inertia}, {right}: {right_inertia}"
    for left, right in left_right_pairs(ranges):
        # A joint range is a design value, so it matches exactly, as is or mirrored.
        (left_lower, left_upper), (right_lower, right_upper) = ranges[left], ranges[right]
        assert ranges[left] == ranges[right] or (left_lower, left_upper) == (-right_upper, -right_lower), (
            f"{left}: {ranges[left]}, {right}: {ranges[right]}"
        )


@pytest.mark.parametrize("model_format", ("urdf", "mjcf"))
@pytest.mark.parametrize("variant", VARIANTS)
def test_mass_properties_are_physical(variant, model_format):
    for name, (mass, inertia) in load(variant, model_format)[0].items():
        assert 0 < mass <= MAX_MASS, f"{name}: {mass} kg"
        # A frame link, such as a fingertip or the IMU, has no inertia.
        if not any(inertia):
            continue
        moments = np.sort(principal_moments(inertia))
        assert 0 < moments[0] and moments[2] <= MAX_INERTIA, f"{name}: principal moments {moments}"
        # The largest principal moment of a rigid body is at most the sum of the other two.
        assert moments[0] + moments[1] + 1e-9 >= moments[2], f"{name}: principal moments {moments}"


@pytest.mark.parametrize("variant", VARIANTS)
def test_urdf_and_mjcf_agree(variant):
    urdf_bodies, urdf_ranges = load(variant, "urdf")
    mjcf_bodies, mjcf_ranges = load(variant, "mjcf")
    urdf_mass = sum(mass for mass, _ in urdf_bodies.values())
    mjcf_mass = sum(mass for mass, _ in mjcf_bodies.values())
    assert abs(urdf_mass - mjcf_mass) <= MASS_TOLERANCE, f"URDF {urdf_mass} kg, MJCF {mjcf_mass} kg"
    assert set(urdf_ranges) == set(mjcf_ranges)


@pytest.mark.parametrize("variant", VARIANTS)
def test_urdf_meshes_match_the_mesh_files(variant):
    """MuJoCo loads each MJCF in the other tests, so only the URDF meshes need a check."""
    urdf = get_urdf_path(variant)
    referenced = {(urdf.parent / mesh.get("filename")).resolve() for mesh in ET.parse(urdf).getroot().iter("mesh")}
    files = {path.resolve() for path in (urdf.parent.parent / "meshes" / "visual").iterdir()}
    assert not referenced - files, f"missing: {sorted(path.name for path in referenced - files)}"
    assert not files - referenced, f"not referenced: {sorted(path.name for path in files - referenced)}"
