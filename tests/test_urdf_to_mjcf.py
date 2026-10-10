"""Check the committed MJCFs against the rules of the mjcf stage."""
import json

import mujoco
import pytest

from lite_description import ROBOTS_DIR, VARIANTS, get_mjcf_path
from lite_description.workflow.urdf_to_mjcf import COLLISION_GROUP

# The MJCF of each variant, and each scene that includes one.
MJCF_FILES = sorted(ROBOTS_DIR.glob("*/mjcf/*.xml"))
MJCF_IDS = [path.stem for path in MJCF_FILES]


@pytest.mark.parametrize("mjcf_path", MJCF_FILES, ids=MJCF_IDS)
def test_collision_geoms_in_group_3_without_cylinders(mjcf_path):
    """Only the scene, such as the floor, is in group 0, so a ray cast for the terrain
    in group 0 never hits the robot."""
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    for i in range(model.ngeom):
        if model.geom_contype[i] == 0 and model.geom_conaffinity[i] == 0:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        expected = 0 if model.geom_bodyid[i] == 0 else COLLISION_GROUP
        assert model.geom_group[i] == expected, f"{name!r} is in group {model.geom_group[i]}"
        assert model.geom_type[i] != mujoco.mjtGeom.mjGEOM_CYLINDER, f"{name!r} is a cylinder"


@pytest.mark.parametrize("mjcf_path", MJCF_FILES, ids=MJCF_IDS)
def test_sensors_are_on_sites(mjcf_path):
    """mujoco_ros2_control looks up the site of each sensor and aborts on a sensor without one."""
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        assert model.sensor_objtype[i] == mujoco.mjtObj.mjOBJ_SITE, f"sensor {name!r} is not on a site"


@pytest.mark.parametrize("variant", VARIANTS)
def test_actuators_are_position_actuators_without_ctrlrange(variant):
    """An RL policy can command a setpoint past the joint limit to hold a saturated torque."""
    model = mujoco.MjModel.from_xml_path(str(get_mjcf_path(variant)))
    assert model.nu == sum(model.jnt_type != mujoco.mjtJoint.mjJNT_FREE)
    assert (model.actuator_biastype == mujoco.mjtBias.mjBIAS_AFFINE).all()
    assert not model.actuator_ctrllimited.any()
    assert (model.actuator_forcerange[:, 0] == -model.actuator_forcerange[:, 1]).all()


@pytest.mark.parametrize("variant", VARIANTS)
def test_physics_excludes(variant):
    pairs = json.loads((ROBOTS_DIR / variant / "cad" / "physics.json").read_text()).get("exclude", [])
    model = mujoco.MjModel.from_xml_path(str(get_mjcf_path(variant)))
    # MuJoCo stores each exclude as (smaller body id << 16) + larger body id.
    signatures = set(model.exclude_signature.tolist())
    for body1, body2 in pairs:
        ids = sorted(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body) for body in (body1, body2))
        assert ids[0] >= 0, f"no body {body1!r} or {body2!r}"
        assert (ids[0] << 16) + ids[1] in signatures, f"{body1} <-> {body2} is not excluded"
