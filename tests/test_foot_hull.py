"""Check the optional foot hull of lite_biped against the capsules that it replaces."""
import mujoco
import numpy as np
import pytest

from lite_description import get_foot_hull_path, get_mjcf_path
from lite_description.workflow.foot_hull import geom_rotation, mesh_geom_vertices


def hull_model(side: str) -> mujoco.MjModel:
    """Load the hull as a training config does: one mesh geom in the foot frame."""
    spec = mujoco.MjSpec()
    spec.add_mesh(name="hull", file=str(get_foot_hull_path("lite_biped", side)))
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="hull")
    return spec.compile()


def sole_plane(side: str) -> float:
    """Return the y of the lowest point of the sole capsules (+y is down)."""
    model = mujoco.MjModel.from_xml_path(str(get_mjcf_path("lite_biped")))
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_foot")
    lowest = -np.inf
    for geom in range(model.ngeom):
        collides = model.geom_contype[geom] or model.geom_conaffinity[geom]
        if model.geom_bodyid[geom] != body or not collides:
            continue
        assert model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_CAPSULE, "the shoe must be capsules"
        axis = geom_rotation(model, geom)[:, 2] * model.geom_size[geom][1]
        ends = model.geom_pos[geom] + np.array([axis, -axis])
        lowest = max(lowest, ends[:, 1].max() + model.geom_size[geom][0])
    return lowest


@pytest.mark.parametrize("side", ("left", "right"))
def test_hull_bottom_is_one_level_face_on_the_sole_plane(side):
    model = hull_model(side)
    normals = model.mesh_polynormal[:model.mesh_polynum[0]] @ geom_rotation(model, 0).T
    level = np.flatnonzero(normals[:, 1] > 1 - 1e-9)
    # One polygon faces down, and no other polygon faces down within 3 deg of it.
    assert len(level) == 1
    assert np.sum(normals[:, 1] > np.cos(np.radians(3))) == 1
    vertices = mesh_geom_vertices(model, 0)
    start = model.mesh_polyvertadr[level[0]]
    bottom = vertices[model.mesh_polyvert[start:start + model.mesh_polyvertnum[level[0]]]]
    assert np.ptp(bottom[:, 1]) < 1e-6
    # The bottom is the lowest part of the hull, at the lowest point of the capsules.
    assert bottom[:, 1].max() == pytest.approx(vertices[:, 1].max(), abs=1e-9)
    assert bottom[0, 1] == pytest.approx(sole_plane(side), abs=1e-4)


def test_hulls_mirror():
    left = mesh_geom_vertices(hull_model("left"), 0)
    right = mesh_geom_vertices(hull_model("right"), 0) * np.array([-1.0, 1.0, 1.0])
    distance = np.linalg.norm(left[:, None] - right[None], axis=2)
    assert distance.min(axis=1).max() < 1e-6


def test_variant_without_a_hull_raises():
    with pytest.raises(FileNotFoundError):
        get_foot_hull_path("lite_biped_debug", "left")
