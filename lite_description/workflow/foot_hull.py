"""Build the optional sole hull of a shoe: ``meshes/collision/<side>_foot_hull.stl``.

The hull is the convex hull of the shoe up to 25 mm above the sole, with 32 vertices. The
MJCF does not reference it. MuJoCo collides a mesh through its convex hull. A plain
simplified hull puts part of the sole on a facet that tilts by up to 3 deg, because an
outline vertex is a fraction of a millimetre off the sole plane. Thus this module puts the
outline of the flat sole on the plane as one 10-vertex polygon. Then it adds the other 22
vertices from the shoe, farthest out first, as the incremental build of qhull does. The
right hull is the mirror of the left hull.

Usage:
  lite-description-foot-hull lite_biped
"""

import argparse
from pathlib import Path

import numpy as np

try:
    import mujoco
except ModuleNotFoundError as error:
    raise ModuleNotFoundError(
        "MuJoCo is needed to build the foot hull, and ships with the CAD toolchain rather than "
        "the base install. Install it with `uv sync --extra cad`."
    ) from error

from . import robot_model

VERTICES = 32  # all hull vertices
BOTTOM_VERTICES = 10  # of them on the sole plane
SOLE_REGION = 0.025  # m above the sole plane that the hull covers
FLAT_TOLERANCE = 0.0007  # m: a face this close to the plane is part of the flat sole
DOWN_FACING = 0.9  # minimum y of the unit normal of a sole face (+y is down)


def geom_rotation(model: mujoco.MjModel, geom: int) -> np.ndarray:
    """Return the rotation of ``geom`` in its body frame, shape (3, 3)."""
    rotation = np.zeros(9)
    mujoco.mju_quat2Mat(rotation, model.geom_quat[geom])
    return rotation.reshape(3, 3)


def mesh_geom_vertices(model: mujoco.MjModel, geom: int) -> np.ndarray:
    """Return the mesh vertices of ``geom`` in its body frame, shape (n, 3)."""
    mesh = model.geom_dataid[geom]
    start = model.mesh_vertadr[mesh]
    vertices = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
    return vertices @ geom_rotation(model, geom).T + model.geom_pos[geom]


def shoe_triangles(mjcf: Path) -> np.ndarray:
    """Return the visual triangles of the left foot in the foot frame, shape (n, 3, 3)."""
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_foot")
    triangles = []
    for geom in range(model.ngeom):
        is_mesh = model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH
        is_visual = model.geom_contype[geom] == 0 and model.geom_conaffinity[geom] == 0
        if model.geom_bodyid[geom] != body or not (is_mesh and is_visual):
            continue
        mesh = model.geom_dataid[geom]
        start = model.mesh_faceadr[mesh]
        faces = model.mesh_face[start:start + model.mesh_facenum[mesh]]
        triangles.append(mesh_geom_vertices(model, geom)[faces])
    return np.vstack(triangles).astype(float)


def hull_faces(points: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    """Return the faces of the convex hull of ``points`` as (vertex indices, outward normal).

    MuJoCo builds the hull with qhull and merges coplanar triangles into polygons. It
    stores the mesh recentred and turned to its principal axes, so the geom pose maps the
    polygons back onto ``points``.
    """
    spec = mujoco.MjSpec()
    spec.add_mesh(name="hull", uservert=points.ravel().tolist())
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="hull")
    model = spec.compile()
    rotation = geom_rotation(model, 0)
    vertices = mesh_geom_vertices(model, 0)
    index = {}  # MuJoCo vertex -> the input point it came from (MuJoCo stores float32)

    def point(vertex: int) -> int:
        if vertex not in index:
            index[vertex] = int(np.argmin(np.linalg.norm(points - vertices[vertex], axis=1)))
        return index[vertex]

    faces = []
    for polygon in range(model.mesh_polynum[0]):
        start = model.mesh_polyvertadr[polygon]
        ring = model.mesh_polyvert[start:start + model.mesh_polyvertnum[polygon]]
        normal = rotation @ model.mesh_polynormal[polygon]
        faces.append((np.array([point(v) for v in ring]), normal))
    return faces


def cross2(a: np.ndarray, b: np.ndarray) -> float:
    """Return the z component of the cross product of two 2-D vectors."""
    return a[0] * b[1] - a[1] * b[0]


def convex_ring(points: np.ndarray) -> np.ndarray:
    """Return the 2-D convex hull of ``points`` in counter-clockwise order (monotone chain)."""
    points = np.unique(points, axis=0)

    def half(sequence):
        chain = []
        for point in sequence:
            while len(chain) >= 2 and cross2(chain[-1] - chain[-2], point - chain[-2]) <= 0:
                chain.pop()
            chain.append(point)
        return chain[:-1]

    return np.array(half(points) + half(points[::-1]))


def flat_bottom_hull(triangles: np.ndarray) -> np.ndarray:
    """Return the vertices of the flat-bottomed sole hull (foot frame: +y down, +z forward)."""
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-15)
    vertices = triangles.reshape(-1, 3)
    sole = vertices[:, 1].max()

    # The bottom: the outline of the flat sole faces, on the sole plane. Each step removes
    # the outline vertex whose removal loses the smallest area.
    on_sole = triangles[:, :, 1].min(axis=1) > sole - FLAT_TOLERANCE
    flat = (normals[:, 1] > DOWN_FACING) & on_sole
    outline = list(convex_ring(triangles[flat].reshape(-1, 3)[:, [0, 2]]))
    while len(outline) > BOTTOM_VERTICES:
        loss = []
        for i in range(len(outline)):
            before, after = outline[i - 1], outline[(i + 1) % len(outline)]
            loss.append(abs(cross2(outline[i] - before, after - before)) / 2)
        outline.pop(int(np.argmin(loss)))
    points = [np.array([x, sole, z]) for x, z in outline]

    # The other vertices come from the shoe vertices in the sole region that are clear of
    # the plane. Only the vertices on the hull of that region can be farthest out.
    above = (vertices[:, 1] > sole - SOLE_REGION) & (vertices[:, 1] < sole - FLAT_TOLERANCE)
    region = np.unique(vertices[above], axis=0)
    region = region[np.unique(np.concatenate([face for face, _ in hull_faces(region)]))]
    # First the extreme vertex along each axis, but not straight down (+y). Then, each
    # time, the vertex farthest outside the hull so far.
    for direction in np.vstack([np.eye(3), -np.eye(3)]):
        candidate = region[np.argmax(region @ direction)]
        if direction[1] <= 0 and not any(np.allclose(candidate, point) for point in points):
            points.append(candidate)
    while len(points) < VERTICES:
        hull = np.array(points)
        distance = np.full(len(region), -np.inf)
        for ring, normal in hull_faces(hull):
            distance = np.maximum(distance, (region - hull[ring[0]]) @ normal)
        best = int(np.argmax(distance))
        if distance[best] < 1e-4:
            break
        points.append(region[best])
    return np.array(points)


def write_stl(path: Path, points: np.ndarray) -> None:
    """Write the convex hull of ``points`` as a binary STL, each polygon as a triangle fan."""
    normals, corners = [], []
    for ring, normal in hull_faces(points):
        for i in range(1, len(ring) - 1):
            triangle = points[[ring[0], ring[i], ring[i + 1]]]
            if np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0]) @ normal < 0:
                triangle = triangle[[0, 2, 1]]
            normals.append(normal)
            corners.append(triangle)
    record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    data = np.zeros(len(corners), record)
    data["normal"] = normals
    data["vertices"] = corners
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(b"lite_description foot hull".ljust(80, b" "))
        stream.write(np.uint32(len(data)).tobytes())
        stream.write(data.tobytes())


def generate(robot_dir: Path) -> list[Path]:
    """Write ``meshes/collision/{left,right}_foot_hull.stl`` for one ``robots/<robot>/`` dir."""
    left = flat_bottom_hull(shoe_triangles(robot_dir / "mjcf" / f"{robot_dir.name}.xml"))
    written = []
    for side, mirror in (("left", 1.0), ("right", -1.0)):
        path = robot_dir / "meshes" / "collision" / f"{side}_foot_hull.stl"
        write_stl(path, left * np.array([mirror, 1.0, 1.0]))
        written.append(path)
    return written


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Build the optional sole hull of a robot's shoes.")
    parser.add_argument("robot", help="Variant name (under lite_description/robots/) or a path to a robot dir.")
    args = parser.parse_args(argv)
    for path in generate(robot_model.resolve_robot_dir(args.robot)):
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
