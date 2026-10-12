# Colliders

## Design

A part has a collider only if `cad/scad/` has a scad file with the name of the part. The export removes the mesh collider of each part. Thus many links have no collider, for example the ankle-yaw, ankle-pitch, wrist-yaw, wrist-roll and finger links. The scad file of each collider tells why the collider has its shape. In most cases, the collider is one capsule. A segment has the same collider in each variant.

The shoe of `lite_biped`, `lite` and `lite_pro` is seven capsules of 8 mm radius along the sole. The five capsules under the flat sole are flush with it, 65 mm below the foot origin. The last two capsules follow the toe spring. `lite_biped_debug` keeps its flat debug box.

MuJoCo ignores the contacts of a body with its parent, but not with bodies farther down the chain. At rest, the hip-motor capsule of `pelvis` overlaps the butt capsule of each `<side>_hip_roll`. Thus `physics.json` excludes those contacts.

## URDF cylinders and MJCF capsules

A URDF has no capsule type, so the URDF colliders are cylinders. The MJCF stage converts each cylinder to a capsule with the same radius and the same cylinder length. The capsule thus extends one radius past each end of the cylinder. For example, the `left_knee_pitch` collider is 0.304 m long in the URDF and 0.392 m long in the MJCF.

A URDF consumer, for example Isaac Lab, gets the cylinders. Its contacts at the collider ends differ from the MuJoCo contacts.

## Contact model of the bipeds

The MJCF of `lite_biped` and `lite_biped_debug` applies the contact model of mjlab:

- Each collider has the name `<body>_collision_<i>`. For example, the sole capsules are `left_foot_collision_0` to `left_foot_collision_6`.
- The foot colliders have `condim` 3, friction 0.6 and priority 1. Priority 1 makes the foot friction win over the friction of the ground.
- The other colliders have `condim` 1. Thus a contact between two of these colliders is frictionless, and a leg that touches the other leg slides and does not catch.

A contact between two geoms with the same priority uses the larger `condim` and friction of the two geoms. The ground has `condim` 3 and friction 1.0, the MuJoCo defaults. Thus a leg that touches the ground has friction 1.0 and does not slide. The ground of mjlab training and the floor of the scene files both keep the MuJoCo defaults, so the scene files give the same contacts as training.

The colliders of the other variants have no names and keep the MuJoCo defaults.

## Select colliders at load time

The repository has one collider set for each variant. To use fewer colliders, change the contact flags at load time. In MuJoCo, `contype` and `conaffinity` select the contacts. The geom `group` does not select contacts.

For example, this mjlab config keeps only the contacts of the feet with the ground. The config works only for the bipeds, because only their colliders have names.

```python
from mjlab.utils.spec_config import CollisionCfg

FEET_ONLY_COLLISION = CollisionCfg(
    geom_names_expr=(r"^(left|right)_foot_collision_\d+$",),
    contype=0,
    conaffinity=1,
    condim=3,
    priority=1,
    friction=(0.6,),
)
```

## Optional sole hull

`lite_biped` also has an optional convex sole hull with a flat bottom, `meshes/collision/<side>_foot_hull.stl`, in the frame of the foot body. The MJCF does not reference the hull. To collide through the hull, replace the capsules of each foot with it:

```python
import mujoco
from lite_description import get_foot_hull_path, get_mjcf_path

spec = mujoco.MjSpec.from_file(str(get_mjcf_path("lite_biped")))
for side in ("left", "right"):
    foot = spec.body(f"{side}_foot")
    for geom in [g for g in foot.geoms if g.contype or g.conaffinity]:
        spec.delete(geom)
    spec.add_mesh(name=f"{side}_foot_hull", file=str(get_foot_hull_path("lite_biped", side)))
    # The contact parameters of the lite_biped sole capsules.
    foot.add_geom(name=f"{side}_foot_collision_0", type=mujoco.mjtGeom.mjGEOM_MESH,
                  meshname=f"{side}_foot_hull", group=3, condim=3, priority=1,
                  friction=[0.6, 0.005, 0.0001])
```

The foot bodies have their own inertials, so the swap does not change the mass properties.
