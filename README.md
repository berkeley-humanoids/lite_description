# Lite Description

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](LICENSE)

Robot description for the Berkeley Humanoid Lite V2, an open-source humanoid built on
[Robstride](https://robstride.com) actuators. For each variant, this repository holds the
URDF, the MJCF, the xacro and the meshes. A generator makes all of them from the Onshape
CAD, so do not edit them by hand. The `lite_description` Python package serves MuJoCo Lab
and Isaac Lab, and the `lite_description` ament package serves ROS 2.

<!-- TODO: add a render / photo of the Lite robot here. -->

## Variants

| Variant | Description | DoF | Root | `ros2_control` |
|---|---|---|---|---|
| `lite` | full body (legs, 1-DoF waist yaw, arms, neck, 5-finger hands) | 72 | `pelvis` | model-only |
| `lite_pro` | full body **Pro** (legs, 3-DoF waist, arms, neck, 5-finger hands) | 74 | `pelvis` | model-only |
| `lite_dummy` | V1 bimanual upper body (arms + neck), the configuration `Humanoid Control` deploys | 17 | `chest` | Robstride on two CAN buses |
| `lite_bimanual` | V2 bimanual arms, no neck | 14 | `chest` | Robstride on two CAN buses |
| `lite_biped` | V2 legs (hip x3, knee, ankle x3 per leg), floating base | 14 | `pelvis` | Robstride on two CAN buses + base IMU |
| `lite_biped_debug` | `lite_biped` with the flat debug foot instead of the rockered sole | 14 | `pelvis` | Robstride on two CAN buses + base IMU |

Each variant is a `Configuration=` of the same Onshape document, `e9ee61a2e2678af2088d9f31`.

## Hardware backends

A variant with a `ros2_control` block selects its backend with two xacro args, as the
[Universal Robots description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/ros2/urdf/ur.ros2_control.xacro)
does:

| `sim_mujoco` | `use_mock_hardware` | Backend |
|---|---|---|
| `false` | `false` | `humanoid_devices_robstride/RobstrideSystem` |
| `false` | `true` | `mock_components/GenericSystem` |
| `true` | any | `mujoco_ros2_control/MujocoSystem` |

The args select the plugin and nothing else. Each backend has one `<ros2_control>` block
for each CAN bus, with the same component names. The joint macros always emit their
hardware params, because each backend ignores the params that it does not know.
`sim_mujoco` is not related to the `use_sim_time` node parameter, which controls the clock.

## Usage

### Simulation and RL (Python, no ROS toolchain)

```bash
uv add git+https://github.com/Berkeley-Humanoids/Lite-Description.git
```

```python
import mujoco
from lite_description import VARIANTS, get_mjcf_path, get_urdf_path

model = mujoco.MjModel.from_xml_path(str(get_mjcf_path("lite_biped")))  # MuJoCo
urdf_path = get_urdf_path("lite")                                      # Isaac Lab
```

Each function returns a path in the installed package, and a name that is not in
`VARIANTS` raises `ValueError`. A URDF or MJCF finds its meshes by a relative path, so load
the file where it is. The package has no dependencies. The `uv.lock` of your project
records the commit, so the assets change only when you upgrade `lite-description`.
`lite_description.actuators` holds the spec tables of the actuators and the BeyondMimic
formulas for the PD gains.

#### Colliders

Each rigid segment has its own collider, in most cases one capsule, and the same collider
in each variant. The scad file of each segment, in `cad/scad/`, tells why its collider has
its shape. The shoe of `lite_biped`, `lite` and `lite_pro` is seven r8 capsules along the
sole. The five under the flat sole are flush with it, 65 mm below the foot origin, and the
last two follow the toe spring. `lite_biped_debug` keeps its flat debug box. At rest, the
hip-motor capsule of `pelvis` overlaps the butt capsule of each `<side>_hip_roll`, so
`physics.json` excludes those contacts.

`lite_biped` also ships an optional convex sole hull with an exactly flat bottom,
`meshes/collision/<side>_foot_hull.stl`, in the frame of the foot body. The MJCF does not
reference it. To collide through the hull, replace the capsules of each foot with it:

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

The foot bodies hold their own inertials, so the swap does not change the mass properties.

### ROS 2

`lite_description` is an `ament_cmake` package with its `package.xml` at the repo root.
Build it with `colcon build` in a ROS 2 workspace, for example from the
`humanoid_control.repos` of `Humanoid Control`. `robot_state_publisher` runs xacro on
`$(find lite_description)/robots/<variant>/xacro/<variant>.urdf.xacro`, and the mesh paths
use `package://lite_description/robots/<variant>/meshes/visual/`. The package installs
each `robots/<variant>/` except `cad/`.

## Re-generating from CAD

Run the generator from a checkout, because the installed package does not have the `cad/`
inputs:

```bash
uv sync --extra cad
uv run lite-description-generate lite_dummy                    # All stages
uv run lite-description-generate lite_dummy --only mjcf,xacro  # After an edit of physics.json or ros2_control.json
uv run lite-description-generate lite_dummy --force            # Export from Onshape again
```

| Stage | Reads | Writes |
|---|---|---|
| `onshape` | `cad/config.json`, `cad/scad/` | `urdf/<variant>.urdf`, `meshes/visual/` |
| `urdf` | `urdf/<variant>.urdf`, `cad/joint_properties.json` | `urdf/<variant>.urdf`, in place |
| `mjcf` | `urdf/<variant>.urdf`, `cad/joint_properties.json`, `cad/physics.json` | `mjcf/<variant>.xml` |
| `xacro` | `urdf/<variant>.urdf`, `cad/ros2_control.json` | `xacro/<variant>.*.xacro` |

The inputs in `robots/<variant>/cad/` are:

- `config.json`: the Onshape document and the export options.
- `joint_properties.json`: the armature, friction loss and effort limit of each joint.
- `physics.json`: the MuJoCo `<option>`, and the opt-in free joint, base IMU, foot contact
  model and contact excludes.
- `ros2_control.json`: the hardware map. A variant without it gets a model-only xacro.
- `scad/`: the collider sources.

The committed URDF is the kinematic hub. The `mjcf` and `xacro` stages derive from it, so
the formats cannot drift apart. The hub has no `base_link`, and only the description xacro
adds it for ROS and KDL. The `onshape` stage costs about 1000 Onshape API requests, so it
runs only if the hub is missing, with `--force`, or with `--only onshape`. The MJCF has
sensors only if `physics.json` asks for the base IMU, because `mujoco_ros2_control` aborts
on a sensor that is not on a `<site>`.

The optional foot hull has its own command. It reads the shoe mesh from the MJCF and
writes `meshes/collision/`:

```bash
uv run lite-description-foot-hull lite_biped
```

### Editing colliders (OpenSCAD)

Install OpenSCAD with `sudo apt install openscad`. Then export the Onshape assets and open
a shape in the editor:

```bash
uv run lite-description-onshape-to-urdf lite_dummy --keep-assets
cd lite_description/robots/lite_dummy/cad/assets/
uv run onshape-to-robot-edit-shape ./chest.stl
```

## Tests

```bash
uv run pytest
```

The tests check the left/right symmetry, the URDF/MJCF parity and the mass properties of
each variant, and that the committed xacro matches the generator. The xacro expansion
tests need ROS, so `uv run pytest` skips them, and the ROS job of the CI runs them.
