# Generation

A generator makes the URDF, the MJCF, the xacro and the meshes of each variant from the Onshape CAD. Each variant is an assembly and a configuration of the Onshape document `e9ee61a2e2678af2088d9f31`. `cad/config.json` sets them.

## Run the generator

Run the generator from a checkout. The installed package does not have the `cad/` inputs.

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

The `onshape` stage costs about 1000 Onshape API requests. Thus it runs only if the URDF is missing, with `--force`, or with `--only onshape`.

The `onshape` stage needs OpenSCAD to make the colliders. Install it with `sudo apt install openscad`. Without OpenSCAD, the export continues with mesh colliders.

The optional sole hull has its own command. The command reads the shoe mesh from the MJCF and writes `meshes/collision/`:

```bash
uv run lite-description-foot-hull lite_biped
```

## Inputs

The inputs of each variant are in `robots/<variant>/cad/`:

- `config.json`: the Onshape document and the export options.
- `joint_properties.json`: the armature, friction loss and effort limit of each joint.
- `physics.json`: the MuJoCo `<option>`, and the opt-in free joint, base IMU, foot contact model and contact excludes.
- `ros2_control.json`: the hardware map. A variant without this file gets a model-only xacro.
- `scad/`: the collider sources.

## Edit colliders

1. Export the Onshape assets to `cad/assets/`:

   ```bash
   uv run lite-description-onshape-to-urdf lite_dummy --keep-assets
   ```

2. Edit a shape. The editor writes `chest.scad` in `cad/assets/`:

   ```bash
   cd lite_description/robots/lite_dummy/cad/assets/
   uv run onshape-to-robot-edit-shape ./chest.stl
   ```

3. Copy the scad file to `cad/scad/`. Git ignores `cad/assets/`, and the next export overwrites it:

   ```bash
   cp chest.scad ../scad/
   ```

4. Make the URDF again from the saved export, without the Onshape API. This command also deletes `cad/assets/`:

   ```bash
   uv run lite-description-onshape-to-urdf lite_dummy --convert
   ```

5. Run the other stages:

   ```bash
   uv run lite-description-generate lite_dummy
   ```

6. [Render the images](#render-the-images) again.

## Render the images

After a change to the meshes or the colliders, render the images of the README again:

```bash
MUJOCO_GL=egl uv run --group docs python -m lite_description.workflow.render_images
```

## Run the tests

```bash
uv run --group test pytest
```

The tests check these properties of each variant:

- the left/right symmetry
- the URDF/MJCF parity
- the mass properties
- the match of the committed xacro with the generator
- the match of the visual mesh files with the URDF

The xacro expansion tests need ROS. Thus `uv run --group test pytest` skips them, and the ROS job of the CI runs them.

## Design notes

- The committed URDF is the kinematic hub. The `mjcf` and `xacro` stages derive from it, so the formats cannot drift apart.
- The URDF has no `base_link`. Only the description xacro adds it, for ROS and KDL.
- The MJCF has sensors only if `physics.json` asks for the base IMU. `mujoco_ros2_control` aborts on a sensor that is not on a `<site>`.
- The `mjcf/<variant>_scene.xml` files of the bipeds are hand-written. The generator does not make them.
- Each variant has its own `meshes/` directory, because each variant is a separate Onshape export. Thus the variants hold duplicate copies of shared parts: 82 of the 211 visual meshes are duplicates.
- The ament package installs each `robots/<variant>/` except `cad/`. The wheel also leaves out `cad/`.
