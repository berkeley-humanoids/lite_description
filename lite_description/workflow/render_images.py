"""Render the README images, ``docs/images/<variant>.png``: the visual meshes (left) and the
colliders (right). Run it from a checkout.

Usage:
  MUJOCO_GL=egl uv run --group docs python -m lite_description.workflow.render_images
"""

from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from lite_description import VARIANTS, get_mjcf_path

IMAGES_DIR = Path(__file__).parents[2] / "docs" / "images"
AZIMUTH = 150  # deg
ELEVATION = -12  # deg
SIZE = 1400  # px, the square render after supersampling
SUPERSAMPLING = 3
HEIGHT = 480  # px, the height of the saved image
COLLIDER_RGBA = (1.0, 0.55, 0.2, 1.0)
# The zero pose holds the arms out to the sides, which makes a wide image. Thus lower the
# arms and point the forearms forward.
POSE = {
    "left_shoulder_roll": -1.35,
    "right_shoulder_roll": 1.35,
    "left_elbow_pitch": -1.4,
    "right_elbow_pitch": -1.4,
}


def render(model: mujoco.MjModel, data: mujoco.MjData, camera: mujoco.MjvCamera, group: int) -> np.ndarray:
    """Return an RGBA image of the geoms in ``group``, with a transparent background."""
    option = mujoco.MjvOption()
    option.geomgroup[:] = 0
    option.geomgroup[group] = 1
    size = SIZE * SUPERSAMPLING
    with mujoco.Renderer(model, size, size) as renderer:
        renderer.update_scene(data, camera, option)
        rgb = renderer.render().astype(np.float32)
        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera, option)
        mask = (renderer.render()[..., 0] >= 0).astype(np.float32)
    rgb = rgb.reshape(SIZE, SUPERSAMPLING, SIZE, SUPERSAMPLING, 3).mean((1, 3))
    alpha = mask.reshape(SIZE, SUPERSAMPLING, SIZE, SUPERSAMPLING).mean((1, 3))[..., None]
    # The clear color is black. Divide by the coverage to remove the dark fringe at the edges.
    rgb = np.where(alpha > 0, rgb / np.maximum(alpha, 1e-6), 0)
    return np.dstack([np.clip(rgb, 0, 255), alpha * 255]).astype(np.uint8)


def main() -> None:
    for variant in VARIANTS:
        # Load the model with a key light, and the pose.
        spec = mujoco.MjSpec.from_file(str(get_mjcf_path(variant)))
        spec.worldbody.add_light(
            type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, dir=[-0.4, -0.6, -1.0], diffuse=[0.5, 0.5, 0.5],
            castshadow=False,
        )
        model = spec.compile()
        model.vis.headlight.ambient = [0.2, 0.2, 0.2]
        model.vis.headlight.diffuse = [0.3, 0.3, 0.3]
        model.vis.headlight.specular = [0.05, 0.05, 0.05]
        # Supersampling replaces multisampling, so the coverage matches the segmentation mask.
        model.vis.quality.offsamples = 0
        model.vis.global_.offwidth = model.vis.global_.offheight = SIZE * SUPERSAMPLING
        model.geom_rgba[model.geom_group == 3] = COLLIDER_RGBA
        data = mujoco.MjData(model)
        for joint, angle in POSE.items():
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint) >= 0:
                data.joint(joint).qpos = angle
        mujoco.mj_forward(model, data)

        # Frame the visual geoms with one camera, so both panels have the same scale.
        visual = model.geom_group == 1
        low = (data.geom_xpos[visual] - model.geom_rbound[visual, None]).min(0)
        high = (data.geom_xpos[visual] + model.geom_rbound[visual, None]).max(0)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = (low + high) / 2
        camera.distance = 1.5 * np.linalg.norm(high - low)
        camera.azimuth = AZIMUTH
        camera.elevation = ELEVATION
        panels = [render(model, data, camera, 1), render(model, data, camera, 3)]

        # Crop the panels to the union of their content, so they stay aligned.
        alpha = np.maximum(panels[0][..., 3], panels[1][..., 3])
        rows = np.flatnonzero(alpha.any(1))
        cols = np.flatnonzero(alpha.any(0))
        panels = [panel[rows[0] : rows[-1] + 1, cols[0] : cols[-1] + 1] for panel in panels]
        gap = np.zeros((panels[0].shape[0], panels[0].shape[0] // 6, 4), np.uint8)
        image = Image.fromarray(np.hstack([panels[0], gap, panels[1]]))
        image = image.resize((round(image.width * HEIGHT / image.height), HEIGHT), Image.LANCZOS)
        path = IMAGES_DIR / f"{variant}.png"
        image.save(path, optimize=True)
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
