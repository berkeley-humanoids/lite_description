"""Paths to the generated Lite robot descriptions.

Each path names a file inside the installed package. A URDF or MJCF reaches its meshes by a
path relative to its own directory, so open the file where it lies instead of copying it out.
"""

from pathlib import Path

ROBOTS_DIR = Path(__file__).parent / "robots"
VARIANTS = tuple(sorted(path.name for path in ROBOTS_DIR.iterdir() if path.is_dir()))

__all__ = ["ROBOTS_DIR", "VARIANTS", "get_foot_hull_path", "get_mjcf_path", "get_urdf_path"]


def get_urdf_path(variant: str) -> Path:
    """Return the flat URDF of ``variant``, for Isaac Lab and other URDF consumers."""
    return _variant_dir(variant) / "urdf" / f"{variant}.urdf"


def get_mjcf_path(variant: str) -> Path:
    """Return the MJCF of ``variant``, the MuJoCo training and deployment model."""
    return _variant_dir(variant) / "mjcf" / f"{variant}.xml"


def get_foot_hull_path(variant: str, side: str) -> Path:
    """Return the optional sole hull of the ``side`` foot, an STL in the foot frame.

    The MJCF does not reference the hull. Only ``lite_biped`` ships one.
    """
    path = _variant_dir(variant) / "meshes" / "collision" / f"{side}_foot_hull.stl"
    if not path.is_file():
        raise FileNotFoundError(f"{variant} ships no {side} foot hull at {path}.")
    return path


def _variant_dir(variant: str) -> Path:
    if variant not in VARIANTS:
        raise ValueError(f"Unknown variant {variant!r}. Expected one of {VARIANTS}.")
    return ROBOTS_DIR / variant
