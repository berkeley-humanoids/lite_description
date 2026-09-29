"""Tests for the path API that consumers import from ``lite_description``."""
import re
from pathlib import Path

import pytest

from lite_description import VARIANTS, get_mjcf_path, get_urdf_path

CMAKE_LISTS = Path(__file__).resolve().parents[1] / "CMakeLists.txt"


@pytest.mark.parametrize("variant", VARIANTS)
def test_paths_name_committed_files(variant):
    assert get_urdf_path(variant).is_file()
    assert get_mjcf_path(variant).is_file()


def test_unknown_variant_raises():
    with pytest.raises(ValueError, match="Unknown variant"):
        get_mjcf_path("lite_unknown")


def test_ament_installs_every_variant():
    # The wheel ships every variant directory and the ament package installs the CMake list,
    # so the two must name the same variants.
    body = re.search(r"set\(ROBOTS(?P<body>.*?)\)", CMAKE_LISTS.read_text(), re.DOTALL)["body"]
    assert tuple(sorted(body.split())) == VARIANTS
