"""Tests for the path API that consumers import from ``lite_description``, and the package metadata."""
from importlib.metadata import version
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from lite_description import VARIANTS, get_mjcf_path, get_urdf_path


@pytest.mark.parametrize("variant", VARIANTS)
def test_paths_name_committed_files(variant):
    assert get_urdf_path(variant).is_file()
    assert get_mjcf_path(variant).is_file()


def test_unknown_variant_raises():
    with pytest.raises(ValueError, match="Unknown variant"):
        get_mjcf_path("lite_unknown")


def test_package_xml_version_matches_pyproject():
    root = ET.parse(Path(__file__).parents[1] / "package.xml").getroot()
    assert root.findtext("version") == version("lite-description")
