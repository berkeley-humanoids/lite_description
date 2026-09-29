"""Tests for the MJCF post-processing steps and the committed MJCF assets."""
import xml.etree.ElementTree as ET

import mujoco
import pytest

from lite_description import ROBOTS_DIR
from lite_description.workflow.urdf_to_mjcf import add_actuators, replace_cylinders_with_capsules

MJCF_FILES = sorted(ROBOTS_DIR.glob("*/mjcf/*.xml"))
MJCF_IDS = [p.parent.parent.name for p in MJCF_FILES]


def test_replace_cylinders_with_capsules():
    root = ET.fromstring(
        """<mujoco>
  <worldbody>
    <body name="base">
      <geom name="upper_arm" type="cylinder" size="0.03 0.04" pos="1 2 3"/>
      <geom name="foot" type="box" size="0.1 0.2 0.3"/>
      <geom name="forearm" type="cylinder" size="0.02 0.05" quat="1 0 0 0"/>
    </body>
  </worldbody>
</mujoco>""",
    )

    count = replace_cylinders_with_capsules(root)

    geoms = {geom.get("name"): geom for geom in root.iter("geom")}
    assert count == 2
    assert geoms["upper_arm"].get("type") == "capsule"
    # Retyping keeps every other attribute untouched.
    assert geoms["upper_arm"].get("size") == "0.03 0.04"
    assert geoms["upper_arm"].get("pos") == "1 2 3"
    assert geoms["foot"].get("type") == "box"
    assert geoms["forearm"].get("type") == "capsule"


def test_replace_cylinders_with_capsules_returns_zero_when_no_cylinders():
    root = ET.fromstring(
        """<mujoco>
  <worldbody>
    <body name="base">
      <geom name="foot" type="box" size="0.1 0.2 0.3"/>
    </body>
  </worldbody>
</mujoco>""",
    )

    assert replace_cylinders_with_capsules(root) == 0


def test_add_actuators_emits_position_actuators_and_no_sensors():
    root = ET.fromstring(
        """<mujoco>
  <worldbody>
    <body name="base">
      <joint name="j1" actuatorfrcrange="-2 2"/>
      <geom name="g" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>""",
    )

    add_actuators(root, {"j1": {"effort_limit": 5}})

    assert root.find("sensor") is None
    # <position>, not <motor>: the joint PD is evaluated every mj_step inside the
    # physics model rather than sampled once per controller_manager tick.
    actuators = root.findall("./actuator/position")
    assert root.findall("./actuator/motor") == []
    assert [a.get("name") for a in actuators] == ["j1"]
    # forcerange is symmetric around zero, from the joint's effort_limit.
    assert actuators[0].get("forcerange") == "-5 5"


def test_add_actuators_emits_no_ctrlrange():
    """An RL policy may command past the joint limit to hold a saturating torque."""
    root = ET.fromstring(
        """<mujoco>
  <worldbody>
    <body name="base"><joint name="j1" actuatorfrcrange="-2 2" range="-1 1"/></body>
  </worldbody>
</mujoco>""",
    )

    add_actuators(root, {"j1": {"effort_limit": 5}})

    actuator = root.find("./actuator/position")
    assert actuator.get("ctrlrange") is None
    assert actuator.get("ctrllimited") is None


@pytest.mark.skipif(not MJCF_FILES, reason="no generated MJCFs")
@pytest.mark.parametrize("mjcf_path", MJCF_FILES, ids=MJCF_IDS)
def test_committed_mjcf_loads_and_sensors_are_site_backed(mjcf_path):
    """Every committed MJCF must load, and every sensor it carries must sit on a <site>.

    This guards ros2_control. mujoco_ros2_control's plugin init loops over every sensor
    and builds ``std::string(mj_id2name(model, mjOBJ_SITE, sensor_objid))``
    (mujoco_ros2_control.cpp:124), which is null and aborts for a sensor whose object is
    not a <site>. Fixed-base variants ship sensor-free. The floating-base biped ships
    base-state sensors (framequat, gyro, accelerometer, velocimeter, framepos) all
    attached to the same IMU site, so the one asset loads under both mjlab, which reads
    qpos/qvel from Entity data, and mujoco_ros2_control.
    """
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        assert model.sensor_objtype[i] == mujoco.mjtObj.mjOBJ_SITE, (
            f"{mjcf_path} sensor {name!r} is not site-backed "
            f"(objtype={model.sensor_objtype[i]}); mujoco_ros2_control would abort"
        )
