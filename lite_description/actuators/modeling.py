"""Compute actuator parameters from a second-order model of the joint.

The equations come from BeyondMimic: From Motion Tracking to Versatile Humanoid Control
via Guided Diffusion (https://arxiv.org/abs/2508.08241). Each input is a field of an
actuator spec table in this package, such as ``ROBSTRIDE_06_ACTUATOR_PARAMS["armature"]``.
"""

from math import pi

# Default closed-loop natural frequency, 10 Hz in rad/s.
NATURAL_FREQUENCY = 10 * 2 * pi


def compute_stiffness(
    armature: float,
    natural_frequency: float = NATURAL_FREQUENCY,
) -> float:
    """Return the joint stiffness in Nm/rad.

    Args:
        armature: Armature in kg·m^2.
        natural_frequency: Closed-loop natural frequency in rad/s.
    """
    return armature * natural_frequency**2


def compute_damping(
    armature: float,
    natural_frequency: float = NATURAL_FREQUENCY,
    damping_ratio: float = 2.0,
) -> float:
    """Return the joint damping in Nm·s/rad.

    Args:
        armature: Armature in kg·m^2.
        natural_frequency: Closed-loop natural frequency in rad/s.
        damping_ratio: Damping ratio. Above 1.0 is overdamped.
    """
    return 2.0 * damping_ratio * armature * natural_frequency


def compute_action_scale(
    armature: float,
    effort_limit: float,
    natural_frequency: float = NATURAL_FREQUENCY,
    action_scale_coefficient: float = 0.25,
) -> float:
    """Return the policy action scale in rad.

    This is the position offset whose stiffness torque reaches
    ``action_scale_coefficient`` of the actuator's effort limit.

    Args:
        armature: Armature in kg·m^2.
        effort_limit: Effort limit in Nm.
        natural_frequency: Closed-loop natural frequency in rad/s.
        action_scale_coefficient: Fraction of the effort limit a unit action commands.
    """
    stiffness = compute_stiffness(armature, natural_frequency)
    return action_scale_coefficient * effort_limit / stiffness
