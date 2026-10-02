from math import pi

# velocity_limit is the no-load output speed of the RobStride datasheet, which gives it in
# rpm. Each value converts it in place, as rpm * 2 * pi / 60. effort_limit is the peak
# load of the same datasheet.
RPM_TO_RAD_PER_S = 2 * pi / 60

# https://robstride.com/products/robStride00
ROBSTRIDE_00_ACTUATOR_PARAMS = {
    "velocity_limit"    : 315 * RPM_TO_RAD_PER_S,   # rad/s, 315 rpm
    "effort_limit"      : 14.0,     # Nm
    "armature"          : 0.002,    # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 00 actuator. """


# https://robstride.com/products/robStride01
ROBSTRIDE_01_ACTUATOR_PARAMS = {
    "velocity_limit"    : 315 * RPM_TO_RAD_PER_S,   # rad/s, 315 rpm
    "effort_limit"      : 17.0,     # Nm
    "armature"          : 0.005,    # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 01 actuator. """


# https://robstride.com/products/robStride02
ROBSTRIDE_02_ACTUATOR_PARAMS = {
    "velocity_limit"    : 410 * RPM_TO_RAD_PER_S,   # rad/s, 410 rpm
    "effort_limit"      : 17.0,     # Nm
    "armature"          : 0.005,    # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 02 actuator. """


# https://robstride.com/products/robStride03
ROBSTRIDE_03_ACTUATOR_PARAMS = {
    "velocity_limit"    : 200 * RPM_TO_RAD_PER_S,   # rad/s, 200 rpm
    "effort_limit"      : 60.0,     # Nm
    "armature"          : 0.01,     # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 03 actuator. """


# https://robstride.com/products/robStride04
ROBSTRIDE_04_ACTUATOR_PARAMS = {
    "velocity_limit"    : 200 * RPM_TO_RAD_PER_S,   # rad/s, 200 rpm
    "effort_limit"      : 120.0,    # Nm
    "armature"          : 0.016,    # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 04 actuator. """


# https://robstride.com/products/robStride05
ROBSTRIDE_05_ACTUATOR_PARAMS = {
    "velocity_limit"    : 480 * RPM_TO_RAD_PER_S,   # rad/s, 480 rpm
    "effort_limit"      : 5.5,      # Nm
    "armature"          : 0.001,    # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 05 actuator. """


# https://robstride.com/products/robStride06
ROBSTRIDE_06_ACTUATOR_PARAMS = {
    "velocity_limit"    : 480 * RPM_TO_RAD_PER_S,   # rad/s, 480 rpm
    "effort_limit"      : 36.0,     # Nm
    "armature"          : 0.008,    # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 06 actuator. """
