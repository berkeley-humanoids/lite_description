
# armature: the output-side equivalent inertia (低速端等效惯量, Moment Inertia) of the
# RobStride RS series specification, 2026-10-09, in
# https://github.com/RobStride/Product_Information.

# https://robstride.com/products/robStride00
ROBSTRIDE_00_ACTUATOR_PARAMS = {
    "velocity_limit"    : 32.99,    # rad/s, 315 rpm no-load
    "effort_limit"      : 14.0,     # Nm
    "armature"          : 0.001,    # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 00 actuator. """


# https://robstride.com/products/robStride01
ROBSTRIDE_01_ACTUATOR_PARAMS = {
    "velocity_limit"    : 32.99,    # rad/s, 315 rpm no-load
    "effort_limit"      : 17.0,     # Nm
    "armature"          : 0.0042,   # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 01 actuator. """


# https://robstride.com/products/robStride02
ROBSTRIDE_02_ACTUATOR_PARAMS = {
    "velocity_limit"    : 42.94,    # rad/s, 410 rpm no-load
    "effort_limit"      : 17.0,     # Nm
    "armature"          : 0.0042,   # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 02 actuator. """


# https://robstride.com/products/robStride03
ROBSTRIDE_03_ACTUATOR_PARAMS = {
    "velocity_limit"    : 20.94,    # rad/s, 200 rpm no-load
    "effort_limit"      : 60.0,     # Nm
    "armature"          : 0.02,     # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 03 actuator. """


# https://robstride.com/products/robStride04
ROBSTRIDE_04_ACTUATOR_PARAMS = {
    "velocity_limit"    : 20.94,    # rad/s, 200 rpm no-load
    "effort_limit"      : 120.0,    # Nm
    "armature"          : 0.04,     # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 04 actuator. """


# https://robstride.com/products/robStride05
ROBSTRIDE_05_ACTUATOR_PARAMS = {
    "velocity_limit"    : 50.27,    # rad/s, 480 rpm no-load
    "effort_limit"      : 5.5,      # Nm
    "armature"          : 0.0007,   # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 05 actuator. """


# https://robstride.com/products/robStride06
ROBSTRIDE_06_ACTUATOR_PARAMS = {
    "velocity_limit"    : 50.27,    # rad/s, 480 rpm no-load
    "effort_limit"      : 36.0,     # Nm
    "armature"          : 0.012,    # kgm^2
    "friction_loss"     : 0.1,      # Nm
}
""" Parameters for the Robstride 06 actuator. """
