"""
Project configuration for EV3 MicroPython (Pybricks).

All comments are intentionally in English (per user rule).
"""

from pybricks.parameters import Port

# ------------------------------
# Ports (as provided by the user)
# ------------------------------

# Drive motors
LEFT_DRIVE_MOTOR_PORT = Port.B
RIGHT_DRIVE_MOTOR_PORT = Port.C

# Gripper motor
GRIPPER_MOTOR_PORT = Port.A

# Sensors
LEFT_COLOR_SENSOR_PORT = Port.S1
CENTER_COLOR_SENSOR_PORT = Port.S2
RIGHT_COLOR_SENSOR_PORT = Port.S3
ULTRASONIC_SENSOR_PORT = Port.S4

# ------------------------------
# Drivebase geometry (tune if needed)
# ------------------------------

# Standard EV3 wheel is often 56mm. Adjust if your wheels differ.
WHEEL_DIAMETER_MM = 56

# Distance between wheel centers. This is robot-dependent; tune for accurate turns.
AXLE_TRACK_MM = 114

# ------------------------------
# Line / surface calibration (tune REQUIRED)
# ------------------------------

# Reflection for black is typically low, white is high.
REFLECTION_BLACK_MAX = 15
REFLECTION_WHITE_MIN = 85

# Threshold to decide "on line" (black) vs "off line" (white).
LINE_THRESHOLD = (REFLECTION_BLACK_MAX + REFLECTION_WHITE_MIN) // 2

# ------------------------------
# Line following control (tune)
# ------------------------------

# mm/s (DriveBase speed)
BASE_SPEED = 140

# Proportional gain for steering. Increase if it reacts too slowly; decrease if it oscillates.
KP_TURN = 220

# Minimum blackness sum to consider that we see the line (prevents division by tiny numbers).
MIN_LINE_STRENGTH = 0.12

# Loop timing
CONTROL_LOOP_MS = 10

# ------------------------------
# Junction / dead-end detection (tune)
# ------------------------------

# How long the junction condition must persist before being accepted.
INTERSECTION_CONFIRM_MS = 70

# How long "line lost" must persist before triggering recovery.
LOST_CONFIRM_MS = 130

# Extra movement into the junction before deciding available directions.
INTERSECTION_ADVANCE_MM = 45

# Probe distance used to test whether a branch exists.
PROBE_FORWARD_MM = 35

# Use a fraction of the full turn for probing (e.g., 0.4 * 90deg ≈ 36deg).
PROBE_TURN_FRACTION = 0.4

# After selecting a direction at an intersection, move forward to exit it.
EXIT_INTERSECTION_MM = 25

# Turning angles (tune if your geometry differs)
TURN_RIGHT_DEG = 90
TURN_LEFT_DEG = -90
TURN_UTURN_DEG = 180

# ------------------------------
# Blue node behavior
# ------------------------------

# Avoid counting the same blue sticker multiple times.
BLUE_DEBOUNCE_MS = 900

# If True, the robot will U-turn after detecting a blue node (common for dead-ends).
AUTO_UTURN_ON_BLUE = True

# Stop after reaching this many blue nodes (set to 0 to never auto-stop).
TARGET_BLUE_COUNT = 0  # e.g. 7

# ------------------------------
# Ultrasonic pickup / drop behavior
# ------------------------------

# If True, the robot will pick up an object when detected by ultrasonic sensor.
ENABLE_PICKUP = True

# Distance in mm to consider an object "close enough" to pick up.
PICKUP_DISTANCE_MM = 85

# Require consecutive confirmations to reduce false triggers.
PICKUP_CONFIRM_COUNT = 3

# Drop behavior: drop the carried object on the first blue node after pickup.
DROP_ON_FIRST_BLUE = True

# ------------------------------
# Gripper configuration (tune)
# ------------------------------

# Angles depend on your gripper build; tune by trial.
GRIPPER_OPEN_ANGLE = -70
GRIPPER_CLOSE_ANGLE = 40
GRIPPER_SPEED_DPS = 500

# ------------------------------
# UX / sound
# ------------------------------

ENABLE_SPEECH = True


