"""
Project configuration for EV3 MicroPython (Pybricks).

All comments are intentionally in English (per user rule).
"""

from pybricks.parameters import Port
from pybricks.parameters import Color

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
# State machine timing (tune)
# ------------------------------

# How long state==7(111) must persist before being accepted as an intersection candidate.
INTERSECTION_CONFIRM_MS = 70

# How long state==0(000) must persist before triggering the lost-line recovery.
LOST_CONFIRM_MS = 130

# ------------------------------
# Intersection classification / maneuvers (tune)
# ------------------------------

# Extra movement into the junction before deciding available directions.
INTERSECTION_ADVANCE_MM = 45

# After selecting a direction at an intersection, move forward to exit it.
EXIT_INTERSECTION_MM = 25

# Turning angles (tune if your geometry differs)
TURN_RIGHT_DEG = 90
TURN_LEFT_DEG = -90
TURN_UTURN_DEG = 180

# A very short right attempt used to distinguish ㅏ/ㅓ type intersections.
SHORT_RIGHT_TRY_DEG = 25
SHORT_TRY_FORWARD_MM = 35
SHORT_TRY_MS = 250

# ------------------------------
# Center sensor (Color mode) classification
# ------------------------------

# Only these colors are trusted; everything else is treated as WHITE to reduce false positives.
CENTER_TRUSTED_COLORS = (Color.BLACK, Color.RED, Color.GREEN)

# ------------------------------
# Node / finish behavior
# ------------------------------

# Debounce for RED node detection to avoid multiple counts.
NODE_DEBOUNCE_MS = 900

# If True, do a U-turn after processing a RED node (dead-end node behavior).
AUTO_UTURN_ON_NODE = True

# ------------------------------
# Ultrasonic pickup / drop behavior
# ------------------------------

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
DROP_ON_NODE_RED = True

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


