"""
Robot hardware wrapper (motors + sensors + brick).

All comments are intentionally in English (per user rule).
"""

from pybricks.ev3devices import ColorSensor, Motor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Direction, Stop, Button, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

import config


class Robot:
    def __init__(self):
        self.brick = EV3Brick()

        # Motors
        self.left_motor = Motor(config.LEFT_DRIVE_MOTOR_PORT, positive_direction=Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(config.RIGHT_DRIVE_MOTOR_PORT, positive_direction=Direction.CLOCKWISE)
        self.gripper_motor = Motor(config.GRIPPER_MOTOR_PORT)

        # Sensors
        self.left_color = ColorSensor(config.LEFT_COLOR_SENSOR_PORT)
        self.center_color = ColorSensor(config.CENTER_COLOR_SENSOR_PORT)
        self.right_color = ColorSensor(config.RIGHT_COLOR_SENSOR_PORT)
        self.ultra = UltrasonicSensor(config.ULTRASONIC_SENSOR_PORT)

        # Drive base
        self.drive = DriveBase(
            self.left_motor,
            self.right_motor,
            wheel_diameter=config.WHEEL_DIAMETER_MM,
            axle_track=config.AXLE_TRACK_MM,
        )

        # Sensible defaults
        self.drive.settings(straight_speed=200, straight_acceleration=400, turn_rate=250, turn_acceleration=400)

    # ------------------------------
    # Read sensors
    # ------------------------------

    def reflections(self):
        # Returns reflections (0..100) from left/center/right sensors.
        return (
            self.left_color.reflection(),
            self.center_color.reflection(),
            self.right_color.reflection(),
        )

    def colors(self):
        # Returns Color enums from left/center/right sensors.
        return (
            self.left_color.color(),
            self.center_color.color(),
            self.right_color.color(),
        )

    def any_blue(self) -> bool:
        l, c, r = self.colors()
        return (l == Color.BLUE) or (c == Color.BLUE) or (r == Color.BLUE)

    def distance_mm(self) -> int:
        # UltrasonicSensor.distance() returns mm in Pybricks.
        d = self.ultra.distance()
        if d is None:
            return 10**9
        return int(d)

    # ------------------------------
    # UI helpers
    # ------------------------------

    def wait_for_center_press(self):
        # Wait for user to press the center button.
        while Button.CENTER not in self.brick.buttons.pressed():
            wait(10)
        # Wait for release to avoid double-trigger.
        while Button.CENTER in self.brick.buttons.pressed():
            wait(10)

    def beep(self, freq: int = 800, ms: int = 120):
        self.brick.speaker.beep(freq, ms)

    def say(self, text: str):
        if config.ENABLE_SPEECH:
            try:
                self.brick.speaker.say(text)
                return
            except Exception:
                # Speech may be unavailable depending on firmware; fall back silently.
                pass

    def show(self, line1: str = "", line2: str = "", line3: str = ""):
        self.brick.screen.clear()
        self.brick.screen.draw_text(0, 0, str(line1))
        if line2:
            self.brick.screen.draw_text(0, 18, str(line2))
        if line3:
            self.brick.screen.draw_text(0, 36, str(line3))

    # ------------------------------
    # Motion helpers
    # ------------------------------

    def stop(self):
        self.drive.stop()
        self.left_motor.stop(Stop.BRAKE)
        self.right_motor.stop(Stop.BRAKE)


