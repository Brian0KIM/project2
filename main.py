"""
Entry point for the EV3 line maze project.

All comments are intentionally in English (per user rule).
"""

from pybricks.parameters import Button
from pybricks.tools import StopWatch, wait

import config
import navigator
from gripper import Gripper
from line_follow import LineFollower
from robot import Robot
from utils import EdgeDebounce, fmt_ms


def main() -> None:
    robot = Robot()
    follower = LineFollower()
    gripper = Gripper(robot.gripper_motor)

    # Initial gripper position.
    gripper.open()

    robot.show("CENTER=Start", "L+R=Stop", "Right-hand rule")
    robot.say("Ready")
    robot.wait_for_center_press()
    robot.beep(900, 150)

    sw = StopWatch()
    blue_debounce = EdgeDebounce(config.BLUE_DEBOUNCE_MS)

    blue_count = 0
    carrying = False
    drop_on_blue = False

    pickup_hits = 0

    while True:
        pressed = robot.brick.buttons.pressed()
        if (Button.LEFT in pressed) and (Button.RIGHT in pressed):
            break

        # Read sensors once per loop.
        ref_l, ref_c, ref_r = robot.reflections()

        # Pickup detection (ultrasonic).
        if config.ENABLE_PICKUP and (not carrying):
            if robot.distance_mm() <= int(config.PICKUP_DISTANCE_MM):
                pickup_hits += 1
            else:
                pickup_hits = 0

            if pickup_hits >= int(config.PICKUP_CONFIRM_COUNT):
                robot.stop()
                robot.say("Pick up")
                gripper.close()
                carrying = True
                drop_on_blue = bool(config.DROP_ON_FIRST_BLUE)
                pickup_hits = 0
                follower.reset_flags()

        # Blue node detection.
        if robot.any_blue() and blue_debounce.ready():
            blue_debounce.trigger()
            blue_count += 1
            elapsed = sw.time()

            robot.beep(1100, 120)
            robot.show(f"BLUE={blue_count}", f"t={fmt_ms(elapsed)}", f"carry={int(carrying)}")

            # Optional drop: first blue after pickup.
            if carrying and drop_on_blue:
                robot.stop()
                robot.say("Drop")
                gripper.open()
                carrying = False
                drop_on_blue = False
                follower.reset_flags()

            # Many blue stickers are at dead-ends; U-turn helps exploration.
            if config.AUTO_UTURN_ON_BLUE:
                navigator.do_turn(robot, int(config.TURN_UTURN_DEG))
                follower.reset_flags()

            if int(config.TARGET_BLUE_COUNT) > 0 and blue_count >= int(config.TARGET_BLUE_COUNT):
                break

        # Normal line following.
        turn_rate = follower.compute_turn_rate(ref_l, ref_c, ref_r)
        robot.drive.drive(int(config.BASE_SPEED), float(turn_rate))

        # Situation detection.
        is_intersection, is_lost = follower.update_flags(ref_l, ref_c, ref_r)

        if is_lost:
            robot.drive.stop()
            navigator.recover_line(robot, follower)
        elif is_intersection:
            robot.drive.stop()
            navigator.handle_intersection(robot, follower)

        wait(int(config.CONTROL_LOOP_MS))

    robot.stop()
    robot.show("Stopped", f"BLUE={blue_count}", f"t={fmt_ms(sw.time())}")
    robot.beep(600, 200)


if __name__ == "__main__":
    main()


