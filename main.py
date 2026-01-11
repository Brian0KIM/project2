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

    robot.show("CENTER=Start", "L+R=Stop", "DFS backtracking")
    robot.say("Ready")
    robot.wait_for_center_press()
    robot.beep(900, 150)

    sw = StopWatch()
    node_debounce = EdgeDebounce(config.NODE_DEBOUNCE_MS)

    # The spec names this counter "blue_stack" even though the sticker is RED.
    blue_stack = 0

    has_block = False
    drop_on_node = False

    # DFS stack: each entry stores remaining options at an intersection.
    dfs_stack = []
    backtracking = False

    # last_dir affects lost-line recovery.
    # Possible: "L", "S", "R", "B"
    last_dir = "S"

    pickup_hits = 0

    while True:
        pressed = robot.brick.buttons.pressed()
        if (Button.LEFT in pressed) and (Button.RIGHT in pressed):
            break

        # Pickup detection (ultrasonic).
        if config.ENABLE_PICKUP and (not has_block):
            if robot.distance_mm() <= int(config.PICKUP_DISTANCE_MM):
                pickup_hits += 1
            else:
                pickup_hits = 0

            if pickup_hits >= int(config.PICKUP_CONFIRM_COUNT):
                robot.stop()
                robot.say("Pick up")
                gripper.close()
                has_block = True
                drop_on_node = bool(config.DROP_ON_NODE_RED)
                pickup_hits = 0
                follower.reset_flags()

        # Finish detection (GREEN).
        if robot.center_is_green():
            robot.stop()
            robot.say("Finish")
            break

        # Node detection (RED).
        if robot.center_is_red() and node_debounce.ready():
            node_debounce.trigger()
            blue_stack += 1

            robot.stop()
            robot.beep(1200, 150)
            robot.say("Red")
            robot.show(f"RED={blue_stack}", f"t={fmt_ms(sw.time())}", f"block={int(has_block)}")

            # Drop if carrying.
            if has_block and drop_on_node:
                gripper.open()
                robot.drive.straight(-50)
                robot.drive.stop()
                has_block = False
                drop_on_node = False

            # Node is typically a dead-end -> backtrack.
            backtracking = True
            if config.AUTO_UTURN_ON_NODE:
                navigator.do_turn(robot, int(config.TURN_UTURN_DEG))
                follower.reset_flags()

        # State calculation (3-bit, 0..7).
        state = int(follower.state_from_sensors(robot))
        is_intersection, is_lost = follower.update_flags_from_state(state)

        if is_lost:
            navigator.recover_from_lost(robot, follower, last_dir)
        elif is_intersection:
            last_dir, backtracking = navigator.handle_intersection_dfs(robot, follower, dfs_stack, backtracking)
        else:
            # Normal line following using left/right reflections only.
            ref_l = robot.left_color.reflection()
            ref_r = robot.right_color.reflection()
            turn_rate = follower.compute_turn_rate(ref_l, ref_r)
            robot.drive.drive(int(config.BASE_SPEED), float(turn_rate))

        wait(int(config.CONTROL_LOOP_MS))

    robot.stop()
    robot.show("Stopped", f"RED={blue_stack}", f"t={fmt_ms(sw.time())}")
    robot.beep(600, 200)


if __name__ == "__main__":
    main()


