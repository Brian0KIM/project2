"""
Intersection handling and recovery behaviors.

All comments are intentionally in English (per user rule).
"""

from pybricks.tools import wait

import config


def _any_on_line(ref_l: int, ref_c: int, ref_r: int) -> bool:
    thr = int(config.LINE_THRESHOLD)
    return (int(ref_l) <= thr) or (int(ref_c) <= thr) or (int(ref_r) <= thr)


def do_turn(robot, angle_deg: int) -> None:
    # Execute an in-place turn using the DriveBase.
    if int(angle_deg) == 0:
        return
    robot.drive.turn(int(angle_deg))


def _probe_branch(robot, turn_deg: int) -> bool:
    """
    Probe whether a branch exists in a given direction.

    Method:
    - Turn partially toward the candidate direction
    - Drive forward a small distance
    - Check if the center sensor still sees the line
    - Return to the original pose
    """
    if int(turn_deg) != 0:
        robot.drive.turn(int(turn_deg))
        wait(30)

    robot.drive.straight(int(config.PROBE_FORWARD_MM))
    wait(20)
    ref_c = robot.center_color.reflection()
    on_line = int(ref_c) <= int(config.LINE_THRESHOLD)

    robot.drive.straight(-int(config.PROBE_FORWARD_MM))
    wait(20)

    if int(turn_deg) != 0:
        robot.drive.turn(-int(turn_deg))
        wait(30)

    return on_line


def handle_intersection(robot, follower) -> None:
    """
    Apply right-hand rule:
    Right > Straight > Left > U-turn
    """
    # Move into the intersection so probes are meaningful.
    robot.drive.straight(int(config.INTERSECTION_ADVANCE_MM))
    robot.drive.stop()
    follower.reset_flags()
    wait(30)

    probe_r = int(config.TURN_RIGHT_DEG * float(config.PROBE_TURN_FRACTION))
    probe_l = int(config.TURN_LEFT_DEG * float(config.PROBE_TURN_FRACTION))

    right_ok = _probe_branch(robot, probe_r)
    straight_ok = _probe_branch(robot, 0)
    left_ok = _probe_branch(robot, probe_l)

    if right_ok:
        do_turn(robot, int(config.TURN_RIGHT_DEG))
    elif straight_ok:
        # Keep heading
        pass
    elif left_ok:
        do_turn(robot, int(config.TURN_LEFT_DEG))
    else:
        do_turn(robot, int(config.TURN_UTURN_DEG))

    # Exit the intersection before resuming normal line following.
    robot.drive.straight(int(config.EXIT_INTERSECTION_MM))
    robot.drive.stop()
    follower.reset_flags()


def recover_line(robot, follower) -> None:
    """
    Simple lost-line recovery:
    - Spin right for a while to search
    - If not found, spin left
    - If still not found, do a U-turn as a last resort
    """
    robot.drive.stop()
    follower.reset_flags()

    def spin_search(turn_rate: int, duration_ms: int) -> bool:
        elapsed = 0
        while elapsed < duration_ms:
            ref_l, ref_c, ref_r = robot.reflections()
            if _any_on_line(ref_l, ref_c, ref_r):
                robot.drive.stop()
                follower.reset_flags()
                return True
            robot.drive.drive(0, int(turn_rate))
            wait(int(config.CONTROL_LOOP_MS))
            elapsed += int(config.CONTROL_LOOP_MS)
        robot.drive.stop()
        return False

    # Try right then left
    if spin_search(turn_rate=160, duration_ms=800):
        return
    if spin_search(turn_rate=-160, duration_ms=1200):
        return

    # Last resort
    do_turn(robot, int(config.TURN_UTURN_DEG))
    follower.reset_flags()


