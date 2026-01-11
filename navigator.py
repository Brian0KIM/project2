"""
Intersection handling and recovery behaviors.

All comments are intentionally in English (per user rule).
"""

from pybricks.tools import wait

import config


def do_turn(robot, angle_deg: int) -> None:
    # Execute an in-place turn using the DriveBase.
    if int(angle_deg) == 0:
        return
    robot.drive.turn(int(angle_deg))


def _follow_for_ms(robot, follower, ms: int, speed: int) -> float:
    """
    Follow the line for a short time window and return the average turn_rate.
    Positive means it tended to turn right.
    """
    elapsed = 0
    acc = 0.0
    n = 0
    while elapsed < int(ms):
        ref_l = robot.left_color.reflection()
        ref_r = robot.right_color.reflection()
        tr = float(follower.compute_turn_rate(ref_l, ref_r))
        robot.drive.drive(int(speed), tr)
        acc += tr
        n += 1
        wait(int(config.CONTROL_LOOP_MS))
        elapsed += int(config.CONTROL_LOOP_MS)
    robot.drive.stop()
    if n <= 0:
        return 0.0
    return acc / float(n)


def _short_right_try(robot, follower) -> bool:
    """
    Distinguish ㅏ/ㅓ type intersections by attempting a very short right turn.

    If the robot keeps converging to the right branch after the attempt, return True.
    Otherwise revert and treat it as straight.
    """
    do_turn(robot, int(config.SHORT_RIGHT_TRY_DEG))
    avg_tr = _follow_for_ms(robot, follower, int(config.SHORT_TRY_MS), speed=int(config.BASE_SPEED * 0.6))

    # Heuristic: if we are still steering right significantly, we likely found a right branch.
    if avg_tr > 30.0:
        # Commit to full right turn (already turned partially).
        remaining = int(config.TURN_RIGHT_DEG) - int(config.SHORT_RIGHT_TRY_DEG)
        do_turn(robot, remaining)
        return True

    # Revert to original heading if it did not commit.
    do_turn(robot, -int(config.SHORT_RIGHT_TRY_DEG))
    return False


def classify_intersection(robot, follower):
    """
    Classify intersection based on the described method:
    - state==7(111) sustained => candidate (+ or T)
    - advance slightly, then check:
      - state==5(101) => T
      - state==7(111) => +
    Returns (kind, dir_array)
    dir_array = [L, S, R] with 0/1.
    """
    robot.drive.straight(int(config.INTERSECTION_ADVANCE_MM))
    robot.drive.stop()
    follower.reset_flags()
    wait(30)

    state = int(follower.state_from_sensors(robot))
    if state == 7:
        return "PLUS", [1, 1, 1]
    if state == 5:
        return "T", [1, 0, 1]
    # Fallback: treat unknown as T-like.
    return "T", [1, 0, 1]


def handle_intersection_dfs(robot, follower, dfs_stack, backtracking: bool):
    """
    DFS(backtracking) intersection handling.

    - When exploring: push remaining options to stack.
    - When backtracking: pop until an intersection with remaining options is found.
      If none, keep going straight (continue backtracking).
    Returns (last_dir, backtracking).
    """
    kind, dir_array = classify_intersection(robot, follower)

    # Build options in priority order: Right -> Straight -> Left
    options = []
    if int(dir_array[2]) == 1:
        options.append("R")
    if int(dir_array[1]) == 1:
        options.append("S")
    if int(dir_array[0]) == 1:
        options.append("L")

    chosen = "S"
    if backtracking:
        # Skip finished intersections.
        while dfs_stack and (not dfs_stack[-1]["options"]):
            dfs_stack.pop()

        if dfs_stack and dfs_stack[-1]["options"]:
            chosen = dfs_stack[-1]["options"].pop(0)
            backtracking = False
        else:
            # Continue backtracking: go straight through this intersection.
            chosen = "S"
            backtracking = True
    else:
        if not options:
            chosen = "B"
            backtracking = True
        else:
            chosen = options.pop(0)
            dfs_stack.append({"options": options})

    last_dir = chosen

    # Execute decision.
    if chosen == "R":
        # For T-like intersections, a short right attempt can distinguish ㅏ/ㅓ behavior.
        if kind == "T":
            committed = _short_right_try(robot, follower)
            if not committed:
                # Treat as straight if it did not commit.
                last_dir = "S"
        else:
            do_turn(robot, int(config.TURN_RIGHT_DEG))
    elif chosen == "L":
        do_turn(robot, int(config.TURN_LEFT_DEG))
    elif chosen == "B":
        do_turn(robot, int(config.TURN_UTURN_DEG))
    else:
        # Straight: do nothing.
        pass

    robot.drive.straight(int(config.EXIT_INTERSECTION_MM))
    robot.drive.stop()
    follower.reset_flags()

    return last_dir, backtracking


def recover_from_lost(robot, follower, last_dir: str) -> None:
    """
    Lost-line recovery based on the last direction (last_dir).

    - If last_dir was left, try turning left.
    - If last_dir was right, try turning right.
    - Otherwise, try backing up.
    Repeat until the center sensor is BLACK again.
    """
    robot.drive.stop()
    follower.reset_flags()

    attempts = 0
    while not robot.center_is_black():
        attempts += 1
        if last_dir == "L":
            robot.drive.drive(0, -160)
            wait(140)
            robot.drive.stop()
        elif last_dir == "R":
            robot.drive.drive(0, 160)
            wait(140)
            robot.drive.stop()
        else:
            robot.drive.straight(-30)
            robot.drive.stop()

        wait(30)

        # Safety fallback to avoid infinite looping.
        if attempts >= 12:
            do_turn(robot, int(config.TURN_UTURN_DEG))
            attempts = 0
            follower.reset_flags()


