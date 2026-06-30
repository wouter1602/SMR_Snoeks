#!/usr/bin/env python3
"""
Simple async Doosan robot move script.
Connects to the robot, gets control, servos on, and moves to a single joint target.
"""

import asyncio
import time
import numpy as np
import doosan_drfl as drfl
import sys

# -------- Configuration --------
IP_ADDRESS = "192.168.0.50"
PORT = 12345

SPEED = 5.0          # deg/s
ACCELERATION = 5.0   # deg/s^2
MOVE_TIMEOUT = 40.0  # seconds
POLL_INTERVAL = 0.05 # seconds

# Target joint position (J1..J6) in degrees
TARGET_POS = np.array([0.0, 0.0, 30.0, 0.0, 0.0, 0.0], dtype=np.float32)

VIRTUAL = False  # True = simulated, False = real robot

# -------- Globals --------
robot = drfl.CDRFLEx()
get_control_access = False
is_standby = False
is_moving = False
move_was_active = False   # set True (by callback) the moment we ever see Moving


# -------- Callbacks --------
def on_access(access):
    global get_control_access
    print(f"[ACCESS] {drfl.MONITORING_ACCESS_CONTROL(access).name}")
    get_control_access = (access == drfl.MONITORING_ACCESS_CONTROL.Grant)


def on_state(state):
    global is_standby, is_moving, move_was_active
    print(f"[STATE] {drfl.ROBOT_STATE(state).name}")
    is_standby = (state == drfl.ROBOT_STATE.Standby)
    is_moving = (state == drfl.ROBOT_STATE.Moving)
    if is_moving:
        move_was_active = True


async def wait_for_move_complete(timeout: float) -> bool:
    """
    Wait until we have observed a Moving state AND the robot is back in Standby.
    This is robust against the callback thread firing Moving->Standby before
    we get a chance to poll.
    """
    start = time.time()
    while not (move_was_active and is_standby):
        await asyncio.sleep(POLL_INTERVAL)
        if time.time() - start > timeout:
            return False
    return True


async def async_input(prompt: str = "") -> str:
    """Non-blocking input using a thread executor."""
    return await asyncio.get_event_loop().run_in_executor(None, input, prompt)


async def main():
    global get_control_access, is_standby, move_was_active

    # 1. Register callbacks
    robot.set_on_monitoring_access_control(on_access)
    robot.set_on_monitoring_state(on_state)

    # 2. Connect
    print(f"Connecting to {IP_ADDRESS}:{PORT} ...")
    if not robot.open_connection(IP_ADDRESS, port=PORT):
        print("Connection failed.")
        sys.exit(1)

    robot.setup_monitoring_version(1)

    # 3. Acquire control + servo on
    for _ in range(10):
        if not get_control_access:
            robot.manage_access_control(drfl.MANAGE_ACCESS_CONTROL.Force_request)
            await asyncio.sleep(0.5)
            continue
        if not is_standby:
            robot.set_robot_control(drfl.ROBOT_CONTROL.Servo_on)
            await asyncio.sleep(1.0)
            continue
        break

    if not (get_control_access and is_standby):
        print("Could not get control or servo on.")
        robot.close_connection()
        sys.exit(1)

    # 4. Configure mode
    system = drfl.ROBOT_SYSTEM.Virtual if VIRTUAL else drfl.ROBOT_SYSTEM.Real
    robot.set_robot_system(system)
    robot.set_robot_mode(drfl.ROBOT_MODE.Autonomous)

    await async_input("Press Enter to move the robot...")

    # 5. Move to target (async movej)
    # Reset the "we saw Moving" latch right before issuing the command.
    move_was_active = False

    print(f"Moving to {TARGET_POS} ...")
    ok = robot.amovej(
        pos=TARGET_POS,
        vel=SPEED,
        acc=ACCELERATION,
        time=0.0,
        move_mode=drfl.MOVE_MODE.Absolute,
        blending_type=drfl.BLENDING_SPEED_TYPE.Duplicate,
    )

    if not ok:
        print("amovej command failed.")
        robot.close_connection()
        sys.exit(1)

    # Wait until we have actually seen a Moving state and returned to Standby.
    if not await wait_for_move_complete(MOVE_TIMEOUT):
        print("Move timed out (never observed Moving->Standby transition).")
        robot.close_connection()
        sys.exit(1)

    print("Target reached!")

    # 6. Cleanup
    robot.close_connection()
    print("Connection closed.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrupted. Closing connection...")
        robot.close_connection()
        sys.exit(1)
