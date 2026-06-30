#!/usr/bin/env python3
"""
Main Python file to control the Doosan robot for the SMR Airbus Minor project.
Loads poses from "poses_V2.json" and executes movement sequences.

This is a movement-only version: camera and database integration have been removed.
Supported step types:
    - sequence
    - pose
    - wait
    - air
    - wait_for_user_input
    - place_down
    - calibrate
"""
from typing import Tuple
import math
import multiprocessing as mp
import logging
import asyncio
import signal
import numpy as np
import sys
from pathlib import Path
import json
import copy
from scipy.spatial.transform import Rotation as R

import doosan_drfl as drfl
from robot_worker import robot_worker, MOVE_TIMEOUT, POLL_INTERVAL

# Setup variables
IP_DOOSAN = "192.168.0.50"
PORT_DOOSAN = 12345

FORCE_Z_AXIS_DOWN = 50.0  # in mm

DOOSAN_SPEED = 30
DOOSAN_ACCELERATION = 30
DOOSAN_LIN_SPEED = 70
DOOSAN_LIN_ACCELERATION = 70
DOOSAN_LIN_SPEED_SLOW = 30
DOOSAN_LIN_ACCELERATION_SLOW = 30

MAX_FORCE_BOX = 7.0
MAX_FORCE_PLACE_DOWN = 15.0


class RetrySequenceError(Exception):
    """ """


logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - [%(levelname)s] - %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

_currently_moving: bool = False
_current_force_pose: np.ndarray | None = None
_pickup_up: np.ndarray | None = None


async def wait_for_motion_complete(
    result_queue: mp.Queue,
    timeout: float = MOVE_TIMEOUT,
) -> None:
    """
    Wait for the robot motion to complete.

    Params:
        result_queue (mp.Queue): Queue that receives the result of the motion.
        timeout (float): Timeout for waiting for motion to complete.
    Raises:
        TimeoutError: If motion does not complete within the timeout.
    """
    global _currently_moving, _current_force_pose
    loop = asyncio.get_running_loop()
    _currently_moving = True

    try:
        result = await asyncio.wait_for(
            loop.run_in_executor(None, result_queue.get),
            timeout=timeout,
        )
    finally:
        _currently_moving = False

    if len(result) == 2:
        logger.debug(f"Stopped based on: {result[0]}. With result: {result[1]}")
        _current_force_pose = result[1].copy()
    else:
        if result != "done":
            raise RuntimeError(result)


async def execute_poses(
    pose: dict,
    command_queue: mp.Queue,
    result_queue: mp.Queue
) -> None:
    """
    Execute a single pose on the robot.

    Params:
        pose (dict): Dictionary containing the pose to execute.
        command_queue (mp.Queue): Queue to send the pose command to.
        result_queue (mp.Queue): Queue to receive the result of the motion.
    """
    move_type = pose["move_type"]
    if move_type not in ("joint", "linear", "force"):
        logger.warning(f"Wrong move type: {move_type}")
        return

    logger.debug(f"Executing pose: {pose['name']} [{move_type}]")
    pose_array = np.array(pose['pose_array'], dtype=np.float32)

    if "max_force" in pose:
        command_queue.put((move_type, pose_array, pose["max_force"]))
    elif "max_speed" in pose:
        logger.debug(f"Sending max_speed: {pose['max_speed']}")
        command_queue.put((move_type, pose_array, pose["max_speed"]))
    else:
        command_queue.put((move_type, pose_array))

    try:
        await wait_for_motion_complete(result_queue=result_queue, timeout=MOVE_TIMEOUT)
    except asyncio.TimeoutError:
        logger.error("Move timed out waiting for motion complete")
        command_queue.put(None)
        raise SystemExit(1)
    except RuntimeError as e:
        logger.error(f"Motion error: {e}")
        command_queue.put(None)
        raise SystemExit(1)

    logger.info("Target Reached!")
    await asyncio.sleep(POLL_INTERVAL)


async def toggle_air(
    command_queue: mp.Queue,
    result_queue: mp.Queue,
    index: drfl.GPIO_CTRLBOX_DIGITAL_INDEX = drfl.GPIO_CTRLBOX_DIGITAL_INDEX.Index_1,
    output: bool | None = None
) -> None:
    """
    Toggles the air valve on or off.
    """
    command_queue.put(("toggle_air", index, output))
    await wait_for_motion_complete(result_queue=result_queue)


async def load_config(filepath: str | Path) -> tuple[dict, str | None, dict]:
    """
    Load poses and sequences from a JSON config file.

    Returns:
        poses     - dict mapping pose name -> pose dict
        home      - name of the home pose (or None)
        sequences - dict mapping destination name -> list of steps
    """
    with open(filepath, "r") as f:
        data = json.load(f)

    if isinstance(data, list):
        poses = {p["name"]: p for p in data}
        return poses, None, {}

    poses = {p["name"]: p for p in data.get("poses", [])}
    home = data.get("home")
    sequences = data.get("sequences", {})
    return poses, home, sequences


async def run_sequence(
    steps: list,
    poses: dict,
    sequences: dict,
    command_queue: mp.Queue,
    result_queue: mp.Queue,
    visited: set | None = None
) -> None:
    """
    Execute a named sequence of pose moves, air toggles and other movement steps.

    Supported step types:
        - sequence: Run another named sequence.
        - pose: Move to a named pose.
        - air: Toggle the air valve.
        - wait: Wait for a specified duration.
        - wait_for_user_input: Wait for the user to press Enter.
        - place_down: Specific move to place an item down (uses force feedback).
        - calibrate: Measure the tray height using force feedback.
    """
    global _pickup_up

    if visited is None:
        visited = set()

    loop = asyncio.get_running_loop()

    for step in steps:
        step_type = step.get("type")

        if step_type == "sequence":
            name = step["name"]
            if name not in sequences:
                logger.error(f"Sequence '{name}' not found")
                raise SystemExit(1)
            if name in visited:
                logger.error(
                    f"Circular reference detected in sequence '{name}' "
                    f"is already in the call chain {visited}"
                )
                raise SystemExit(1)
            logger.info(f"Running sub-sequence: '{name}'")
            await run_sequence(
                sequences[name], poses, sequences,
                command_queue, result_queue,
                visited | {name},
            )

        elif step_type == "pose":
            name = step["name"]
            if name not in poses:
                logger.error(f"Pose '{name}' not found in loaded poses")
                raise SystemExit(1)
            await execute_poses(poses[name], command_queue, result_queue)

        elif step_type == "wait":
            seconds = step.get("seconds", 0)
            logger.info(f"Waiting {seconds}s...")
            await asyncio.sleep(seconds)

        elif step_type == "air":
            raw_index = step.get("index", 1)
            if raw_index == 1:
                index = drfl.GPIO_CTRLBOX_DIGITAL_INDEX.Index_1
            else:
                index = drfl.GPIO_CTRLBOX_DIGITAL_INDEX(raw_index)
            output = step.get("output", None)
            await toggle_air(command_queue, result_queue, index=index, output=output)
            state = "ON" if output else ("OFF" if output is False else "TOGGLE")
            logger.info(f"Air set: index={raw_index} → {state}")

        elif step_type == "wait_for_user_input":
            message = step.get("message", "Press Enter to continue...")
            await loop.run_in_executor(None, _prompt, message)

        elif step_type == "place_down":
            name = step["name"]
            if name not in poses:
                logger.error(f"Pose `{name}` not found in loaded poses")
                raise SystemExit(1)

            pose = copy.deepcopy(poses[name])
            await execute_poses(pose, command_queue, result_queue)

            coords = pose["pose_array"]
            coords[2] = coords[2] - FORCE_Z_AXIS_DOWN

            pose_force = {
                "name": "force down",
                "pose_array": list(coords),
                "move_type": "force",
                "max_force": MAX_FORCE_PLACE_DOWN,
            }
            await execute_poses(pose_force, command_queue, result_queue)

            coords = pose["pose_array"]
            coords[2] = coords[2] + (FORCE_Z_AXIS_DOWN * 2)
            _pickup_up = coords.copy()

        elif step_type == "calibrate":
            name = step["name"]
            if name not in poses:
                logger.error(f"Pose `{name}` not found in loaded poses")
                raise SystemExit(1)

            pose = copy.deepcopy(poses[name])
            await execute_poses(pose, command_queue, result_queue)

            coords = pose["pose_array"]
            coords[2] = coords[2] - FORCE_Z_AXIS_DOWN

            pose["pose_array"] = coords
            pose["move_type"] = "force"
            pose["max_force"] = MAX_FORCE_BOX

            logger.info(f"new pose: {pose['pose_array']}")

            await asyncio.sleep(0.5)
            await execute_poses(pose, command_queue, result_queue)

            logger.info(f"pose is: {_current_force_pose}")

        else:
            logger.warning(f"Unknown step type '{step_type}', skipping")


def _prompt(message: str) -> str:
    """Blocking stdin prompt, safe to run in an executor."""
    return input(message)


async def main() -> None:
    """
    Main entry point for the movement script.
    """
    loop = asyncio.get_event_loop()
    main_task = asyncio.current_task()

    def _handle_sigint():
        if _currently_moving:
            logger.info("SIGINT received, waiting for movement to finish...")
        main_task.cancel()

    loop.add_signal_handler(signal.SIGINT, _handle_sigint)

    filepath = sys.argv[1] if len(sys.argv) > 1 else "poses_V2.json"
    poses, home, sequences = await load_config(filepath)

    command_queue: mp.Queue = mp.Queue()
    result_queue: mp.Queue = mp.Queue()

    worker = mp.Process(
        target=robot_worker,
        args=(
            command_queue, result_queue,
            IP_DOOSAN, PORT_DOOSAN,
            DOOSAN_SPEED, DOOSAN_ACCELERATION,
            DOOSAN_LIN_SPEED, DOOSAN_LIN_ACCELERATION,
        ),
        daemon=True,
    )
    worker.start()

    result = await loop.run_in_executor(None, result_queue.get)
    if result != "ready":
        logger.error(f"Worker failed during setup: {result}")
        worker.terminate()
        raise SystemExit(1)

    logger.info("--- Ready to Move ---")

    # ── Wait for user to start ──────────────────────────────────────────────
    await loop.run_in_executor(None, _prompt, "Press Enter to start...")

    # ── Move to home ────────────────────────────────────────────────────────
    if home and home in poses:
        raw = await loop.run_in_executor(
            None, _prompt, f"Move to home '{home}'? [y/N]: "
        )
        if raw.strip().lower() in ("y", "yes"):
            logger.info(f"Moving to home position: '{home}'")
            await execute_poses(poses[home], command_queue, result_queue)
        else:
            logger.info("Skipping move to home.")

    # ── Main movement loop ──────────────────────────────────────────────────
    available = sorted(sequences.keys())
    while True:
        menu = "\n".join(f"  [{i+1}]: {name}" for i, name in enumerate(available))
        print(f"\n{menu}\n  [q]: quit")
        raw = await loop.run_in_executor(None, _prompt, "Select: ")
        choice = raw.strip().lower()

        if choice in ("q", "quit", "exit"):
            logger.info("Quit requested.")
            break

        if choice.isdigit() and 1 <= int(choice) <= len(available):
            dest = available[int(choice) - 1]
        else:
            logger.warning(f"Invalid selection '{choice}'.")
            continue

        logger.info(f"Starting sequence → '{dest}'")
        try:
            await run_sequence(
                sequences[dest], poses, sequences,
                command_queue, result_queue,
            )
        except SystemExit:
            raise

        logger.info(f"Sequence '{dest}' complete.")

        if home:
            logger.info(f"Returning to home position: '{home}'")
            await execute_poses(poses[home], command_queue, result_queue)

    # ── Graceful shutdown ───────────────────────────────────────────────────
    command_queue.put(None)
    worker.join()
    logger.info("Connection closed")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except asyncio.CancelledError:
        logger.info("Keyboard interrupt, exiting...")
        sys.exit(0)
    except Exception as e:
        logger.error(f"System error: {e}")
        sys.exit(1)
