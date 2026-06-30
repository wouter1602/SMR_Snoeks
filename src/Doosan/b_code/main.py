#!/usr/bin/env python3
"""
Doosan main control script — with TCP integration to SAM3.

Startup order:
    1. Start TCP server (comm.Link) and wait for SMA3 to connect.
    2. Start robot_worker process and wait for "ready".
    3. Announce "doosan_ready" + list of available sequences to SMA3.
    4. Main loop: wait for "run:<sequence_name>" commands from SMA3,
       execute them, and report "started:" / "done:" / "error:" back.

Supported step types in sequences:
    - sequence, pose, wait, air, wait_for_user_input, place_down, calibrate
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
from comm import Link

# ── Setup ──────────────────────────────────────────────────────────────
IP_DOOSAN = "192.168.0.50"
PORT_DOOSAN = 12345

# TCP link to SAM3
TCP_HOST = "0.0.0.0"
TCP_PORT = 9000
TCP_CONNECT_TIMEOUT = 120   # seconds to wait for SAM3 to connect

FORCE_Z_AXIS_DOWN = 50.0    # in mm

DOOSAN_SPEED = 20
DOOSAN_ACCELERATION = 20
DOOSAN_LIN_SPEED = 60
DOOSAN_LIN_ACCELERATION = 60
DOOSAN_LIN_SPEED_SLOW = 20
DOOSAN_LIN_ACCELERATION_SLOW = 20

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

# ── TCP bridge globals (set in main) ───────────────────────────────────
_tcp_queue: asyncio.Queue | None = None
_tcp_link: Link | None = None
_loop_ref: asyncio.AbstractEventLoop | None = None


def _on_tcp_message(msg: str) -> None:
    """Runs in Link's background thread — push into asyncio queue safely."""
    if _loop_ref is None or _tcp_queue is None:
        return
    logger.debug(f"[TCP] received: {msg}")
    _loop_ref.call_soon_threadsafe(_tcp_queue.put_nowait, msg)


def tcp_send(msg: str) -> None:
    """Send a message back to SAM3 (thread-safe)."""
    if _tcp_link and _tcp_link.is_connected():
        _tcp_link.send(msg)
    else:
        logger.warning(f"[TCP] not connected, dropping: {msg}")


async def wait_for_motion_complete(
    result_queue: mp.Queue,
    timeout: float = MOVE_TIMEOUT,
) -> None:
    """Wait for the robot motion to complete."""
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
    """Execute a single pose on the robot."""
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
    """Toggles the air valve on or off."""
    command_queue.put(("toggle_air", index, output))
    await wait_for_motion_complete(result_queue=result_queue)


async def load_config(filepath: str | Path) -> tuple[dict, str | None, dict]:
    """Load poses and sequences from a JSON config file."""
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
    """Execute a named sequence of pose moves, air toggles and other movement steps."""
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
    """Main entry point — TCP-driven control."""
    global _tcp_queue, _tcp_link, _loop_ref

    loop = asyncio.get_event_loop()
    _loop_ref = loop
    _tcp_queue = asyncio.Queue()

    main_task = asyncio.current_task()

    def _handle_sigint():
        if _currently_moving:
            logger.info("SIGINT received, waiting for movement to finish...")
        main_task.cancel()

    loop.add_signal_handler(signal.SIGINT, _handle_sigint)

    # ── 1. Bring up TCP link FIRST ──────────────────────────────────────
    _tcp_link = Link(
        role="server",
        host=TCP_HOST,
        port=TCP_PORT,
        on_message=_on_tcp_message,
    )
    _tcp_link.start()
    logger.info(f"[TCP] Waiting for SAM3 to connect on {TCP_HOST}:{TCP_PORT} ...")
    connected = await loop.run_in_executor(
        None, _tcp_link.wait_until_connected, TCP_CONNECT_TIMEOUT
    )
    if not connected:
        logger.error(f"[TCP] SAM3 did not connect within {TCP_CONNECT_TIMEOUT}s")
        _tcp_link.stop()
        raise SystemExit(1)
    logger.info("[TCP] SAM3 connected.")

    # ── 2. Load config ──────────────────────────────────────────────────
    filepath = sys.argv[1] if len(sys.argv) > 1 else "poses_V2.json"
    poses, home, sequences = await load_config(filepath)

    # ── 3. Start robot worker ───────────────────────────────────────────
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
        tcp_send(f"error:worker_setup:{result}")
        _tcp_link.stop()
        raise SystemExit(1)

    logger.info("--- Ready to Move ---")

    # ── 4. Optional: move to home before accepting commands ─────────────
    if home and home in poses:
        logger.info(f"Moving to home position: '{home}'")
        await execute_poses(poses[home], command_queue, result_queue)

    # ── 5. Announce readiness to SAM3 ───────────────────────────────────
    available = sorted(sequences.keys())
    logger.info(f"Available sequences: {available}")
    tcp_send("doosan_ready")
    tcp_send("sequences:" + ",".join(available))

    # ── 6. TCP-driven main loop ─────────────────────────────────────────
    while True:
        logger.info("[TCP] Waiting for next command from SAM3...")
        msg = await _tcp_queue.get()
        msg = msg.strip()

        if msg.lower() in ("quit", "exit", "shutdown"):
            logger.info("Quit requested by SAM3.")
            tcp_send("shutting_down")
            break

        # Expected format: "run:<sequence_name>"
        if msg.startswith("run:"):
            dest = msg.split(":", 1)[1].strip()
        else:
            logger.warning(f"[TCP] Unknown command: {msg}")
            tcp_send(f"error:unknown_command:{msg}")
            continue

        if dest not in sequences:
            logger.warning(f"Sequence '{dest}' not in {available}")
            tcp_send(f"error:no_such_sequence:{dest}")
            continue

        logger.info(f"Starting sequence → '{dest}'")
        tcp_send(f"started:{dest}")
        try:
            await run_sequence(
                sequences[dest], poses, sequences,
                command_queue, result_queue,
            )
        except SystemExit:
            tcp_send(f"error:sequence_failed:{dest}")
            raise
        except Exception as e:
            logger.error(f"Sequence '{dest}' crashed: {e}")
            tcp_send(f"error:sequence_failed:{dest}:{e}")
            continue

        logger.info(f"Sequence '{dest}' complete.")
        tcp_send(f"done:{dest}")

        if home and home in poses:
            logger.info(f"Returning to home position: '{home}'")
            await execute_poses(poses[home], command_queue, result_queue)
            tcp_send("done:home")

    # ── 7. Graceful shutdown ────────────────────────────────────────────
    command_queue.put(None)
    worker.join()
    if _tcp_link:
        _tcp_link.stop()
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
